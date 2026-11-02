"""EXR decoders for CameraCapture's serialized captures (colour, depth,
masks, motion vectors).  Needs OpenEXR (``pip install ramms-tools[exr]``):
each raises ImportError without it."""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


def require_openexr():
    """Import OpenEXR + Imath; raise ImportError, saying how to install them, if
    they are missing."""
    try:
        import OpenEXR
        import Imath
    except ImportError as e:
        raise ImportError(
            "OpenEXR is required to read EXR captures: pip install ramms-tools[exr]"
        ) from e
    return OpenEXR, Imath


def load_exr_frame(
    exr_path: Path,
    want_depth: bool = False,
) -> tuple[np.ndarray, np.ndarray | None, int, int] | None:
    """Load an EXR and return (bgra8_array, depth_f32_or_None, width, height).

    Opens the file once, extracts RGB → BGRA8 (linear, no color space
    conversion — UE handles sRGB via the SRGB texture flag) and optionally
    depth (Depth channel, falling back to Alpha).
    """
    OpenEXR, Imath = require_openexr()

    try:
        exr_file = OpenEXR.InputFile(str(exr_path))
    except Exception as e:
        logger.warning("Failed to open %s: %s", exr_path, e)
        return None

    header = exr_file.header()
    dw = header["dataWindow"]
    w = dw.max.x - dw.min.x + 1
    h = dw.max.y - dw.min.y + 1
    channels = header["channels"].keys()

    if not all(ch in channels for ch in ("R", "G", "B")):
        logger.warning("%s missing RGB channels (has: %s)", exr_path, list(channels))
        return None

    # Assemble BGRA8 with minimal allocations
    bgra = np.empty((h, w, 4), dtype=np.uint8)
    bgra[:, :, 3] = 255
    scratch = np.empty((h, w), dtype=np.float32)
    pt = Imath.PixelType(Imath.PixelType.FLOAT)

    b_buf = exr_file.channel("B", pt)
    g_buf = exr_file.channel("G", pt)
    r_buf = exr_file.channel("R", pt)

    # Detect value range from ALL channels to decide scaling.
    # Linear HDR data lives in [0,1] (with some headroom) → needs ×255.
    # Pre-quantized data is already [0,255] → no scaling needed.
    first = np.frombuffer(b_buf, dtype=np.float32).reshape((h, w))
    g_arr = np.frombuffer(g_buf, dtype=np.float32).reshape((h, w))
    r_arr = np.frombuffer(r_buf, dtype=np.float32).reshape((h, w))
    rgb_max = max(float(first.max()), float(g_arr.max()), float(r_arr.max()))
    needs_scale = rgb_max <= 2.0
    scale = np.float32(255.0) if needs_scale else np.float32(1.0)
    if logger.isEnabledFor(logging.DEBUG):
        logger.debug(
            "%s: rgb_max=%.3f → %s",
            exr_path.name, rgb_max,
            "scaling ×255 (linear)" if needs_scale else "no scale (already 0-255)",
        )

    for buf, ch in ((b_buf, 0), (g_buf, 1), (r_buf, 2)):
        raw = np.frombuffer(buf, dtype=np.float32).reshape((h, w))
        np.multiply(raw, scale, out=scratch)
        np.clip(scratch, 0, 255, out=scratch)
        np.copyto(bgra[:, :, ch], scratch, casting="unsafe")

    depth = None
    if want_depth:
        if "Depth" in channels:
            depth = np.frombuffer(
                exr_file.channel("Depth", pt), dtype=np.float32
            ).reshape((h, w))
        elif "A" in channels:
            depth = np.frombuffer(
                exr_file.channel("A", pt), dtype=np.float32
            ).reshape((h, w))
        # Filter out default alpha (1.0 everywhere = no real depth data)
        if depth is not None:
            dmin, dmax = float(depth.min()), float(depth.max())
            if abs(dmin - 1.0) < 1e-5 and abs(dmax - 1.0) < 1e-5:
                depth = None

    return bgra, depth, w, h


def load_mask_exr(exr_path: Path) -> tuple[np.ndarray, int, int] | None:
    """Load a single-channel mask EXR and return (mask_f32, width, height).

    Reads the Y channel as UINT and converts to float32 so that mask IDs
    (0, 1, 2, …) are preserved as exact float values without normalization.
    """
    OpenEXR, Imath = require_openexr()

    try:
        exr_file = OpenEXR.InputFile(str(exr_path))
    except Exception as e:
        logger.warning("Failed to open mask %s: %s", exr_path, e)
        return None

    header = exr_file.header()
    dw = header["dataWindow"]
    w = dw.max.x - dw.min.x + 1
    h = dw.max.y - dw.min.y + 1
    channels = header["channels"].keys()

    if "Y" not in channels:
        logger.warning("Mask EXR %s has no Y channel (has: %s)", exr_path, list(channels))
        return None

    pt_uint = Imath.PixelType(Imath.PixelType.UINT)
    y_buf = exr_file.channel("Y", pt_uint)
    raw = np.frombuffer(y_buf, dtype=np.uint32).reshape((h, w))

    if logger.isEnabledFor(logging.DEBUG):
        logger.debug("Mask EXR %s: shape=%s min=%s max=%s", exr_path.name, raw.shape, raw.min(), raw.max())
        unique, counts = np.unique(raw, return_counts=True)
        logger.debug(
            "Mask EXR %s: unique_values(%d)=%s counts=%s",
            exr_path.name, len(unique),
            unique[:20], counts[:20],
        )

    return raw.astype(np.float32), w, h


def load_motion_exr(exr_path: Path) -> tuple[np.ndarray, int, int] | None:
    """Load a motion-vector EXR and return (motion_f32, width, height).

    Expects an EXR with at least two float channels representing XY screen-
    space velocity.  Looks for ``(X, Y)``, ``(U, V)``, ``(R, G)`` or
    ``(forward.u, forward.v)`` channel pairs, in that order.

    Returns a contiguous float32 array of shape (H, W, 2) — two floats per
    pixel (X/U velocity, Y/V velocity).
    """
    OpenEXR, Imath = require_openexr()

    try:
        exr_file = OpenEXR.InputFile(str(exr_path))
    except Exception as e:
        logger.warning("Failed to open motion EXR %s: %s", exr_path, e)
        return None

    header = exr_file.header()
    dw = header["dataWindow"]
    w = dw.max.x - dw.min.x + 1
    h = dw.max.y - dw.min.y + 1
    channels = set(header["channels"].keys())

    # Try common channel-name pairs for motion vectors
    pairs = [
        ("X", "Y"),
        ("U", "V"),
        ("R", "G"),
        ("forward.u", "forward.v"),
    ]
    ch_x = ch_y = None
    for a, b in pairs:
        if a in channels and b in channels:
            ch_x, ch_y = a, b
            break

    if ch_x is None:
        logger.warning(
            "Motion EXR %s has no recognized velocity pair (has: %s)",
            exr_path, sorted(channels),
        )
        return None

    pt = Imath.PixelType(Imath.PixelType.FLOAT)
    vx = np.frombuffer(exr_file.channel(ch_x, pt), dtype=np.float32).reshape((h, w))
    vy = np.frombuffer(exr_file.channel(ch_y, pt), dtype=np.float32).reshape((h, w))

    motion = np.empty((h, w, 2), dtype=np.float32)
    motion[:, :, 0] = vx
    motion[:, :, 1] = vy

    logger.debug(
        "Motion EXR %s (%s/%s): shape=%s vx=[%.4f..%.4f] vy=[%.4f..%.4f]",
        exr_path.name, ch_x, ch_y, motion.shape,
        float(vx.min()), float(vx.max()),
        float(vy.min()), float(vy.max()),
    )

    return motion, w, h
