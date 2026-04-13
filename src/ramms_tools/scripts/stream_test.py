#!/usr/bin/env python3
"""ramms-stream-test — test the RMSS streaming pipeline by sending frames to UE.

Modes:
  --synthetic     Send animated color-bar test frames (no extra deps).
  --capture-dir   Replay captured EXR+JSON data from CameraCapture plugin
                  (requires: pip install ramms-tools[exr]).
  --mask-dir      Replay single-channel mask EXRs as float32 on a dedicated
                  stream (requires: pip install ramms-tools[exr]).
  --motion-dir    Replay motion-vector EXRs (XY float) on a dedicated stream
                  (requires: pip install ramms-tools[exr]).

  --capture-dir, --mask-dir and --motion-dir may be combined to stream all
  data types simultaneously in lockstep.

Prerequisites in UE:
  - Start PIE
  - The streaming server must be running (URammsStreamingSubsystem::StartServer)
  - An actor needs URammsStreamSinkComponent (listens for IMAGE_DATA)
  - Optionally URammsStreamCameraBridge (auto-forwards to camera provider/UI)
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np

from ramms_tools.streaming.sender import StreamSender

logger = logging.getLogger(__name__)

# Default stream ID templates — {name} is the camera/source label, {index}
# is the 0-based camera index.  Users can override via CLI args.
DEFAULT_COLOR_ID = "camera/{name}/color"
DEFAULT_DEPTH_ID = "camera/{name}/depth"
DEFAULT_MASK_ID = "mask/{name}"
DEFAULT_MOTION_ID = "motion/{name}"


def _resolve_stream_id(template: str, *, name: str = "default",
                       index: int = 0) -> str:
    """Format a stream-ID template, substituting ``{name}`` and ``{index}``."""
    return template.format(name=name, index=index)


# ---------------------------------------------------------------------------
# Synthetic frame generator
# ---------------------------------------------------------------------------

def generate_test_frame(width: int, height: int, frame_num: int) -> np.ndarray:
    """Generate a synthetic BGRA8 test frame with moving color bars."""
    img = np.zeros((height, width, 4), dtype=np.uint8)
    offset = (frame_num * 4) % width

    bar_width = width // 8
    colors_bgra = [
        (255, 255, 255, 255),  # white
        (0, 255, 255, 255),    # yellow (BGR)
        (255, 255, 0, 255),    # cyan
        (0, 255, 0, 255),      # green
        (255, 0, 255, 255),    # magenta
        (0, 0, 255, 255),      # red
        (255, 0, 0, 255),      # blue
        (0, 0, 0, 255),        # black
    ]

    for i, color in enumerate(colors_bgra):
        x_start = ((i * bar_width) + offset) % width
        x_end = x_start + bar_width
        if x_end <= width:
            img[:, x_start:x_end] = color
        else:
            img[:, x_start:] = color
            img[:, :x_end - width] = color

    # Pulsing block in top-left corner as frame counter indicator
    block_size = 20
    shade = (frame_num * 3) % 256
    img[:block_size, :block_size * 4] = (shade, shade, shade, 255)

    return img


def run_synthetic(args: argparse.Namespace) -> None:
    """Send synthetic test frames to UE."""
    sender = StreamSender(args.host, args.port)
    print(f"Connecting to RMSS server at {args.host}:{args.port}...")
    sender.connect()
    print("Connected! Waiting for server to be ready...")
    time.sleep(0.2)  # Allow UE recv thread to start

    fps = args.fps
    frame_interval = 1.0 / fps if fps > 0 else 0
    width, height = args.width, args.height

    name = args.name or "synthetic"
    stream_id = _resolve_stream_id(args.color_id, name=name)

    print(f"Sending {args.num_frames} synthetic frames ({width}x{height}) "
          f"as '{stream_id}' at {fps} fps")

    try:
        for i in range(args.num_frames):
            t0 = time.monotonic()
            frame = generate_test_frame(width, height, i)

            meta = {
                "w": width, "h": height, "fmt": "bgra8",
                "frame": i, "source": "synthetic_test",
            }

            sender.send_image(
                channel=0,
                image_bytes=frame.tobytes(),
                width=width, height=height,
                fmt="bgra8",
                metadata=meta,
                stream_id=stream_id,
                name=f"{name} Color",
                material_params=args.material_params,
            )

            if (i + 1) % 30 == 0 or i == 0:
                print(f"  Sent frame {i + 1}/{args.num_frames} "
                      f"({len(frame.tobytes())} bytes)")

            elapsed = time.monotonic() - t0
            if frame_interval > elapsed:
                time.sleep(frame_interval - elapsed)

        print(f"\nDone! Sent {args.num_frames} frames.")
    finally:
        sender.disconnect()


# ---------------------------------------------------------------------------
# Capture directory replay (EXR → BGRA8)
# ---------------------------------------------------------------------------

def _load_openexr():
    """Import OpenEXR + Imath, exiting with a helpful message if unavailable."""
    try:
        import OpenEXR
        import Imath
        return OpenEXR, Imath
    except ImportError:
        print("ERROR: OpenEXR is required for EXR replay. Install with:")
        print("  pip install ramms-tools[exr]")
        sys.exit(1)


def load_exr_frame(
    exr_path: Path,
    want_depth: bool = False,
) -> tuple[np.ndarray, np.ndarray | None, int, int] | None:
    """Load an EXR and return (bgra8_array, depth_f32_or_None, width, height).

    Opens the file once, extracts RGB → BGRA8 (linear, no color space
    conversion — UE handles sRGB via the SRGB texture flag) and optionally
    depth (Depth channel, falling back to Alpha).
    """
    OpenEXR, Imath = _load_openexr()

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
    OpenEXR, Imath = _load_openexr()

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
    OpenEXR, Imath = _load_openexr()

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


def discover_capture_cameras(capture_dir: Path) -> list[tuple[str, str, Path]]:
    """Discover actor/camera pairs in a nested capture directory.

    Returns list of (actor_name, camera_name, camera_dir) tuples.
    """
    cameras = []
    for actor_dir in sorted(capture_dir.iterdir()):
        if not actor_dir.is_dir():
            continue
        for camera_dir in sorted(actor_dir.iterdir()):
            if not camera_dir.is_dir():
                continue
            if any(camera_dir.glob("frame_*.json")):
                cameras.append((actor_dir.name, camera_dir.name, camera_dir))
    return cameras


def discover_capture_frames(
    capture_dir: Path,
    camera_filter: str | None = None,
) -> list[tuple[Path, Path]]:
    """Discover (json, exr) frame pairs for a single camera directory or nested layout.

    Supports:
      Flat:     capture_dir/frame_NNNNNNN.{json,exr}        (single camera dir)
      Nested:   capture_dir/ActorName/CameraName/frame_*     (full capture tree)
      Mid-level: capture_dir/CameraName/frame_*              (actor-level dir)

    Args:
        camera_filter: If set, only include cameras whose path contains this
                       substring (case-insensitive). Matches against
                       "ActorName/CameraName".
    """
    # Try flat layout first (pointing at a single camera folder)
    json_files = sorted(capture_dir.glob("frame_*.json"))
    if json_files:
        pairs = []
        for jf in json_files:
            exr = jf.with_suffix(".exr")
            if exr.exists():
                pairs.append((jf, exr))
        return pairs

    # Try nested (ActorName/CameraName/) layout
    pairs = []
    filt = camera_filter.lower() if camera_filter else None
    for actor_dir in sorted(capture_dir.iterdir()):
        if not actor_dir.is_dir():
            continue
        for camera_dir in sorted(actor_dir.iterdir()):
            if not camera_dir.is_dir():
                continue
            if filt:
                full_name = f"{actor_dir.name}/{camera_dir.name}".lower()
                if filt not in full_name:
                    continue
            for jf in sorted(camera_dir.glob("frame_*.json")):
                exr = jf.with_suffix(".exr")
                if exr.exists():
                    pairs.append((jf, exr))

    # Try mid-level (CameraName/frame_*) — user pointed at actor folder
    if not pairs:
        for camera_dir in sorted(capture_dir.iterdir()):
            if not camera_dir.is_dir():
                continue
            if filt and filt not in camera_dir.name.lower():
                continue
            for jf in sorted(camera_dir.glob("frame_*.json")):
                exr = jf.with_suffix(".exr")
                if exr.exists():
                    pairs.append((jf, exr))

    return pairs


def discover_cameras_in_dir(
    capture_dir: Path,
    camera_filter: str | None = None,
) -> list[tuple[str, Path]]:
    """Discover individual camera directories within a capture directory.

    Returns list of (camera_label, camera_dir) where camera_label is
    'ActorName/CameraName' for nested layout, 'CameraName' for mid-level,
    or '.' for flat layout (single camera folder).
    """
    filt = camera_filter.lower() if camera_filter else None

    # Flat layout — the directory itself contains frames
    if any(capture_dir.glob("frame_*.json")):
        label = capture_dir.name
        if not filt or filt in label.lower():
            return [(label, capture_dir)]
        return []

    cameras: list[tuple[str, Path]] = []

    # Try nested (ActorName/CameraName/) layout
    for actor_dir in sorted(capture_dir.iterdir()):
        if not actor_dir.is_dir():
            continue
        for camera_dir in sorted(actor_dir.iterdir()):
            if not camera_dir.is_dir():
                continue
            if not any(camera_dir.glob("frame_*.json")):
                continue
            label = f"{actor_dir.name}/{camera_dir.name}"
            if filt and filt not in label.lower():
                continue
            cameras.append((label, camera_dir))

    # Try mid-level (CameraName/frame_*) — actor-level dir
    if not cameras:
        for camera_dir in sorted(capture_dir.iterdir()):
            if not camera_dir.is_dir():
                continue
            if not any(camera_dir.glob("frame_*.json")):
                continue
            label = camera_dir.name
            if filt and filt not in label.lower():
                continue
            cameras.append((label, camera_dir))

    return cameras


def _load_single_frame(
    ci: dict,
    frame_idx: int,
    want_depth: bool,
) -> dict | None:
    """Load one frame for one camera.  Returns a dict ready for sending, or None."""
    jf, exr_path = ci["frames"][frame_idx]

    with open(jf) as f:
        file_meta = json.load(f)

    result = load_exr_frame(exr_path, want_depth=want_depth)
    if result is None:
        return None

    bgra, depth, w, h = result

    meta: dict = {
        "w": w, "h": h, "fmt": "bgra8",
        "frame": file_meta.get("frame_number", frame_idx),
        "source": "capture_replay",
        "camera": file_meta.get("camera_id", ci["label"]),
    }

    if "intrinsics" in file_meta:
        intr = file_meta["intrinsics"]
        meta["intrinsics"] = {
            "fx": intr.get("focal_length_x", 0),
            "fy": intr.get("focal_length_y", 0),
            "cx": intr.get("principal_point_x", 0),
            "cy": intr.get("principal_point_y", 0),
        }

    xform_key = (
        "relative_transform" if "relative_transform" in file_meta
        else "world_transform"
    )
    if xform_key in file_meta:
        wt = file_meta[xform_key]
        loc = wt.get("location", [0, 0, 0])
        rot = wt.get("rotation", [0, 0, 0])
        meta["transform"] = {
            "x": loc[0], "y": loc[1], "z": loc[2],
            "pitch": rot[0], "yaw": rot[1], "roll": rot[2],
        }
        meta["transform_space"] = (
            "relative" if xform_key == "relative_transform" else "world"
        )

    return {
        "bgra": bgra.tobytes(),
        "depth": depth.tobytes() if depth is not None else None,
        "w": w,
        "h": h,
        "meta": meta,
    }


# Default number of frame-sets (one set = all cameras for one time-step) to
# keep buffered ahead of the send loop.
_DEFAULT_PREFETCH = 30


def _load_camera_chunk(
    ci: dict,
    start: int,
    end: int,
    want_depth: bool,
) -> list[dict | None]:
    """Load frames [start, end) for one camera sequentially.

    Called from a worker process — one process per camera, so file I/O and
    decompression run with independent GILs.  Returns list of loaded frame
    dicts (or None for failures).
    """
    loaded: list[dict | None] = []
    for frame_idx in range(start, end):
        loaded.append(_load_single_frame(ci, frame_idx, want_depth))
    return loaded


def _submit_chunk(
    pool: "ProcessPoolExecutor",
    camera_info: list[dict],
    chunk_start: int,
    chunk_end: int,
    want_depth: bool,
) -> "dict[Future, int]":
    """Submit one chunk load — one future per camera, returns {future: cam_idx}."""
    return {
        pool.submit(
            _load_camera_chunk, ci, chunk_start, chunk_end, want_depth,
        ): idx
        for idx, ci in enumerate(camera_info)
    }


def _collect_chunk(
    futures: "dict[Future, int]",
    num_cameras: int,
) -> list[list["dict | None"]]:
    """Block until all camera futures complete, return [cam_idx][local_frame]."""
    from concurrent.futures import as_completed
    result: list[list[dict | None]] = [[] for _ in range(num_cameras)]
    for future in as_completed(futures):
        idx = futures[future]
        result[idx] = future.result()
    return result


def run_capture_replay(args: argparse.Namespace) -> None:
    """Replay captured EXR+JSON data, decoding EXR to BGRA8 pixels.

    Uses double-buffered chunk loading: while chunk N is being sent, chunk N+1
    is loaded in the background by a persistent process pool (one process per
    camera).  ProcessPoolExecutor avoids GIL contention — loading in worker
    processes doesn't starve the main thread's socket sends.  Memory is bounded
    to roughly ``2 × chunk × cameras × frame_size`` (two chunks resident at
    the transition point).

    Supports single-camera (flat folder) and multi-camera (actor or full
    capture tree).  Each camera gets its own channel pair and meaningful
    stream IDs::

      camera 0 ("FL_Capture") → camera/FL_Capture/color, camera/FL_Capture/depth
      camera 1 ("Gripper")    → camera/Gripper/color,    camera/Gripper/depth

    Streams with the same camera label share a group ID; UE uses this group
    ID to auto-link color↔depth. The traditional ``channel`` / ``channel+100``
    pattern is retained only as the default channel assignment for backward
    compatibility, not as a requirement for auto-linking.

    When ``--mask-dir`` and/or ``--motion-dir`` are also provided, those
    EXRs are loaded and sent in lockstep on their dedicated channels.
    """
    from concurrent.futures import ProcessPoolExecutor
    import multiprocessing

    capture_dir = Path(args.capture_dir)
    if not capture_dir.is_dir():
        print(f"ERROR: Capture directory not found: {capture_dir}")
        sys.exit(1)

    camera_filter = getattr(args, "camera", None)

    # Discover cameras
    cameras = discover_cameras_in_dir(capture_dir, camera_filter=camera_filter)
    if not cameras:
        print(f"ERROR: No cameras with frame data found in {capture_dir}")
        print("  Expected layout: .../frame_NNNNNNN.{json,exr}")
        print("  Or nested:       .../ActorName/CameraName/frame_NNNNNNN.{json,exr}")
        sys.exit(1)

    # Build per-camera frame lists and stream ID assignments
    color_tpl = args.color_id
    depth_tpl = args.depth_id

    camera_info: list[dict] = []
    for i, (label, cam_dir) in enumerate(cameras):
        frames = []
        for jf in sorted(cam_dir.glob("frame_*.json")):
            exr = jf.with_suffix(".exr")
            if exr.exists():
                frames.append((jf, exr))
        if not frames:
            continue
        camera_info.append({
            "label": label,
            "frames": frames,
            "rgb_channel": i,
            "depth_channel": 100 + i,
            "group": label,
            "color_stream_id": _resolve_stream_id(color_tpl, name=label, index=i),
            "depth_stream_id": _resolve_stream_id(depth_tpl, name=label, index=i),
        })

    if not camera_info:
        print(f"ERROR: No frame pairs found for any camera in {capture_dir}")
        sys.exit(1)

    num_cameras = len(camera_info)
    num_frames_per_cam = min(len(ci["frames"]) for ci in camera_info)
    max_frames = args.num_frames if args.num_frames > 0 else 0
    if max_frames > 0:
        num_frames_per_cam = min(num_frames_per_cam, max_frames)

    print(f"Found {num_cameras} camera(s) in {capture_dir}:")
    for ci in camera_info:
        n = len(ci["frames"])
        print(f"  {ci['label']:40s}  {n:>4d} frames  "
              f"color→{ci['color_stream_id']}  "
              f"depth→{ci['depth_stream_id']}")

    # ── Mask setup (optional) ──────────────────────────────────────
    mask_dir = Path(args.mask_dir) if getattr(args, "mask_dir", None) else None
    mask_paths: list[Path] = []
    mask_name = args.name or "default"
    mask_stream_id = _resolve_stream_id(args.mask_id, name=mask_name)
    if mask_dir is not None:
        if not mask_dir.is_dir():
            print(f"ERROR: Mask directory not found: {mask_dir}")
            sys.exit(1)
        mask_paths = _discover_mask_frames(mask_dir)
        if not mask_paths:
            print(f"WARNING: No frame_*.exr files in {mask_dir}, masks disabled")
        else:
            if len(mask_paths) < num_frames_per_cam:
                print(f"WARNING: Mask directory has {len(mask_paths)} frames "
                      f"but cameras have {num_frames_per_cam}; "
                      f"clamping to {len(mask_paths)}")
                num_frames_per_cam = len(mask_paths)
            mask_paths = mask_paths[:num_frames_per_cam]
            print(f"  Masks: {len(mask_paths)} frames from {mask_dir} "
                  f"→ {mask_stream_id}")
    send_masks = len(mask_paths) > 0

    # ── Motion setup (optional) ────────────────────────────────────
    motion_dir = Path(args.motion_dir) if getattr(args, "motion_dir", None) else None
    motion_paths: list[Path] = []
    motion_stream_id = _resolve_stream_id(args.motion_id, name=mask_name)
    if motion_dir is not None:
        if not motion_dir.is_dir():
            print(f"ERROR: Motion directory not found: {motion_dir}")
            sys.exit(1)
        motion_paths = _discover_motion_frames(motion_dir)
        if not motion_paths:
            print(f"WARNING: No frame_*.exr files in {motion_dir}, motion disabled")
        else:
            if len(motion_paths) < num_frames_per_cam:
                print(f"WARNING: Motion directory has {len(motion_paths)} frames "
                      f"but expected {num_frames_per_cam}; "
                      f"clamping to {len(motion_paths)}")
                num_frames_per_cam = len(motion_paths)
            motion_paths = motion_paths[:num_frames_per_cam]
            print(f"  Motion: {len(motion_paths)} frames from {motion_dir} "
                  f"→ {motion_stream_id}")
    send_motion = len(motion_paths) > 0

    # If frame count was clamped by mask/motion, re-slice mask_paths too
    if send_masks and len(mask_paths) > num_frames_per_cam:
        mask_paths = mask_paths[:num_frames_per_cam]

    # ── Connect ─────────────────────────────────────────────────────
    sender = StreamSender(args.host, args.port)
    print(f"\nConnecting to RMSS server at {args.host}:{args.port}...")
    sender.connect()
    print("Connected! Waiting for server to be ready...")
    time.sleep(0.2)

    want_depth = args.send_depth
    prefetch = args.prefetch
    if prefetch <= 0:
        prefetch = num_frames_per_cam  # 0 = preload everything

    fps = args.fps
    frame_interval = 1.0 / fps if fps > 0 else 0
    loop = args.loop
    total_sent = 0
    frame_count = 0
    send_time_accum = 0.0

    # Build list of (chunk_start, chunk_end) ranges
    def build_chunks() -> list[tuple[int, int]]:
        chunks = []
        for cs in range(0, num_frames_per_cam, prefetch):
            chunks.append((cs, min(cs + prefetch, num_frames_per_cam)))
        return chunks

    extra_workers = (1 if send_masks else 0) + (1 if send_motion else 0)
    pool_workers = num_cameras + extra_workers
    print(f"Streaming (chunk_size={prefetch}, double-buffered)...\n")
    t_start = time.monotonic()

    pool = ProcessPoolExecutor(
        max_workers=pool_workers,
        mp_context=multiprocessing.get_context("spawn"),
    )

    try:
        while True:
            chunks = build_chunks()

            # Kick off loading of the first chunk
            next_futures = _submit_chunk(
                pool, camera_info, chunks[0][0], chunks[0][1], want_depth,
            )
            next_mask_future = (
                pool.submit(_load_mask_chunk, mask_paths, chunks[0][0], chunks[0][1])
                if send_masks else None
            )
            next_motion_future = (
                pool.submit(_load_motion_chunk, motion_paths, chunks[0][0], chunks[0][1])
                if send_motion else None
            )
            t_first_load = time.monotonic()

            for chunk_idx, (chunk_start, chunk_end) in enumerate(chunks):
                chunk_size = chunk_end - chunk_start

                # Wait for current chunk to finish loading
                loaded_chunk = _collect_chunk(next_futures, num_cameras)
                loaded_masks = (
                    next_mask_future.result() if next_mask_future else None
                )
                loaded_motions = (
                    next_motion_future.result() if next_motion_future else None
                )
                load_elapsed = time.monotonic() - t_first_load
                load_fps = chunk_size / max(load_elapsed, 0.001)
                print(f"  Loaded chunk [{chunk_start}..{chunk_end}) "
                      f"in {load_elapsed:.2f}s ({load_fps:.0f} frames/s)")

                # Immediately kick off loading the NEXT chunk (double-buffer)
                next_chunk_idx = chunk_idx + 1
                if next_chunk_idx < len(chunks):
                    nc_start, nc_end = chunks[next_chunk_idx]
                    next_futures = _submit_chunk(
                        pool, camera_info, nc_start, nc_end, want_depth,
                    )
                    next_mask_future = (
                        pool.submit(_load_mask_chunk, mask_paths, nc_start, nc_end)
                        if send_masks else None
                    )
                    next_motion_future = (
                        pool.submit(_load_motion_chunk, motion_paths, nc_start, nc_end)
                        if send_motion else None
                    )
                    t_first_load = time.monotonic()

                # Send current chunk — pure network I/O
                t_chunk_send = time.monotonic()
                for local_idx in range(chunk_size):
                    t0 = time.monotonic()

                    for cam_idx, ci in enumerate(camera_info):
                        frame = loaded_chunk[cam_idx][local_idx]
                        if frame is None:
                            continue

                        sender.send_image(
                            channel=ci["rgb_channel"],
                            image_bytes=frame["bgra"],
                            width=frame["w"], height=frame["h"],
                            fmt="bgra8",
                            metadata=frame["meta"],
                            group=ci["group"],
                            role="color",
                            stream_id=ci["color_stream_id"],
                            name=f"{ci['label']} Color",
                            material_params=args.material_params,
                        )

                        if want_depth and frame["depth"] is not None:
                            depth_meta = dict(frame["meta"])
                            depth_meta["fmt"] = "float32"
                            depth_meta["unit"] = "cm"
                            sender.send_depth(
                                channel=ci["depth_channel"],
                                depth_bytes=frame["depth"],
                                width=frame["w"], height=frame["h"],
                                metadata=depth_meta,
                                group=ci["group"],
                                role="depth",
                                stream_id=ci["depth_stream_id"],
                                name=f"{ci['label']} Depth",
                            )

                    # Send mask frame (if available for this index)
                    if loaded_masks is not None:
                        mf = loaded_masks[local_idx]
                        if mf is not None:
                            mask_meta = {
                                "w": mf["w"], "h": mf["h"],
                                "source": "mask_replay",
                            }
                            sender.send_mask(
                                channel=200,
                                mask_bytes=mf["mask"],
                                width=mf["w"], height=mf["h"],
                                metadata=mask_meta,
                                stream_id=mask_stream_id,
                                name=f"{mask_name} Mask",
                                material_params=args.mask_material_params,
                            )

                    # Send motion-vector frame (if available for this index)
                    if loaded_motions is not None:
                        mv = loaded_motions[local_idx]
                        if mv is not None:
                            motion_meta = {
                                "w": mv["w"], "h": mv["h"],
                                "fmt": "rg32f",
                                "source": "motion_replay",
                            }
                            sender.send_motion(
                                channel=300,
                                motion_bytes=mv["motion"],
                                width=mv["w"], height=mv["h"],
                                metadata=motion_meta,
                                stream_id=motion_stream_id,
                                name=f"{mask_name} Motion",
                            )

                    total_sent += num_cameras + (1 if send_masks else 0) + (1 if send_motion else 0)
                    frame_count += 1

                    if (frame_count - 1) % 10 == 0 or frame_count == 1:
                        cur_send = send_time_accum + (time.monotonic() - t_chunk_send)
                        send_fps = frame_count / max(cur_send, 0.001)
                        streams = f"{num_cameras} cam"
                        if send_masks:
                            streams += " + mask"
                        if send_motion:
                            streams += " + motion"
                        print(f"  Frame {frame_count}/{num_frames_per_cam} "
                              f"({streams}, {total_sent} sends, "
                              f"tx {send_fps:.1f} fps)")

                    elapsed = time.monotonic() - t0
                    if frame_interval > elapsed:
                        time.sleep(frame_interval - elapsed)

                send_time_accum += time.monotonic() - t_chunk_send
                del loaded_chunk

            if not loop:
                break
            print(f"  Looping... ({total_sent} total sends so far)")

        elapsed_total = time.monotonic() - t_start
        send_fps = frame_count / max(send_time_accum, 0.001)
        print(f"\nDone! Sent {frame_count} frames × {num_cameras} cameras "
              f"= {total_sent} total in {elapsed_total:.1f}s "
              f"(tx {send_fps:.1f} fps, "
              f"overall {frame_count / max(elapsed_total, 0.001):.1f} fps)")
    finally:
        sender.disconnect()
        pool.shutdown(wait=False)


# ---------------------------------------------------------------------------
# Mask-only replay pipeline
# ---------------------------------------------------------------------------

def _discover_mask_frames(
    mask_dir: Path,
) -> list[Path]:
    """Find mask EXR files in a flat directory, sorted by name."""
    return sorted(mask_dir.glob("frame_*.exr"))


def _load_single_mask(exr_path: Path) -> dict | None:
    """Load one mask frame.  Returns a dict ready for sending, or None."""
    result = load_mask_exr(exr_path)
    if result is None:
        return None
    mask_f32, w, h = result
    # Convert float32 mask IDs (0.0, 1.0, 2.0, …) to uint8 for mono8 transport.
    # Values >255 are clamped — typical mask sets have far fewer classes.
    mask_u8 = np.clip(mask_f32, 0, 255).astype(np.uint8)
    return {
        "mask": mask_u8.tobytes(),
        "w": w,
        "h": h,
    }


def _load_mask_chunk(
    exr_paths: list[Path],
    start: int,
    end: int,
) -> list[dict | None]:
    """Load mask frames [start, end) sequentially.  Called from a worker."""
    return [_load_single_mask(exr_paths[i]) for i in range(start, end)]


def run_mask_replay(args: argparse.Namespace) -> None:
    """Replay mask EXRs from a masks directory as mono8 mask data.

    Sends each mask frame on a dedicated stream using ``send_mask`` so
    that UE receives uint8 mask IDs (0, 1, 2, …) as mono8 via FRAME_DATA.
    """
    from concurrent.futures import ProcessPoolExecutor
    import multiprocessing

    mask_dir = Path(args.mask_dir)
    if not mask_dir.is_dir():
        print(f"ERROR: Mask directory not found: {mask_dir}")
        sys.exit(1)

    exr_paths = _discover_mask_frames(mask_dir)
    if not exr_paths:
        print(f"ERROR: No frame_*.exr files found in {mask_dir}")
        sys.exit(1)

    num_frames = len(exr_paths)
    max_frames = args.num_frames if args.num_frames > 0 else 0
    if max_frames > 0:
        num_frames = min(num_frames, max_frames)
        exr_paths = exr_paths[:num_frames]

    name = args.name or "default"
    stream_id = _resolve_stream_id(args.mask_id, name=name)
    print(f"Found {num_frames} mask frames in {mask_dir}")
    print(f"  Sending as '{stream_id}' (mono8)")

    # ── Connect ─────────────────────────────────────────────────────
    sender = StreamSender(args.host, args.port)
    print(f"\nConnecting to RMSS server at {args.host}:{args.port}...")
    sender.connect()
    print("Connected!")
    time.sleep(0.2)

    fps = args.fps
    frame_interval = 1.0 / fps if fps > 0 else 0
    loop = args.loop
    prefetch = args.prefetch
    if prefetch <= 0:
        prefetch = num_frames

    total_sent = 0
    frame_count = 0
    send_time_accum = 0.0

    def build_chunks() -> list[tuple[int, int]]:
        chunks = []
        for cs in range(0, num_frames, prefetch):
            chunks.append((cs, min(cs + prefetch, num_frames)))
        return chunks

    print(f"Streaming masks (chunk_size={prefetch})...\n")
    t_start = time.monotonic()

    pool = ProcessPoolExecutor(
        max_workers=1,
        mp_context=multiprocessing.get_context("spawn"),
    )

    try:
        while True:
            chunks = build_chunks()

            next_future = pool.submit(
                _load_mask_chunk, exr_paths, chunks[0][0], chunks[0][1],
            )
            t_load = time.monotonic()

            for chunk_idx, (chunk_start, chunk_end) in enumerate(chunks):
                chunk_size = chunk_end - chunk_start

                loaded = next_future.result()
                load_elapsed = time.monotonic() - t_load
                load_fps = chunk_size / max(load_elapsed, 0.001)
                print(f"  Loaded chunk [{chunk_start}..{chunk_end}) "
                      f"in {load_elapsed:.2f}s ({load_fps:.0f} frames/s)")

                # Kick off next chunk
                next_chunk_idx = chunk_idx + 1
                if next_chunk_idx < len(chunks):
                    nc_start, nc_end = chunks[next_chunk_idx]
                    next_future = pool.submit(
                        _load_mask_chunk, exr_paths, nc_start, nc_end,
                    )
                    t_load = time.monotonic()

                # Send current chunk
                t_chunk_send = time.monotonic()
                for local_idx in range(chunk_size):
                    t0 = time.monotonic()

                    frame = loaded[local_idx]
                    if frame is not None:
                        meta = {
                            "w": frame["w"],
                            "h": frame["h"],
                            "source": "mask_replay",
                        }
                        sender.send_mask(
                            channel=200,
                            mask_bytes=frame["mask"],
                            width=frame["w"], height=frame["h"],
                            metadata=meta,
                            stream_id=stream_id,
                            name=f"{name} Mask",
                            material_params=args.mask_material_params,
                        )

                    total_sent += 1
                    frame_count += 1

                    if (frame_count - 1) % 10 == 0 or frame_count == 1:
                        cur_send = send_time_accum + (time.monotonic() - t_chunk_send)
                        send_fps_cur = frame_count / max(cur_send, 0.001)
                        print(f"  Frame {frame_count}/{num_frames} "
                              f"({total_sent} sends, tx {send_fps_cur:.1f} fps)")

                    elapsed = time.monotonic() - t0
                    if frame_interval > elapsed:
                        time.sleep(frame_interval - elapsed)

                send_time_accum += time.monotonic() - t_chunk_send
                del loaded

            if not loop:
                break
            print(f"  Looping... ({total_sent} total sends so far)")

        elapsed_total = time.monotonic() - t_start
        send_fps = frame_count / max(send_time_accum, 0.001)
        print(f"\nDone! Sent {frame_count} mask frames "
              f"in {elapsed_total:.1f}s "
              f"(tx {send_fps:.1f} fps, "
              f"overall {frame_count / max(elapsed_total, 0.001):.1f} fps)")
    finally:
        sender.disconnect()
        pool.shutdown(wait=False)


# ---------------------------------------------------------------------------
# Motion-vector-only replay pipeline
# ---------------------------------------------------------------------------

def _discover_motion_frames(motion_dir: Path) -> list[Path]:
    """Find motion-vector EXR files in a flat directory, sorted by name."""
    return sorted(motion_dir.glob("frame_*.exr"))


def _load_single_motion(exr_path: Path) -> dict | None:
    """Load one motion-vector frame.  Returns a dict ready for sending, or None."""
    result = load_motion_exr(exr_path)
    if result is None:
        return None
    motion, w, h = result
    return {
        "motion": motion.tobytes(),
        "w": w,
        "h": h,
    }


def _load_motion_chunk(
    exr_paths: list[Path],
    start: int,
    end: int,
) -> list[dict | None]:
    """Load motion frames [start, end) sequentially.  Called from a worker."""
    return [_load_single_motion(exr_paths[i]) for i in range(start, end)]


def run_motion_replay(args: argparse.Namespace) -> None:
    """Replay motion-vector EXRs as float32 XY data on a dedicated stream.

    Sends each frame using ``send_motion`` (FRAME_MOTION message type) so
    that UE receives raw float32 velocity vectors.
    """
    from concurrent.futures import ProcessPoolExecutor
    import multiprocessing

    motion_dir = Path(args.motion_dir)
    if not motion_dir.is_dir():
        print(f"ERROR: Motion directory not found: {motion_dir}")
        sys.exit(1)

    exr_paths = _discover_motion_frames(motion_dir)
    if not exr_paths:
        print(f"ERROR: No frame_*.exr files found in {motion_dir}")
        sys.exit(1)

    num_frames = len(exr_paths)
    max_frames = args.num_frames if args.num_frames > 0 else 0
    if max_frames > 0:
        num_frames = min(num_frames, max_frames)
        exr_paths = exr_paths[:num_frames]

    name = args.name or "default"
    stream_id = _resolve_stream_id(args.motion_id, name=name)
    print(f"Found {num_frames} motion-vector frames in {motion_dir}")
    print(f"  Sending as '{stream_id}' (rg32f)")

    # ── Connect ─────────────────────────────────────────────────────
    sender = StreamSender(args.host, args.port)
    print(f"\nConnecting to RMSS server at {args.host}:{args.port}...")
    sender.connect()
    print("Connected!")
    time.sleep(0.2)

    fps = args.fps
    frame_interval = 1.0 / fps if fps > 0 else 0
    loop = args.loop
    prefetch = args.prefetch
    if prefetch <= 0:
        prefetch = num_frames

    total_sent = 0
    frame_count = 0
    send_time_accum = 0.0

    def build_chunks() -> list[tuple[int, int]]:
        chunks = []
        for cs in range(0, num_frames, prefetch):
            chunks.append((cs, min(cs + prefetch, num_frames)))
        return chunks

    print(f"Streaming motion vectors (chunk_size={prefetch})...\n")
    t_start = time.monotonic()

    pool = ProcessPoolExecutor(
        max_workers=1,
        mp_context=multiprocessing.get_context("spawn"),
    )

    try:
        while True:
            chunks = build_chunks()

            next_future = pool.submit(
                _load_motion_chunk, exr_paths, chunks[0][0], chunks[0][1],
            )
            t_load = time.monotonic()

            for chunk_idx, (chunk_start, chunk_end) in enumerate(chunks):
                chunk_size = chunk_end - chunk_start

                loaded = next_future.result()
                load_elapsed = time.monotonic() - t_load
                load_fps = chunk_size / max(load_elapsed, 0.001)
                print(f"  Loaded chunk [{chunk_start}..{chunk_end}) "
                      f"in {load_elapsed:.2f}s ({load_fps:.0f} frames/s)")

                next_chunk_idx = chunk_idx + 1
                if next_chunk_idx < len(chunks):
                    nc_start, nc_end = chunks[next_chunk_idx]
                    next_future = pool.submit(
                        _load_motion_chunk, exr_paths, nc_start, nc_end,
                    )
                    t_load = time.monotonic()

                t_chunk_send = time.monotonic()
                for local_idx in range(chunk_size):
                    t0 = time.monotonic()

                    frame = loaded[local_idx]
                    if frame is not None:
                        meta = {
                            "w": frame["w"],
                            "h": frame["h"],
                            "fmt": "rg32f",
                            "source": "motion_replay",
                        }
                        sender.send_motion(
                            channel=300,
                            motion_bytes=frame["motion"],
                            width=frame["w"], height=frame["h"],
                            metadata=meta,
                            stream_id=stream_id,
                            name=f"{name} Motion",
                        )

                    total_sent += 1
                    frame_count += 1

                    if (frame_count - 1) % 10 == 0 or frame_count == 1:
                        cur_send = send_time_accum + (time.monotonic() - t_chunk_send)
                        send_fps_cur = frame_count / max(cur_send, 0.001)
                        print(f"  Frame {frame_count}/{num_frames} "
                              f"({total_sent} sends, tx {send_fps_cur:.1f} fps)")

                    elapsed = time.monotonic() - t0
                    if frame_interval > elapsed:
                        time.sleep(frame_interval - elapsed)

                send_time_accum += time.monotonic() - t_chunk_send
                del loaded

            if not loop:
                break
            print(f"  Looping... ({total_sent} total sends so far)")

        elapsed_total = time.monotonic() - t_start
        send_fps = frame_count / max(send_time_accum, 0.001)
        print(f"\nDone! Sent {frame_count} motion frames "
              f"in {elapsed_total:.1f}s "
              f"(tx {send_fps:.1f} fps, "
              f"overall {frame_count / max(elapsed_total, 0.001):.1f} fps)")
    finally:
        sender.disconnect()
        pool.shutdown(wait=False)


def _parse_material_params(raw: list[str]) -> dict[str, float] | None:
    """Parse ``['Key=1.0', 'Foo=2']`` into ``{'Key': 1.0, 'Foo': 2.0}``."""
    if not raw:
        return None
    result: dict[str, float] = {}
    for item in raw:
        if "=" not in item:
            print(f"ERROR: invalid material param '{item}' — expected KEY=VALUE")
            sys.exit(1)
        key, _, val = item.partition("=")
        key = key.strip()
        if not key:
            print(f"ERROR: material param '{item}' has an empty key")
            sys.exit(1)
        try:
            result[key] = float(val.strip())
        except ValueError:
            print(f"ERROR: material param value '{val}' is not a number")
            sys.exit(1)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="ramms-stream-test",
        description="Test the RMSS streaming pipeline by sending frames to UE",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="RMSS server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=30030,
                        help="RMSS server port (default: 30030)")
    parser.add_argument("--color-id", default=DEFAULT_COLOR_ID,
                        metavar="TEMPLATE",
                        help="Stream ID template for color streams "
                             f"(default: {DEFAULT_COLOR_ID}). "
                             "Supports {name} (camera label) and {index} placeholders.")
    parser.add_argument("--depth-id", default=DEFAULT_DEPTH_ID,
                        metavar="TEMPLATE",
                        help="Stream ID template for depth streams "
                             f"(default: {DEFAULT_DEPTH_ID}).")
    parser.add_argument("--mask-id", default=DEFAULT_MASK_ID,
                        metavar="TEMPLATE",
                        help="Stream ID template for mask streams "
                             f"(default: {DEFAULT_MASK_ID}).")
    parser.add_argument("--motion-id", default=DEFAULT_MOTION_ID,
                        metavar="TEMPLATE",
                        help="Stream ID template for motion streams "
                             f"(default: {DEFAULT_MOTION_ID}).")
    parser.add_argument("--name", default=None, metavar="NAME",
                        help="Default name for the {name} placeholder when no "
                             "camera label is available (e.g. synthetic mode). "
                             "Defaults to 'synthetic' or 'default' depending on mode.")
    parser.add_argument("--fps", type=float, default=30.0,
                        help="Target frame rate (default: 30)")
    parser.add_argument("-n", "--num-frames", type=int, default=300,
                        help="Number of frames to send (default: 300, 0=all)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable debug logging")

    # Synthetic mode options
    parser.add_argument("--width", type=int, default=640,
                        help="Synthetic frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480,
                        help="Synthetic frame height (default: 480)")

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--synthetic", action="store_true",
                      help="Send synthetic color-bar test frames")

    parser.add_argument("--capture-dir", metavar="DIR",
                        help="Replay captured EXR+JSON frames from CameraCapture")
    parser.add_argument("--mask-dir", metavar="DIR",
                        help="Replay mask EXRs (single-channel Y) as float32 on a "
                             "dedicated stream (can combine with --capture-dir)")
    parser.add_argument("--motion-dir", metavar="DIR",
                        help="Replay motion-vector EXRs (XY float) on a dedicated "
                             "stream (can combine with --capture-dir)")

    # Capture replay options
    parser.add_argument("--send-depth", action="store_true",
                        help="Also send depth from EXR alpha channel.")
    parser.add_argument("--loop", action="store_true",
                        help="Loop capture replay continuously")
    parser.add_argument("--camera", metavar="FILTER",
                        help="Filter cameras by substring (e.g. 'FL_Capture', 'Gripper')")
    parser.add_argument("--list-cameras", action="store_true",
                        help="List available cameras in capture dir and exit")
    parser.add_argument("--prefetch", type=int, default=_DEFAULT_PREFETCH,
                        help=f"Frame-sets to buffer ahead of sending "
                             f"(default: {_DEFAULT_PREFETCH}). "
                             f"Use 0 to preload ALL frames into memory.")

    # Material scalar parameters forwarded to UE MIDs
    parser.add_argument("--material-param", action="append", default=[],
                        metavar="KEY=VALUE",
                        help="Material scalar parameter sent with color/depth "
                             "streams (repeatable, e.g. --material-param "
                             "NumSegmentIDs=5)")
    parser.add_argument("--mask-material-param", action="append", default=[],
                        metavar="KEY=VALUE",
                        help="Material scalar parameter sent with mask streams "
                             "(repeatable, e.g. --mask-material-param "
                             "NumSegmentIDs=5)")

    args = parser.parse_args()

    # Parse KEY=VALUE material param args into dicts (or None if empty)
    args.material_params = _parse_material_params(args.material_param)
    args.mask_material_params = _parse_material_params(args.mask_material_param)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    if args.list_cameras:
        if not args.capture_dir:
            print("ERROR: --list-cameras requires --capture-dir")
            sys.exit(1)
        cap = Path(args.capture_dir)
        cameras = discover_capture_cameras(cap)
        if not cameras:
            print(f"No cameras found in {cap}")
            sys.exit(1)
        print(f"Cameras in {cap}:")
        for actor, cam, cam_dir in cameras:
            n = len(list(cam_dir.glob("frame_*.json")))
            print(f"  {actor}/{cam}  ({n} frames)")
        sys.exit(0)

    if args.synthetic:
        if args.capture_dir or args.mask_dir or args.motion_dir:
            print("ERROR: --synthetic cannot be combined with "
                  "--capture-dir, --mask-dir, or --motion-dir")
            sys.exit(1)
        run_synthetic(args)
    elif args.capture_dir:
        # capture-dir mode: masks and motion are integrated into the same loop
        run_capture_replay(args)
    elif args.mask_dir and not args.motion_dir:
        run_mask_replay(args)
    elif args.motion_dir and not args.mask_dir:
        run_motion_replay(args)
    elif args.mask_dir and args.motion_dir:
        print("ERROR: --mask-dir + --motion-dir without --capture-dir is not "
              "supported. Use --capture-dir to stream them together.")
        sys.exit(1)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
