#!/usr/bin/env python3
"""ramms-stream-test — test the RMSS streaming pipeline by sending frames to UE.

Modes:
  --synthetic     Send animated color-bar test frames (no extra deps).
  --capture-dir   Replay captured EXR+JSON data from CameraCapture plugin
                  (requires: pip install ramms-tools[exr]).

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
    channel = args.channel

    print(f"Sending {args.num_frames} synthetic frames ({width}x{height}) "
          f"on channel {channel} at {fps} fps")

    try:
        for i in range(args.num_frames):
            t0 = time.monotonic()
            frame = generate_test_frame(width, height, i)

            meta = {
                "w": width, "h": height, "fmt": "bgra8",
                "frame": i, "source": "synthetic_test",
            }

            sender.send_image(
                channel=channel,
                image_bytes=frame.tobytes(),
                width=width, height=height,
                fmt="bgra8",
                metadata=meta,
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


def load_exr_as_bgra8(exr_path: Path) -> tuple[np.ndarray, int, int] | None:
    """Load an EXR file and return (bgra8_array, width, height).

    Uses OpenEXR directly for reliable EXR support with UE capture data.
    The CameraCapture EXR stores RGBA as FLinearColor (float32 per channel).
    We convert to uint8 BGRA for the streaming sink.
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

    pt = Imath.PixelType(Imath.PixelType.FLOAT)

    if not all(ch in channels for ch in ("R", "G", "B")):
        logger.warning("%s missing RGB channels (has: %s)", exr_path, list(channels))
        return None

    r = np.frombuffer(exr_file.channel("R", pt), dtype=np.float32).reshape((h, w))
    g = np.frombuffer(exr_file.channel("G", pt), dtype=np.float32).reshape((h, w))
    b = np.frombuffer(exr_file.channel("B", pt), dtype=np.float32).reshape((h, w))

    # Linear float → sRGB uint8
    rgb = np.dstack((r, g, b))
    rgb_clipped = np.clip(rgb, 0.0, 1.0)
    rgb_srgb = np.where(
        rgb_clipped <= 0.0031308,
        rgb_clipped * 12.92,
        1.055 * np.power(rgb_clipped, 1.0 / 2.4) - 0.055,
    )
    rgb_u8 = (rgb_srgb * 255.0).astype(np.uint8)

    # Build BGRA8 (swap R↔B, add full alpha)
    bgra = np.zeros((h, w, 4), dtype=np.uint8)
    bgra[:, :, 0] = rgb_u8[:, :, 2]  # B
    bgra[:, :, 1] = rgb_u8[:, :, 1]  # G
    bgra[:, :, 2] = rgb_u8[:, :, 0]  # R
    bgra[:, :, 3] = 255               # A

    return bgra, w, h


def extract_depth_from_exr(exr_path: Path) -> tuple[np.ndarray, int, int] | None:
    """Extract depth channel from EXR (alpha channel or explicit Depth channel)."""
    OpenEXR, Imath = _load_openexr()

    try:
        exr_file = OpenEXR.InputFile(str(exr_path))
    except Exception:
        return None

    header = exr_file.header()
    dw = header["dataWindow"]
    w = dw.max.x - dw.min.x + 1
    h = dw.max.y - dw.min.y + 1
    channels = header["channels"].keys()
    pt = Imath.PixelType(Imath.PixelType.FLOAT)

    # Prefer explicit Depth channel, fall back to Alpha
    depth = None
    if "Depth" in channels:
        depth = np.frombuffer(exr_file.channel("Depth", pt), dtype=np.float32).reshape((h, w))
    elif "A" in channels:
        depth = np.frombuffer(exr_file.channel("A", pt), dtype=np.float32).reshape((h, w))

    if depth is None:
        return None

    # Filter out default alpha (1.0 everywhere = no real depth data)
    if np.allclose(depth, 1.0):
        return None

    return depth, w, h


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


def run_capture_replay(args: argparse.Namespace) -> None:
    """Replay captured EXR+JSON data, decoding EXR to BGRA8 pixels.

    Supports single-camera (flat folder) and multi-camera (actor or full
    capture tree).  Each camera gets its own channel pair:
      camera 0 → RGB on base_channel+0,   depth on depth_base+0
      camera 1 → RGB on base_channel+1,   depth on depth_base+1
      ...
    """
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

    # Build per-camera frame lists and channel assignments
    base_channel = args.channel
    depth_base = args.depth_channel

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
            "rgb_channel": base_channel + i,
            "depth_channel": depth_base + i,
        })

    if not camera_info:
        print(f"ERROR: No frame pairs found for any camera in {capture_dir}")
        sys.exit(1)

    print(f"Found {len(camera_info)} camera(s) in {capture_dir}:")
    for ci in camera_info:
        n = len(ci["frames"])
        print(f"  {ci['label']:40s}  {n:>4d} frames  "
              f"RGB→stream/{ci['rgb_channel']}  "
              f"depth→stream/{ci['depth_channel']}")

    sender = StreamSender(args.host, args.port)
    print(f"\nConnecting to RMSS server at {args.host}:{args.port}...")
    sender.connect()
    print("Connected! Waiting for server to be ready...")
    time.sleep(0.2)

    fps = args.fps
    frame_interval = 1.0 / fps if fps > 0 else 0
    max_frames = args.num_frames if args.num_frames > 0 else 0
    loop = args.loop
    total_sent = 0

    try:
        while True:
            # Determine how many frames to send (use the shortest camera's count
            # if not specified, so all cameras stay in sync)
            num_frames_per_cam = min(len(ci["frames"]) for ci in camera_info)
            if max_frames > 0:
                num_frames_per_cam = min(num_frames_per_cam, max_frames)

            for frame_idx in range(num_frames_per_cam):
                t0 = time.monotonic()

                for ci in camera_info:
                    jf, exr_file = ci["frames"][frame_idx]

                    with open(jf) as f:
                        file_meta = json.load(f)

                    result = load_exr_as_bgra8(exr_file)
                    if result is None:
                        logger.warning("Failed to load %s, skipping", exr_file.name)
                        continue

                    bgra, w, h = result

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

                    xform_key = "relative_transform" if "relative_transform" in file_meta else "world_transform"
                    if xform_key in file_meta:
                        wt = file_meta[xform_key]
                        loc = wt.get("location", [0, 0, 0])
                        rot = wt.get("rotation", [0, 0, 0])
                        meta["transform"] = {
                            "x": loc[0], "y": loc[1], "z": loc[2],
                            "pitch": rot[0], "yaw": rot[1], "roll": rot[2],
                        }
                        meta["transform_space"] = "relative" if xform_key == "relative_transform" else "world"

                    sender.send_image(
                        channel=ci["rgb_channel"],
                        image_bytes=bgra.tobytes(),
                        width=w, height=h,
                        fmt="bgra8",
                        metadata=meta,
                    )

                    if args.send_depth:
                        depth_result = extract_depth_from_exr(exr_file)
                        if depth_result is not None:
                            depth, dw, dh = depth_result
                            depth_meta = dict(meta)
                            depth_meta["fmt"] = "float32"
                            depth_meta["unit"] = "cm"
                            sender.send_depth(
                                channel=ci["depth_channel"],
                                depth_bytes=depth.tobytes(),
                                width=dw, height=dh,
                                metadata=depth_meta,
                            )

                total_sent += len(camera_info)

                if frame_idx % 10 == 0 or frame_idx == 0:
                    print(f"  Frame {frame_idx + 1}/{num_frames_per_cam} "
                          f"({len(camera_info)} cameras, {total_sent} total sends)")

                elapsed = time.monotonic() - t0
                if frame_interval > elapsed:
                    time.sleep(frame_interval - elapsed)

            if not loop:
                break
            print(f"  Looping... ({total_sent} total sends so far)")

        print(f"\nDone! Sent {num_frames_per_cam} frames × {len(camera_info)} cameras "
              f"= {total_sent} total")
    finally:
        sender.disconnect()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
    parser.add_argument("--channel", type=int, default=0,
                        help="RGB channel ID (default: 0 → stream/0 in UE)")
    parser.add_argument("--depth-channel", type=int, default=None,
                        help="Depth channel ID (default: channel+100 → stream/100 in UE)")
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

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--synthetic", action="store_true",
                      help="Send synthetic color-bar test frames")
    mode.add_argument("--capture-dir", metavar="DIR",
                      help="Replay captured EXR+JSON frames from CameraCapture")

    # Capture replay options
    parser.add_argument("--send-depth", action="store_true",
                        help="Also send depth from EXR alpha channel (on channel+100)")
    parser.add_argument("--loop", action="store_true",
                        help="Loop capture replay continuously")
    parser.add_argument("--camera", metavar="FILTER",
                        help="Filter cameras by substring (e.g. 'FL_Capture', 'Gripper')")
    parser.add_argument("--list-cameras", action="store_true",
                        help="List available cameras in capture dir and exit")

    args = parser.parse_args()

    # Resolve depth channel default
    if args.depth_channel is None:
        args.depth_channel = args.channel + 100

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
        run_synthetic(args)
    else:
        run_capture_replay(args)


if __name__ == "__main__":
    main()
