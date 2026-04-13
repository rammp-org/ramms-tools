#!/usr/bin/env python3
"""ramms-stream — CLI tool for RMSS binary streaming.

Connects to the UE RMSS server, subscribes to camera channels, and either:
  - Prints stats (default)
  - Saves frames to disk in CameraCapture-compatible format (--save)
  - Replays captured frames back to UE (--replay)
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import struct
import sys
import time
from pathlib import Path

logger = logging.getLogger(__name__)


def _setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


# ---------------------------------------------------------------------------
# Receive / record mode
# ---------------------------------------------------------------------------

def _run_receive(args: argparse.Namespace) -> None:
    from ramms_tools.streaming.client import StreamClient
    from ramms_tools.streaming.protocol import MessageType

    save_dir: Path | None = None
    if args.save:
        save_dir = Path(args.save)
        save_dir.mkdir(parents=True, exist_ok=True)
        logger.info("Saving frames to %s", save_dir)

    client = StreamClient(args.host, args.port)
    client.connect()

    channels = [int(c) for c in args.channels.split(",")] if args.channels else None
    client.subscribe(channels=channels, compression=args.compression)
    client.start()

    frame_counts: dict[int, int] = {}
    total_bytes = 0
    t_start = time.monotonic()

    stop_event = False

    def _sigint(sig, frame):
        nonlocal stop_event
        stop_event = True

    signal.signal(signal.SIGINT, _sigint)

    print(f"Connected to {args.host}:{args.port}")
    print(f"Subscribed channels: {channels or 'all'}")
    print(f"Compression: {args.compression}")
    if save_dir:
        print(f"Saving to: {save_dir}")
    print("Press Ctrl+C to stop\n")

    try:
        while not stop_event:
            msg = client.poll(timeout=0.5)
            if msg is None:
                continue

            ch = msg.header.channel_id
            mt = msg.header.message_type
            frame_counts[ch] = frame_counts.get(ch, 0) + 1
            total_bytes += len(msg.payload) + len(msg.metadata)

            # Save to disk
            if save_dir and mt in (
                MessageType.FRAME_RGB, MessageType.FRAME_DEPTH,
                MessageType.FRAME_RGBD, MessageType.FRAME_DATA,
                MessageType.IMAGE_DATA
            ):
                _save_frame(save_dir, ch, msg, frame_counts[ch])

            # Stats output
            elapsed = time.monotonic() - t_start
            if frame_counts[ch] % max(1, args.stats_interval) == 0:
                fps = frame_counts[ch] / elapsed if elapsed > 0 else 0
                bw = total_bytes / elapsed / 1024 / 1024 if elapsed > 0 else 0
                meta = msg.get_metadata_json() if msg.metadata else {}
                w = meta.get("w", "?")
                h = meta.get("h", "?")
                print(
                    f"  Ch{ch:02d} | {MessageType(mt).name:14s} | "
                    f"#{frame_counts[ch]:6d} | {w}x{h} | "
                    f"{fps:5.1f} fps | {bw:6.1f} MB/s | "
                    f"payload={len(msg.payload)} bytes"
                )

            if 0 < args.max_frames <= sum(frame_counts.values()):
                break
    finally:
        client.disconnect()
        elapsed = time.monotonic() - t_start
        print(f"\n--- Summary ---")
        print(f"Duration: {elapsed:.1f}s")
        for ch, count in sorted(frame_counts.items()):
            print(f"  Channel {ch}: {count} frames ({count/elapsed:.1f} fps)")
        print(f"Total data: {total_bytes / 1024 / 1024:.1f} MB")


def _save_frame(save_dir: Path, channel: int, msg, frame_num: int) -> None:
    """Save a received frame in CameraCapture-compatible format."""
    ch_dir = save_dir / f"channel_{channel:02d}"
    ch_dir.mkdir(exist_ok=True)

    meta = msg.get_metadata_json() if msg.metadata else {}
    meta["frame_number"] = frame_num
    meta["receive_timestamp"] = time.time()
    meta["message_type"] = msg.header.message_type
    meta["channel_id"] = channel
    meta["sequence_num"] = msg.header.sequence_num

    # Save metadata JSON
    json_path = ch_dir / f"frame_{frame_num:07d}.json"
    with open(json_path, "w") as f:
        json.dump(meta, f, indent=2)

    # Save binary payload
    fmt = meta.get("fmt", "raw")
    if fmt == "exr":
        ext = ".exr"
    elif fmt in ("bgra8", "rgba8"):
        ext = ".raw"
    elif fmt == "float32":
        ext = ".depth"
    else:
        ext = ".bin"

    bin_path = ch_dir / f"frame_{frame_num:07d}{ext}"
    with open(bin_path, "wb") as f:
        f.write(msg.payload)


# ---------------------------------------------------------------------------
# Replay mode
# ---------------------------------------------------------------------------

def _run_replay(args: argparse.Namespace) -> None:
    from ramms_tools.streaming.sender import StreamSender

    sender = StreamSender(args.host, args.port)
    sender.connect()

    try:
        count = sender.send_capture_dir(
            args.replay,
            channel=args.replay_channel,
            fps=args.replay_fps,
            max_frames=args.max_frames,
        )
        print(f"Replayed {count} frames from {args.replay}")
    finally:
        sender.disconnect()


# ---------------------------------------------------------------------------
# Ping mode
# ---------------------------------------------------------------------------

def _run_ping(args: argparse.Namespace) -> None:
    from ramms_tools.streaming.client import StreamClient

    client = StreamClient(args.host, args.port)
    client.connect()
    client.start()

    try:
        for i in range(args.ping_count):
            try:
                rtt = client.ping()
                print(f"PING {i+1}: {rtt*1000:.1f} ms")
            except TimeoutError:
                print(f"PING {i+1}: timeout")
            time.sleep(1.0)
    finally:
        client.disconnect()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        prog="ramms-stream",
        description="RMSS binary streaming client — receive, record, or replay camera data",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=30030, help="Server port (default: 30030)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable debug logging")

    # Mode selection
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--replay", metavar="DIR",
                       help="Replay captured frames from DIR to UE")
    group.add_argument("--ping", action="store_true",
                       help="Ping the server to test connectivity")

    # Receive options
    parser.add_argument("-c", "--channels", default=None,
                        help="Comma-separated channel IDs to subscribe to (default: all)")
    parser.add_argument("--compression", default="none",
                        choices=["none", "lz4", "jpeg", "png"],
                        help="Requested compression (default: none)")
    parser.add_argument("--save", metavar="DIR",
                        help="Save received frames to DIR")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Stop after N frames (0 = unlimited)")
    parser.add_argument("--stats-interval", type=int, default=30,
                        help="Print stats every N frames (default: 30)")

    # Replay options
    parser.add_argument("--replay-channel", type=int, default=0,
                        help="Channel ID for replay (default: 0)")
    parser.add_argument("--replay-fps", type=float, default=30.0,
                        help="Replay frame rate (default: 30)")

    # Ping options
    parser.add_argument("--ping-count", type=int, default=5,
                        help="Number of pings (default: 5)")

    args = parser.parse_args()
    _setup_logging(args.verbose)

    if args.replay:
        _run_replay(args)
    elif args.ping:
        _run_ping(args)
    else:
        _run_receive(args)


if __name__ == "__main__":
    main()
