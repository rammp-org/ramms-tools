"""RMSS StreamSender — sends image/binary data to the UE streaming server.

Usage::

    import numpy as np
    from ramms_tools.streaming import StreamSender

    sender = StreamSender("127.0.0.1", 30030)
    sender.connect()

    # Send a numpy BGRA image with stream association
    img = np.zeros((720, 1280, 4), dtype=np.uint8)  # BGRA
    sender.send_numpy_image(channel=0, array=img, group="wrist", role="color")

    # Send float32 depth (cm) linked to the same group
    depth = np.zeros((720, 1280), dtype=np.float32)
    sender.send_numpy_depth(channel=100, array=depth, group="wrist")

    # Send uint16 depth (mm) — native PF_G16 on GPU, no CPU conversion
    depth_mm = np.zeros((720, 1280), dtype=np.uint16)
    sender.send_numpy_depth_uint16(channel=100, array=depth_mm, group="wrist")

    # Send raw bytes with explicit format
    sender.send_image(channel=0, image_bytes=img.tobytes(),
                      width=1280, height=720, fmt="bgra8",
                      group="wrist", role="color")

    # Send with JPEG compression
    from ramms_tools.streaming.protocol import Compression
    from ramms_tools.streaming.compression import compress_jpeg
    raw = img.tobytes()
    compressed = compress_jpeg(raw, width=1280, height=720, quality=85)
    sender.send_image(channel=0, image_bytes=compressed,
                      width=1280, height=720, fmt="bgra8",
                      compression=Compression.JPEG)

    sender.disconnect()
"""

from __future__ import annotations

import json
import logging
import socket
import time
from pathlib import Path
from typing import Optional, Union

from ramms_tools.streaming.protocol import (
    Compression,
    MessageType,
    StreamHeader,
    StreamMessage,
)

logger = logging.getLogger(__name__)


class StreamSender:
    """TCP client that sends data TO the RMSS streaming server (external → UE)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 30030):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._seq: dict[int, int] = {}  # per-channel sequence counter

    # ── Connection ────────────────────────────────────────────────────

    def connect(self, timeout: float = 5.0) -> None:
        if self._sock is not None:
            raise RuntimeError("Already connected")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((self.host, self.port))
        sock.settimeout(None)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2 * 1024 * 1024)
        self._sock = sock
        logger.info("StreamSender connected to %s:%d", self.host, self.port)

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    def __enter__(self) -> "StreamSender":
        self.connect()
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    # ── Send helpers ──────────────────────────────────────────────────

    def _next_seq(self, channel: int) -> int:
        seq = self._seq.get(channel, 0)
        self._seq[channel] = seq + 1
        return seq

    def _send_msg(self, msg: StreamMessage) -> None:
        if self._sock is None:
            raise RuntimeError("Not connected")
        self._sock.sendall(msg.serialize())

    def send_image(self, channel: int, image_bytes: bytes,
                   width: int, height: int,
                   fmt: str = "bgra8",
                   compression: Compression = Compression.NONE,
                   metadata: Optional[dict] = None,
                   group: Optional[str] = None,
                   role: Optional[str] = None,
                   stream_id: Optional[str] = None,
                   name: Optional[str] = None) -> None:
        """Send a raw image to UE as an IMAGE_DATA message.

        If *compression* is not NONE, the caller must have already
        compressed *image_bytes* using the appropriate helper
        (``compress_jpeg`` or ``compress_lz4`` from
        ``ramms_tools.streaming.compression``).
        The compression flag is set in the header so the receiver
        knows how to decompress.

        Args:
            group:     Stream association group (e.g. "wrist"). Streams with the
                       same group are automatically linked in UE (color↔depth↔mask).
            role:      Stream role hint — "color", "depth", "mask", "infrared".
                       Auto-inferred as "color" if not set.
            stream_id: Override the auto-generated stream ID in UE (default:
                       ``{StreamPrefix}/{channel}``).  E.g. "camera/wrist/color".
            name:      Human-readable display name shown in UE UI.
        """
        msg = StreamMessage()
        msg.header.message_type = MessageType.IMAGE_DATA
        msg.header.channel_id = channel
        msg.header.sequence_num = self._next_seq(channel)
        msg.header.timestamp = StreamHeader.now_timestamp()
        if compression != Compression.NONE:
            msg.header.set_compression(compression)

        meta = metadata or {}
        meta.setdefault("w", width)
        meta.setdefault("h", height)
        meta.setdefault("fmt", fmt)
        if group is not None:
            meta["group"] = group
        if role is not None:
            meta["role"] = role
        if stream_id is not None:
            meta["stream_id"] = stream_id
        if name is not None:
            meta["name"] = name
        msg.set_metadata_string(json.dumps(meta))
        msg.payload = image_bytes if isinstance(image_bytes, bytes) else bytes(image_bytes)
        self._send_msg(msg)

    def send_numpy_image(self, channel: int, array, metadata: Optional[dict] = None,
                         group: Optional[str] = None, role: Optional[str] = None,
                         stream_id: Optional[str] = None,
                         name: Optional[str] = None) -> None:
        """Send a numpy array as an image.  Expects shape (H, W, 4) uint8 BGRA."""
        h, w = array.shape[:2]
        channels = array.shape[2] if array.ndim == 3 else 1
        fmt = "rgb8" if channels == 3 else "bgra8"
        self.send_image(channel, array.tobytes(), w, h, fmt=fmt,
                        metadata=metadata, group=group, role=role,
                        stream_id=stream_id, name=name)

    def send_depth(self, channel: int, depth_bytes: bytes,
                   width: int, height: int,
                   metadata: Optional[dict] = None,
                   group: Optional[str] = None,
                   role: Optional[str] = None,
                   stream_id: Optional[str] = None,
                   name: Optional[str] = None) -> None:
        """Send raw depth data (float32 per pixel, values in centimeters).

        Args:
            group:     Stream association group (e.g. "wrist").
            role:      Stream role hint (default: "depth"). Callers may override.
            stream_id: Override auto-generated stream ID. E.g. "camera/wrist/depth".
            name:      Human-readable display name shown in UE UI.
        """
        msg = StreamMessage()
        msg.header.message_type = MessageType.FRAME_DEPTH
        msg.header.channel_id = channel
        msg.header.sequence_num = self._next_seq(channel)
        msg.header.timestamp = StreamHeader.now_timestamp()

        meta = metadata or {}
        meta.setdefault("w", width)
        meta.setdefault("h", height)
        meta.setdefault("fmt", "float32")
        meta.setdefault("unit", "cm")
        meta.setdefault("role", "depth")
        if group is not None:
            meta["group"] = group
        if role is not None:
            meta["role"] = role
        if stream_id is not None:
            meta["stream_id"] = stream_id
        if name is not None:
            meta["name"] = name
        msg.set_metadata_string(json.dumps(meta))
        msg.payload = depth_bytes if isinstance(depth_bytes, bytes) else bytes(depth_bytes)
        self._send_msg(msg)

    def send_numpy_depth(self, channel: int, array, metadata: Optional[dict] = None,
                         group: Optional[str] = None, role: Optional[str] = None,
                         stream_id: Optional[str] = None,
                         name: Optional[str] = None) -> None:
        """Send a numpy float32 depth array. Expects shape (H, W), values in cm."""
        h, w = array.shape[:2]
        self.send_depth(channel, array.tobytes(), w, h,
                        metadata=metadata, group=group, role=role,
                        stream_id=stream_id, name=name)

    def send_depth_uint16(self, channel: int, depth_bytes: bytes,
                          width: int, height: int,
                          metadata: Optional[dict] = None,
                          group: Optional[str] = None,
                          role: Optional[str] = None,
                          stream_id: Optional[str] = None,
                          name: Optional[str] = None) -> None:
        """Send raw uint16 depth data (values in millimeters).

        The UE sink creates a native PF_G16 texture — no CPU conversion.
        The PGM shader and camera widget materials auto-detect the format
        and apply the correct unnormalization (×65535) and scale (×0.1 → cm).

        Args:
            group:     Stream association group (e.g. "wrist").
            role:      Stream role hint (default: "depth"). Callers may override.
            stream_id: Override auto-generated stream ID. E.g. "camera/wrist/depth".
            name:      Human-readable display name shown in UE UI.
        """
        msg = StreamMessage()
        msg.header.message_type = MessageType.FRAME_DEPTH
        msg.header.channel_id = channel
        msg.header.sequence_num = self._next_seq(channel)
        msg.header.timestamp = StreamHeader.now_timestamp()

        meta = metadata or {}
        meta.setdefault("w", width)
        meta.setdefault("h", height)
        meta.setdefault("fmt", "16uc1")
        meta.setdefault("unit", "mm")
        meta.setdefault("role", "depth")
        if group is not None:
            meta["group"] = group
        if role is not None:
            meta["role"] = role
        if stream_id is not None:
            meta["stream_id"] = stream_id
        if name is not None:
            meta["name"] = name
        msg.set_metadata_string(json.dumps(meta))
        msg.payload = depth_bytes if isinstance(depth_bytes, bytes) else bytes(depth_bytes)
        self._send_msg(msg)

    def send_numpy_depth_uint16(self, channel: int, array,
                                metadata: Optional[dict] = None,
                                group: Optional[str] = None,
                                role: Optional[str] = None,
                                stream_id: Optional[str] = None,
                                name: Optional[str] = None) -> None:
        """Send a numpy uint16 depth array. Expects shape (H, W), values in mm.

        If the array is already ``np.uint16`` and C-contiguous, no copy is made.
        Otherwise the array is cast to uint16 (which may involve a copy).
        """
        import numpy as np

        if array.dtype != np.uint16:
            array = array.astype(np.uint16)
        elif not array.flags["C_CONTIGUOUS"]:
            array = np.ascontiguousarray(array)
        h, w = array.shape[:2]
        self.send_depth_uint16(channel, array.tobytes(), w, h,
                               metadata=metadata, group=group, role=role,
                               stream_id=stream_id, name=name)

    def send_motion(self, channel: int, motion_bytes: bytes,
                    width: int, height: int,
                    metadata: Optional[dict] = None,
                    group: Optional[str] = None,
                    role: Optional[str] = None,
                    stream_id: Optional[str] = None,
                    name: Optional[str] = None) -> None:
        """Send raw motion-vector data (float32 XY per pixel).

        Args:
            group:     Stream association group (e.g. "wrist").
            role:      Stream role hint (default: "motion"). Callers may override.
            stream_id: Override auto-generated stream ID.
            name:      Human-readable display name shown in UE UI.
        """
        msg = StreamMessage()
        msg.header.message_type = MessageType.FRAME_MOTION
        msg.header.channel_id = channel
        msg.header.sequence_num = self._next_seq(channel)
        msg.header.timestamp = StreamHeader.now_timestamp()

        meta = metadata or {}
        meta.setdefault("w", width)
        meta.setdefault("h", height)
        meta.setdefault("fmt", "rg32f")
        meta.setdefault("role", "motion")
        if group is not None:
            meta["group"] = group
        if role is not None:
            meta["role"] = role
        if stream_id is not None:
            meta["stream_id"] = stream_id
        if name is not None:
            meta["name"] = name
        msg.set_metadata_string(json.dumps(meta))
        msg.payload = motion_bytes if isinstance(motion_bytes, bytes) else bytes(motion_bytes)
        self._send_msg(msg)

    def send_numpy_motion(self, channel: int, array, metadata: Optional[dict] = None,
                          group: Optional[str] = None, role: Optional[str] = None,
                          stream_id: Optional[str] = None,
                          name: Optional[str] = None) -> None:
        """Send a numpy float32 motion-vector array. Expects shape (H, W, 2)."""
        h, w = array.shape[:2]
        self.send_motion(channel, array.tobytes(), w, h,
                         metadata=metadata, group=group, role=role,
                         stream_id=stream_id, name=name)

    # ── Capture directory replay ──────────────────────────────────────

    def send_capture_dir(self, capture_dir: Union[str, Path],
                         channel: int = 0,
                         fps: float = 0,
                         max_frames: int = 0) -> int:
        """
        Send frames from a CameraCapture serialized directory to UE.

        The directory should contain frame_XXXXXXX.exr + frame_XXXXXXX.json pairs.
        If fps > 0, paces sending at that rate.  Returns number of frames sent.

        NOTE: This reads the JSON metadata and sends it along. The EXR pixel data
        is read as raw bytes — the UE sink component will need to interpret the format.
        For simplicity, this sends the raw EXR file bytes with message type IMAGE_DATA.
        """
        capture_path = Path(capture_dir)
        if not capture_path.is_dir():
            raise FileNotFoundError(f"Capture directory not found: {capture_dir}")

        # Find frame pairs
        json_files = sorted(capture_path.glob("frame_*.json"))
        if not json_files:
            logger.warning("No frame_*.json files found in %s", capture_dir)
            return 0

        sent = 0
        frame_interval = 1.0 / fps if fps > 0 else 0

        for jf in json_files:
            if 0 < max_frames <= sent:
                break

            # Find matching EXR
            exr_file = jf.with_suffix(".exr")
            if not exr_file.exists():
                logger.warning("Missing EXR for %s, skipping", jf.name)
                continue

            # Read metadata
            with open(jf, "r") as f:
                meta = json.load(f)

            # Read EXR as raw bytes
            with open(exr_file, "rb") as f:
                exr_data = f.read()

            meta["source"] = "capture_replay"
            meta["original_file"] = exr_file.name

            self.send_image(
                channel=channel,
                image_bytes=exr_data,
                width=meta.get("Width", meta.get("w", 0)),
                height=meta.get("Height", meta.get("h", 0)),
                fmt="exr",
                metadata=meta,
            )
            sent += 1
            logger.debug("Sent frame %s (%d bytes)", exr_file.name, len(exr_data))

            if frame_interval > 0:
                time.sleep(frame_interval)

        logger.info("Sent %d frames from %s", sent, capture_dir)
        return sent
