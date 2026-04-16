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

    # Send a mono8 mask (uint8, 1 byte/pixel)
    mask = np.zeros((720, 1280), dtype=np.uint8)
    sender.send_mask(channel=200, mask_bytes=mask.tobytes(),
                     width=1280, height=720, group="wrist")
    # or from numpy:
    sender.send_numpy_mask(channel=200, array=mask, group="wrist")

    # Send generic frame data with any format (uses FRAME_DATA message type)
    sender.send_data(channel=0, data=raw_bytes, width=1280, height=720,
                     fmt="mono8", role="mask")

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

Non-blocking mode (default)::

    # Queue size is configurable
    sender = StreamSender("127.0.0.1", 30030, queue_size=4)
    sender.connect()

    # send_image / send_depth etc. are non-blocking.
    # If the queue is full the oldest queued message is dropped (FIFO eviction),
    # keeping the most recently enqueued frame.
    sender.send_numpy_image(channel=0, array=img, group="wrist", role="color")

    # Synchronous mode (legacy behaviour) — blocks on each send:
    sender = StreamSender("127.0.0.1", 30030, queue_size=0)
"""

from __future__ import annotations

import json
import logging
import queue
import socket
import threading
import time
from pathlib import Path
from typing import Optional, Union

from .protocol import (
    Compression,
    MessageType,
    StreamHeader,
    StreamMessage,
)

logger = logging.getLogger(__name__)


class StreamSender:
    """TCP client that sends data TO the RMSS streaming server (external → UE).

    Args:
        host:        Server IP address.
        port:        Server TCP port.
        queue_size:  Max queued messages. 0 = synchronous (blocking) mode.
                     When the queue is full the oldest queued message is evicted
                     (regardless of channel) so the newest frame always wins.

    Note:
        A single background worker thread is used for non-blocking mode.
        Multiple workers are not supported (would interleave ``sendall()``
        on the same TCP socket, corrupting message framing).
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 30030,
        queue_size: int = 4,
    ):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._seq: dict[int, int] = {}  # per-channel sequence counter

        # Queue / worker config — single worker to avoid interleaved writes.
        self._queue_size = max(queue_size, 0)
        self._num_workers = 1 if self._queue_size > 0 else 0
        self._send_queue: Optional[queue.Queue] = None
        self._workers: list[threading.Thread] = []
        self._shutdown_event = threading.Event()
        self._send_lock = threading.Lock()  # guards _sock.sendall in sync mode

        # Stats — guarded by _drop_lock for thread safety
        self._drop_lock = threading.Lock()
        self.dropped_frames: int = 0

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

        # Start worker threads
        if self._queue_size > 0:
            self._shutdown_event.clear()
            self._send_queue = queue.Queue(maxsize=self._queue_size)
            self._workers = []
            for i in range(self._num_workers):
                t = threading.Thread(
                    target=self._send_worker,
                    name=f"StreamSender-worker-{i}",
                    daemon=True,
                )
                t.start()
                self._workers.append(t)
            logger.info(
                "StreamSender: %d worker(s), queue_size=%d",
                self._num_workers,
                self._queue_size,
            )

    def is_connected(self) -> bool:
        return self._sock is not None

    def disconnect(self, flush: bool = False, flush_timeout: float = 10.0) -> None:
        """Disconnect and stop the worker thread.

        Args:
            flush:         If True, wait for the worker to send all queued
                           messages before shutting down. If False (default),
                           queued messages are discarded.
            flush_timeout: Max seconds to wait for the flush to complete.
                           Ignored when *flush* is False.

        Note:
            When *flush* is False, any messages still in the queue are dropped.
            Use ``flush=True`` if you need to guarantee delivery of enqueued
            frames before disconnecting.
        """
        q = self._send_queue

        if flush and q is not None:
            # Block until every enqueued item has been dequeued AND task_done()
            # has been called — this means the worker has finished sendall()
            # for each item, not just dequeued it.
            done = threading.Event()

            def _join_waiter():
                q.join()
                done.set()

            t = threading.Thread(target=_join_waiter, daemon=True)
            t.start()
            if not done.wait(timeout=flush_timeout):
                logger.warning(
                    "StreamSender flush timed out — %d task(s) remaining",
                    q.unfinished_tasks,
                )

        # Signal workers to stop
        self._shutdown_event.set()

        # Close/shutdown the socket so a worker blocked in sendall()
        # gets an OSError and unblocks, rather than hanging until join timeout.
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

        # Wait for workers to exit (they should be unblocked now)
        for t in self._workers:
            t.join(timeout=5.0)
            if t.is_alive():
                logger.warning("StreamSender worker %s did not stop", t.name)
        self._workers.clear()

        # Safe to clear the queue now — no workers are running
        self._send_queue = None

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

    def _send_worker(self) -> None:
        """Background thread: drains the queue and does blocking socket sends."""
        while not self._shutdown_event.is_set():
            q = self._send_queue
            if q is None:
                break
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                sock = self._sock
                if sock is not None:
                    sock.sendall(msg.serialize())
            except OSError as exc:
                logger.warning("StreamSender send error: %s", exc)
                self._shutdown_event.set()
                # Transition to disconnected so is_connected() returns False.
                sock = self._sock
                self._sock = None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                q.task_done()
                break
            else:
                q.task_done()

    def _send_msg(self, msg: StreamMessage) -> None:
        """Enqueue a message (non-blocking) or send synchronously if queue_size=0."""
        if self._sock is None:
            raise RuntimeError("Not connected")

        # Synchronous mode — direct blocking send (legacy behaviour)
        if self._send_queue is None:
            with self._send_lock:
                sock = self._sock
                if sock is None:
                    raise RuntimeError("Not connected")
                sock.sendall(msg.serialize())
            return

        # Non-blocking mode — try to enqueue, evict oldest if full
        try:
            self._send_queue.put_nowait(msg)
        except queue.Full:
            # Evict the oldest item and retry.  Call task_done() for the
            # evicted item so the unfinished_tasks counter stays consistent
            # (required for Queue.join()-based flushing).
            try:
                self._send_queue.get_nowait()
                self._send_queue.task_done()
            except queue.Empty:
                pass
            with self._drop_lock:
                self.dropped_frames += 1
            try:
                self._send_queue.put_nowait(msg)
            except queue.Full:
                # Queue still full (shouldn't happen after eviction), drop silently
                with self._drop_lock:
                    self.dropped_frames += 1

    def send_image(
        self,
        channel: int,
        image_bytes: bytes,
        width: int,
        height: int,
        fmt: str = "bgra8",
        compression: Compression = Compression.NONE,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
        material_params: Optional[dict[str, float]] = None,
    ) -> None:
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
            material_params: Optional dict of scalar material parameters to forward
                       to UE MIDs (e.g. ``{"NumSegmentIDs": 5.0}``).
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
        if material_params is not None:
            meta["MaterialScalarParameters"] = material_params
        msg.set_metadata_string(json.dumps(meta))
        msg.payload = (
            image_bytes if isinstance(image_bytes, bytes) else bytes(image_bytes)
        )
        self._send_msg(msg)

    def send_numpy_image(
        self,
        channel: int,
        array,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
        material_params: Optional[dict[str, float]] = None,
    ) -> None:
        """Send a numpy array as an image.

        Supported inputs:
          - ``(H, W, 4)`` uint8 BGRA → fmt ``"bgra8"``
          - ``(H, W, 3)`` uint8 RGB  → fmt ``"rgb8"``

        Raises:
            ValueError: If *array* is not 3-dimensional with 3 or 4 channels,
                        or if the dtype is not ``uint8``.
        """
        import numpy as np

        if array.ndim != 3 or array.shape[2] not in (3, 4):
            raise ValueError(
                f"send_numpy_image expects shape (H, W, 3) or (H, W, 4), "
                f"got {array.shape!r}. For grayscale/mono data use "
                f"send_numpy_mask() or send_data()."
            )
        if array.dtype != np.uint8:
            raise ValueError(
                f"send_numpy_image expects uint8 dtype, got {array.dtype}. "
                f"Cast the array first or use send_data() with an explicit fmt."
            )
        h, w = array.shape[:2]
        fmt = "rgb8" if array.shape[2] == 3 else "bgra8"
        self.send_image(
            channel,
            array.tobytes(),
            w,
            h,
            fmt=fmt,
            metadata=metadata,
            group=group,
            role=role,
            stream_id=stream_id,
            name=name,
            material_params=material_params,
        )

    def send_depth(
        self,
        channel: int,
        depth_bytes: bytes,
        width: int,
        height: int,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
    ) -> None:
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
        msg.payload = (
            depth_bytes if isinstance(depth_bytes, bytes) else bytes(depth_bytes)
        )
        self._send_msg(msg)

    def send_numpy_depth(
        self,
        channel: int,
        array,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
    ) -> None:
        """Send a numpy float32 depth array. Expects shape (H, W), values in cm."""
        h, w = array.shape[:2]
        self.send_depth(
            channel,
            array.tobytes(),
            w,
            h,
            metadata=metadata,
            group=group,
            role=role,
            stream_id=stream_id,
            name=name,
        )

    def send_depth_uint16(
        self,
        channel: int,
        depth_bytes: bytes,
        width: int,
        height: int,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
    ) -> None:
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
        msg.payload = (
            depth_bytes if isinstance(depth_bytes, bytes) else bytes(depth_bytes)
        )
        self._send_msg(msg)

    def send_numpy_depth_uint16(
        self,
        channel: int,
        array,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
    ) -> None:
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
        self.send_depth_uint16(
            channel,
            array.tobytes(),
            w,
            h,
            metadata=metadata,
            group=group,
            role=role,
            stream_id=stream_id,
            name=name,
        )

    def send_data(
        self,
        channel: int,
        data: bytes,
        width: int,
        height: int,
        fmt: str,
        compression: Compression = Compression.NONE,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
        material_params: Optional[dict[str, float]] = None,
    ) -> None:
        """Send arbitrary frame data using the generic FRAME_DATA message type.

        Unlike ``send_image`` (which uses IMAGE_DATA), this uses FRAME_DATA
        (0x11) and works with any pixel format supported by UE's
        ``ResolvePixelFormat`` — e.g. ``mono8``, ``r8``, ``gray8``,
        ``bgra8``, ``16uc1``, ``float32``, ``rg32f``.

        Args:
            fmt:       Pixel format string (e.g. "mono8", "float32", "bgra8").
            group:     Stream association group (e.g. "wrist").
            role:      Stream role hint — "color", "depth", "mask", "motion".
            stream_id: Override auto-generated stream ID.
            name:      Human-readable display name shown in UE UI.
            material_params: Optional dict of scalar material parameters to forward
                       to UE MIDs (e.g. ``{"NumSegmentIDs": 5.0}``).
        """
        msg = StreamMessage()
        msg.header.message_type = MessageType.FRAME_DATA
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
        if material_params is not None:
            meta["MaterialScalarParameters"] = material_params
        msg.set_metadata_string(json.dumps(meta))
        msg.payload = data if isinstance(data, bytes) else bytes(data)
        self._send_msg(msg)

    def send_mask(
        self,
        channel: int,
        mask_bytes: bytes,
        width: int,
        height: int,
        fmt: str = "mono8",
        compression: Compression = Compression.NONE,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
        material_params: Optional[dict[str, float]] = None,
    ) -> None:
        """Send single-channel mask data (uint8 per pixel by default).

        Convenience wrapper around ``send_data`` that defaults to
        ``fmt="mono8"`` and ``role="mask"``.

        Args:
            fmt:       Pixel format (default "mono8").  Also accepts "float32"
                       for float mask IDs.
            group:     Stream association group (e.g. "wrist").
            stream_id: Override auto-generated stream ID.
            name:      Human-readable display name shown in UE UI.
            material_params: Optional dict of scalar material parameters to forward
                       to UE MIDs (e.g. ``{"NumSegmentIDs": 5.0}``).
        """
        self.send_data(
            channel=channel,
            data=mask_bytes,
            width=width,
            height=height,
            fmt=fmt,
            compression=compression,
            metadata=metadata,
            group=group,
            role="mask",
            stream_id=stream_id,
            name=name,
            material_params=material_params,
        )

    def send_numpy_mask(
        self,
        channel: int,
        array,
        fmt: Optional[str] = None,
        compression: Compression = Compression.NONE,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
        material_params: Optional[dict[str, float]] = None,
    ) -> None:
        """Send a numpy mask array.

        If *fmt* is not provided, it is inferred from the array dtype:
          - ``uint8``   → ``"mono8"`` (1 byte/pixel)
          - ``float32`` → ``"float32"`` (4 bytes/pixel)
          - Other       → cast to ``uint8``, sent as ``"mono8"``

        All other parameters (compression, material_params, etc.) are
        forwarded to ``send_mask()`` for full parity with the raw API.
        """
        import numpy as np

        h, w = array.shape[:2]
        if fmt is None:
            if array.dtype == np.float32:
                fmt = "float32"
            elif array.dtype == np.uint8:
                fmt = "mono8"
            else:
                array = array.astype(np.uint8)
                fmt = "mono8"
        if not array.flags["C_CONTIGUOUS"]:
            array = np.ascontiguousarray(array)
        self.send_mask(
            channel,
            array.tobytes(),
            w,
            h,
            fmt=fmt,
            compression=compression,
            metadata=metadata,
            group=group,
            stream_id=stream_id,
            name=name,
            material_params=material_params,
        )

    def send_motion(
        self,
        channel: int,
        motion_bytes: bytes,
        width: int,
        height: int,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
    ) -> None:
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
        msg.payload = (
            motion_bytes if isinstance(motion_bytes, bytes) else bytes(motion_bytes)
        )
        self._send_msg(msg)

    def send_numpy_motion(
        self,
        channel: int,
        array,
        metadata: Optional[dict] = None,
        group: Optional[str] = None,
        role: Optional[str] = None,
        stream_id: Optional[str] = None,
        name: Optional[str] = None,
    ) -> None:
        """Send a numpy float32 motion-vector array. Expects shape (H, W, 2)."""
        h, w = array.shape[:2]
        self.send_motion(
            channel,
            array.tobytes(),
            w,
            h,
            metadata=metadata,
            group=group,
            role=role,
            stream_id=stream_id,
            name=name,
        )

    # ── Capture directory replay ──────────────────────────────────────

    def send_capture_dir(
        self,
        capture_dir: Union[str, Path],
        channel: int = 0,
        fps: float = 0,
        max_frames: int = 0,
    ) -> int:
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
