"""RMSS StreamClient — connects to the UE streaming server and receives frames.

Usage::

    from ramms_tools.streaming import StreamClient

    def on_frame(msg):
        meta = msg.get_metadata_json()
        print(f"Frame {msg.header.sequence_num}: {meta['w']}x{meta['h']}")

    client = StreamClient("127.0.0.1", 30030)
    client.connect()
    client.subscribe(channels=[0, 1], compression="none")
    client.on_message = on_frame
    client.start()          # background receive thread
    # ... do work ...
    client.stop()
    client.disconnect()

Or use the context manager::

    with StreamClient("127.0.0.1", 30030) as c:
        c.subscribe([0])
        for msg in c.iter_messages(timeout=1.0):
            process(msg)
"""

from __future__ import annotations

import json
import logging
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

from ramms_tools.streaming.protocol import (
    HEADER_SIZE,
    MAGIC,
    VERSION,
    Compression,
    MessageType,
    StreamHeader,
    StreamMessage,
)
from ramms_tools.streaming.compression import can_decompress, decompress_payload

logger = logging.getLogger(__name__)

# Message types that carry frame/sensor data (used for stats tracking)
_FRAME_TYPES = frozenset({
    MessageType.FRAME_RGB,
    MessageType.FRAME_DEPTH,
    MessageType.FRAME_RGBD,
    MessageType.FRAME_MOTION,
    MessageType.FRAME_DATA,
    MessageType.POINT_CLOUD,
    MessageType.OCTO_MAP,
    MessageType.IMAGE_DATA,
})


class _MissingCodec(Exception):
    """A frame's codec is not installed here: it is delivered as it was sent."""


def _require_codecs(*codecs: Compression) -> None:
    missing = [c.name for c in codecs if not can_decompress(c)]
    if missing:
        raise _MissingCodec("/".join(missing))


def _decompress_rgbd(msg: StreamMessage) -> tuple[bytes, bytes]:
    """A compressed FRAME_RGBD is a JPEG colour block then an LZ4 depth block
    (header flagged JPEG); the metadata's rgb_size gives the split.  Returns
    the raw payload (BGRA8 then float32 depth) and metadata restating the
    raw sizes.  Raises _MissingCodec, before touching anything, if either
    codec is not installed."""
    meta = msg.get_metadata_json()
    depth_lz4 = meta.get("depth_comp") == "lz4"
    _require_codecs(Compression.JPEG, *([Compression.LZ4] if depth_lz4 else []))
    rgb_size = int(meta.get("rgb_size", len(msg.payload)))
    rgb = decompress_payload(msg.payload[:rgb_size], Compression.JPEG)
    depth = msg.payload[rgb_size:]
    if depth_lz4:
        depth = decompress_payload(depth, Compression.LZ4)
    for key in ("rgb_comp", "depth_comp", "rgb_raw_size", "depth_raw_size"):
        meta.pop(key, None)
    meta["rgb_size"], meta["depth_size"] = len(rgb), len(depth)
    return rgb + depth, json.dumps(meta).encode("utf-8")


@dataclass
class ChannelStats:
    """Per-channel streaming statistics."""
    channel_id: int = 0
    frames: int = 0
    bytes_total: int = 0
    bytes_compressed: int = 0
    last_seq: int = -1
    dropped: int = 0
    _timestamps: deque = field(default_factory=lambda: deque(maxlen=200), repr=False)
    _lock: threading.Lock | threading.RLock | None = field(default=None, repr=False)

    def fps(self) -> float:
        """Approximate FPS over the last 2 seconds of frames.

        Snapshots timestamps under lock (if set) to avoid racing
        with the recv thread that appends to the deque.
        """
        if self._lock is not None:
            with self._lock:
                ts = list(self._timestamps)
        else:
            ts = list(self._timestamps)
        now = time.monotonic()
        cutoff = now - 2.0
        count = sum(1 for t in ts if t > cutoff)
        return count / 2.0

    @property
    def bandwidth_mib_s(self) -> float:
        """Approximate bandwidth in MiB/s over last 2s window."""
        return self.fps() * (self.bytes_total / max(self.frames, 1)) / (1024 * 1024)


class StreamClient:
    """TCP client for the RMSS binary streaming protocol."""

    def __init__(self, host: str = "127.0.0.1", port: int = 30030,
                 recv_buffer_size: int = 256 * 1024,
                 auto_decompress: bool = True):
        self.host = host
        self.port = port
        self._recv_buffer_size = recv_buffer_size
        self._auto_decompress = auto_decompress
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stopping = threading.Event()
        self._connected = threading.Event()

        # Inbound message queue (bounded to avoid memory blowup)
        self._queue: deque[StreamMessage] = deque(maxlen=120)
        self._queue_lock = threading.Lock()
        self._queue_event = threading.Event()

        # User callback (called from recv thread)
        self.on_message: Optional[Callable[[StreamMessage], None]] = None

        # Stats
        self.bytes_received = 0
        self.messages_received = 0
        self.errors = 0
        self._stats_lock = threading.RLock()
        self._channel_stats: dict[int, ChannelStats] = {}
        self._connect_time: float = 0.0

        # Out-of-band PING reply handling
        self._ping_event = threading.Event()
        self._ping_rtt: float = 0.0

        # Codecs already warned about as missing (once each, not per frame)
        self._missing_codecs_warned: set[str] = set()

    # ── Connection ────────────────────────────────────────────────────

    def connect(self, timeout: float = 5.0) -> None:
        """Connect to the RMSS server."""
        if self._sock is not None:
            raise RuntimeError("Already connected")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((self.host, self.port))
        sock.settimeout(None)
        # Set large receive buffer
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self._recv_buffer_size)
        self._sock = sock
        self._connected.set()
        self._connect_time = time.monotonic()
        logger.info("Connected to RMSS server at %s:%d", self.host, self.port)

    def disconnect(self) -> None:
        """Disconnect from the server."""
        self.stop()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
        self._connected.clear()
        logger.info("Disconnected from RMSS server")

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    # ── Context manager ───────────────────────────────────────────────

    def __enter__(self) -> "StreamClient":
        self.connect()
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    # ── Subscribe / Unsubscribe ───────────────────────────────────────

    def subscribe(self, channels: list[int] | None = None,
                  compression: str = "none") -> None:
        """Send a SUBSCRIBE message to the server."""
        meta = {}
        if channels is not None:
            meta["channels"] = channels
        if compression != "none":
            meta["compression"] = compression

        msg = StreamMessage()
        msg.header.message_type = MessageType.SUBSCRIBE
        msg.header.timestamp = StreamHeader.now_timestamp()
        msg.set_metadata_string(json.dumps(meta))
        self._send(msg)
        logger.info("Subscribed to channels=%s compression=%s", channels, compression)

    def unsubscribe(self, channels: list[int] | None = None) -> None:
        """Send an UNSUBSCRIBE message."""
        meta = {}
        if channels is not None:
            meta["channels"] = channels
        msg = StreamMessage()
        msg.header.message_type = MessageType.UNSUBSCRIBE
        msg.header.timestamp = StreamHeader.now_timestamp()
        msg.set_metadata_string(json.dumps(meta))
        self._send(msg)

    def ping(self) -> float:
        """Send a PING and wait for the response.  Returns round-trip time in seconds.

        Uses an out-of-band Event so that arriving frames are not
        dequeued/dropped while waiting for the PING reply.
        """
        self._ping_event.clear()
        msg = StreamMessage()
        msg.header.message_type = MessageType.PING
        msg.header.timestamp = StreamHeader.now_timestamp()
        self._send(msg)
        if not self._ping_event.wait(timeout=5.0):
            raise TimeoutError("PING timeout")
        return self._ping_rtt

    # ── Receive thread ────────────────────────────────────────────────

    def start(self) -> None:
        """Start the background receive thread.

        Raises RuntimeError if the client is not connected.
        """
        if self._thread is not None:
            return
        if self._sock is None:
            raise RuntimeError("Cannot start receive thread: not connected. Call connect() first.")
        self._stopping.clear()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True, name="rmss-recv")
        self._thread.start()
        logger.debug("Receive thread started")

    def stop(self) -> None:
        """Stop the background receive thread."""
        self._stopping.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def _recv_loop(self) -> None:
        buf = bytearray()
        # Bytes the next message needs: a large frame arrives over many
        # recv calls, and copying the buffer to parse it after each one
        # would be quadratic in the frame size.
        need = HEADER_SIZE
        while not self._stopping.is_set():
            try:
                self._sock.settimeout(0.2)
                chunk = self._sock.recv(65536)
                if not chunk:
                    logger.info("Server closed connection")
                    self._connected.clear()
                    break
                buf.extend(chunk)
                self.bytes_received += len(chunk)
            except socket.timeout:
                continue
            except OSError:
                if not self._stopping.is_set():
                    logger.warning("Socket error in recv loop")
                    self._connected.clear()
                break

            if len(buf) < need:
                continue

            # Parse complete messages — convert to bytes once per recv
            # iteration, then use offsets to avoid repeated copies.
            buf_bytes = bytes(buf)
            offset = 0
            while True:
                if (len(buf_bytes) - offset >= HEADER_SIZE
                        and (buf_bytes[offset:offset + 4] != MAGIC
                             or buf_bytes[offset + 4] != VERSION)):
                    # Lost framing (corrupt bytes or another protocol
                    # version): skip to the next magic instead of waiting
                    # forever for a message that never completes.
                    nxt = buf_bytes.find(MAGIC, offset + 1)
                    skip_to = nxt if nxt >= 0 else len(buf_bytes) - (len(MAGIC) - 1)
                    logger.warning("Bad RMSS header; skipped %d bytes", skip_to - offset)
                    offset = skip_to
                    continue
                result = StreamMessage.deserialize(buf_bytes, offset)
                if result is None:
                    break
                msg, consumed = result
                offset += consumed
                self.messages_received += 1

                mtype = msg.header.message_type
                if not isinstance(mtype, MessageType):
                    logger.debug("Skipping message with unknown opcode 0x%02X", mtype)
                    continue

                # Handle PING replies out-of-band (don't enqueue)
                if mtype == MessageType.PING:
                    self._ping_rtt = (
                        StreamHeader.now_timestamp() - msg.header.timestamp
                    ) / 1_000_000.0  # µs → s
                    self._ping_event.set()
                    continue

                # Track per-channel stats (frame-bearing messages only)
                ch = msg.header.channel_id
                is_frame = mtype in _FRAME_TYPES
                if is_frame:
                    with self._stats_lock:
                        if ch not in self._channel_stats:
                            self._channel_stats[ch] = ChannelStats(
                                channel_id=ch, _lock=self._stats_lock,
                            )
                        cs = self._channel_stats[ch]
                        cs.frames += 1
                        cs.bytes_compressed += len(msg.payload)
                        cs._timestamps.append(time.monotonic())
                        # Detect dropped frames
                        if cs.last_seq >= 0 and msg.header.sequence_num > cs.last_seq + 1:
                            cs.dropped += msg.header.sequence_num - cs.last_seq - 1
                        cs.last_seq = msg.header.sequence_num

                # Auto-decompress if enabled
                comp = msg.header.get_compression()
                if self._auto_decompress and comp != Compression.NONE:
                    try:
                        if mtype == MessageType.FRAME_RGBD:
                            msg.payload, msg.metadata = _decompress_rgbd(msg)
                        else:
                            _require_codecs(comp)
                            msg.payload = decompress_payload(
                                msg.payload, comp,
                            )
                        msg.header.set_compression(Compression.NONE)
                    except _MissingCodec as e:
                        if str(e) not in self._missing_codecs_warned:
                            self._missing_codecs_warned.add(str(e))
                            logger.warning(
                                "%s not installed: delivering those frames compressed "
                                "(pip install ramms-tools[compression])", e)
                    except Exception:
                        logger.exception("Decompression failed for channel %d", ch)

                if is_frame:
                    with self._stats_lock:
                        cs.bytes_total += len(msg.payload)

                # Deliver via callback
                if self.on_message:
                    try:
                        self.on_message(msg)
                    except Exception:
                        logger.exception("Error in on_message callback")

                # Also enqueue for polling
                with self._queue_lock:
                    self._queue.append(msg)
                    self._queue_event.set()

            # Compact buffer
            if offset > 0:
                del buf[:offset]
            header = StreamHeader.from_bytes(bytes(buf[:HEADER_SIZE]))
            need = header.total_message_size if header else HEADER_SIZE

    # ── Polling API ───────────────────────────────────────────────────

    def poll(self, timeout: float = 0.0) -> Optional[StreamMessage]:
        """Dequeue one inbound message. Returns None if empty."""
        if timeout > 0:
            self._queue_event.wait(timeout)
        with self._queue_lock:
            if self._queue:
                msg = self._queue.popleft()
                if not self._queue:
                    self._queue_event.clear()
                return msg
        return None

    def iter_messages(self, timeout: float = 1.0) -> Iterable[StreamMessage]:
        """Yield messages as they arrive.  Blocks up to *timeout* between messages."""
        self.start()
        try:
            while not self._stopping.is_set():
                msg = self.poll(timeout=timeout)
                if msg is not None:
                    yield msg
        finally:
            self.stop()

    # ── Internal ──────────────────────────────────────────────────────

    def _send(self, msg: StreamMessage) -> None:
        if self._sock is None:
            raise RuntimeError("Not connected")
        data = msg.serialize()
        self._sock.sendall(data)

    # ── Statistics ────────────────────────────────────────────────────

    def get_channel_stats(self) -> dict[int, ChannelStats]:
        """Return a snapshot of per-channel statistics."""
        with self._stats_lock:
            return dict(self._channel_stats)

    @property
    def uptime(self) -> float:
        """Seconds since connect()."""
        if self._connect_time <= 0:
            return 0.0
        return time.monotonic() - self._connect_time

    @property
    def total_fps(self) -> float:
        """Sum of FPS across all channels."""
        with self._stats_lock:
            return sum(cs.fps() for cs in self._channel_stats.values())
