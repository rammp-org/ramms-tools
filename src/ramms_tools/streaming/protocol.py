"""RMSS binary streaming protocol — wire format and serialization.

All multi-byte fields are little-endian.  The 32-byte header layout is:

    Offset  Size  Field
    ------  ----  -----
     0       4    Magic ("RMSS")
     4       1    Version (1)
     5       1    MessageType
     6       2    ChannelID
     8       2    Flags
    10       4    SequenceNum
    14       8    Timestamp (µs since epoch)
    22       4    MetadataLen
    26       4    PayloadLen
    30       2    Reserved

This module uses only the Python standard library (struct).
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


MAGIC = b"RMSS"
VERSION = 1
HEADER_SIZE = 32
HEADER_FMT = "<4sBBHHIqIIH"  # 32 bytes total

# Sanity-check that the struct matches HEADER_SIZE
assert struct.calcsize(HEADER_FMT) == HEADER_SIZE, (
    f"Header format size mismatch: {struct.calcsize(HEADER_FMT)} != {HEADER_SIZE}"
)


class MessageType(IntEnum):
    """RMSS message types."""
    NONE          = 0x00
    FRAME_RGB     = 0x01
    FRAME_DEPTH   = 0x02
    FRAME_RGBD    = 0x03
    FRAME_MOTION  = 0x04
    POINT_CLOUD   = 0x05
    OCTO_MAP      = 0x06
    IMAGE_DATA    = 0x10
    METADATA_ONLY = 0xF0
    SUBSCRIBE     = 0xF1
    UNSUBSCRIBE   = 0xF2
    ACK           = 0xFD
    ERROR         = 0xFE
    PING          = 0xFF


class Compression(IntEnum):
    """Compression types stored in the Flags field."""
    NONE = 0
    LZ4  = 1
    JPEG = 2
    PNG  = 3


# Flag bit constants
FLAG_COMPRESSED      = 0x0001
FLAG_COMP_TYPE_MASK  = 0x0006
FLAG_COMP_TYPE_SHIFT = 1
FLAG_HAS_ALPHA       = 0x0008
FLAG_HIGH_PRIORITY   = 0x0010


@dataclass
class StreamHeader:
    """32-byte RMSS message header."""
    message_type: MessageType = MessageType.NONE
    channel_id: int = 0
    flags: int = 0
    sequence_num: int = 0
    timestamp: int = 0  # microseconds since epoch
    metadata_len: int = 0
    payload_len: int = 0

    @property
    def total_message_size(self) -> int:
        return HEADER_SIZE + self.metadata_len + self.payload_len

    # ── Compression helpers ───────────────────────────────────────────

    def set_compression(self, comp: Compression) -> None:
        self.flags &= ~(FLAG_COMPRESSED | FLAG_COMP_TYPE_MASK)
        if comp != Compression.NONE:
            self.flags |= FLAG_COMPRESSED
            self.flags |= (int(comp) << FLAG_COMP_TYPE_SHIFT) & FLAG_COMP_TYPE_MASK

    def get_compression(self) -> Compression:
        if not (self.flags & FLAG_COMPRESSED):
            return Compression.NONE
        return Compression((self.flags & FLAG_COMP_TYPE_MASK) >> FLAG_COMP_TYPE_SHIFT)

    # ── Serialization ─────────────────────────────────────────────────

    def to_bytes(self) -> bytes:
        return struct.pack(
            HEADER_FMT,
            MAGIC,
            VERSION,
            int(self.message_type),
            self.channel_id,
            self.flags,
            self.sequence_num,
            self.timestamp,
            self.metadata_len,
            self.payload_len,
            0,  # reserved
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional["StreamHeader"]:
        """Parse a 32-byte header.  Returns None on magic/version mismatch."""
        if len(data) < HEADER_SIZE:
            return None
        (magic, ver, msg_type, channel, flags, seq,
         ts, meta_len, payload_len, _reserved) = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        if magic != MAGIC or ver != VERSION:
            return None
        h = cls()
        h.message_type = MessageType(msg_type)
        h.channel_id = channel
        h.flags = flags
        h.sequence_num = seq
        h.timestamp = ts
        h.metadata_len = meta_len
        h.payload_len = payload_len
        return h

    @staticmethod
    def now_timestamp() -> int:
        """Return current time as microseconds since epoch."""
        return int(time.time() * 1_000_000)


@dataclass
class StreamMessage:
    """Complete RMSS message: header + metadata + payload."""
    header: StreamHeader = field(default_factory=StreamHeader)
    metadata: bytes = b""
    payload: bytes = b""

    # ── Convenience ───────────────────────────────────────────────────

    def set_metadata_string(self, s: str) -> None:
        self.metadata = s.encode("utf-8")

    def get_metadata_string(self) -> str:
        return self.metadata.decode("utf-8") if self.metadata else ""

    def get_metadata_json(self) -> dict:
        import json
        s = self.get_metadata_string()
        return json.loads(s) if s else {}

    # ── Serialization ─────────────────────────────────────────────────

    def serialize(self) -> bytes:
        h = self.header
        h.metadata_len = len(self.metadata)
        h.payload_len = len(self.payload)
        return h.to_bytes() + self.metadata + self.payload

    @classmethod
    def deserialize(cls, buf: bytes, offset: int = 0) -> Optional[tuple["StreamMessage", int]]:
        """
        Try to deserialize one message starting at *offset* in *buf*.

        Returns (message, bytes_consumed) or None if not enough data.
        """
        available = len(buf) - offset
        if available < HEADER_SIZE:
            return None

        header = StreamHeader.from_bytes(buf[offset:offset + HEADER_SIZE])
        if header is None:
            return None

        total = header.total_message_size
        if available < total:
            return None  # need more data

        meta_start = offset + HEADER_SIZE
        meta_end = meta_start + header.metadata_len
        payload_end = meta_end + header.payload_len

        msg = cls()
        msg.header = header
        msg.metadata = buf[meta_start:meta_end]
        msg.payload = buf[meta_end:payload_end]

        return (msg, total)
