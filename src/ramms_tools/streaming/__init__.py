"""RMSS binary streaming protocol — Python implementation.

Provides StreamClient for receiving data from UE and StreamSender for
sending data to UE, plus the low-level protocol module for
message framing and compression utilities.
"""

from ramms_tools.streaming.protocol import (
    HEADER_SIZE,
    MessageType,
    Compression,
    StreamHeader,
    StreamMessage,
)
from ramms_tools.streaming.client import StreamClient, ChannelStats
from ramms_tools.streaming.sender import StreamSender
from ramms_tools.streaming.compression import (
    has_jpeg,
    has_lz4,
    decompress_payload,
    compress_jpeg,
    compress_lz4,
)

__all__ = [
    "HEADER_SIZE",
    "MessageType",
    "Compression",
    "StreamHeader",
    "StreamMessage",
    "StreamClient",
    "ChannelStats",
    "StreamSender",
    "has_jpeg",
    "has_lz4",
    "decompress_payload",
    "compress_jpeg",
    "compress_lz4",
]
