"""RMSS compression/decompression utilities.

Supports JPEG (via Pillow) and LZ4 (via lz4 package) with graceful
fallback when optional dependencies are not installed.
"""

from __future__ import annotations

import io
import logging
import struct
from typing import Optional

from ramms_tools.streaming.protocol import Compression

logger = logging.getLogger(__name__)

# ── Optional dependency availability ─────────────────────────────────────

_has_pillow = False
_has_lz4 = False

try:
    from PIL import Image

    _has_pillow = True
except ImportError:
    pass

try:
    import lz4.block  # type: ignore[import-untyped]

    _has_lz4 = True
except ImportError:
    pass


def has_jpeg() -> bool:
    """Return True if JPEG decompression is available (Pillow installed)."""
    return _has_pillow


def has_lz4() -> bool:
    """Return True if LZ4 decompression is available (lz4 installed)."""
    return _has_lz4


# ── JPEG ─────────────────────────────────────────────────────────────────


def decompress_jpeg(data: bytes, output_format: str = "bgra") -> Optional[bytes]:
    """Decompress JPEG bytes to raw pixel data.

    Returns BGRA8 (or RGBA8 if *output_format* = ``"rgba"``) bytes,
    or ``None`` if Pillow is not available.
    """
    if not _has_pillow:
        logger.warning("Pillow not installed — cannot decompress JPEG")
        return None

    img = Image.open(io.BytesIO(data))
    img = img.convert("RGBA")  # ensure 4 channels

    if output_format == "bgra":
        # Swap R and B channels
        r, g, b, a = img.split()
        img = Image.merge("RGBA", (b, g, r, a))

    return img.tobytes()


def compress_jpeg(
    pixel_data: bytes, width: int, height: int, quality: int = 85,
    input_format: str = "bgra",
) -> Optional[bytes]:
    """Compress raw pixel data to JPEG.

    *pixel_data* is expected to be BGRA8 (or RGBA8 if *input_format* = ``"rgba"``).
    Returns JPEG bytes or ``None`` if Pillow is not available.
    """
    if not _has_pillow:
        logger.warning("Pillow not installed — cannot compress JPEG")
        return None

    if input_format == "bgra":
        # Convert BGRA → RGBA for Pillow
        img = Image.frombytes("RGBA", (width, height), pixel_data)
        r, g, b, a = img.split()
        img = Image.merge("RGBA", (b, g, r, a))
    else:
        img = Image.frombytes("RGBA", (width, height), pixel_data)

    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


# ── LZ4 ──────────────────────────────────────────────────────────────────


def decompress_lz4(data: bytes) -> Optional[bytes]:
    """Decompress LZ4 data.

    The UE encoder prefixes 4 bytes (little-endian int32) with the
    uncompressed size.  This function reads that prefix and uses it
    as the *uncompressed_size* hint for ``lz4.block.decompress``.

    Returns decompressed bytes or ``None`` if lz4 is not available.
    """
    if not _has_lz4:
        logger.warning("lz4 not installed — cannot decompress LZ4")
        return None

    if len(data) < 4:
        logger.warning("LZ4 payload too small (< 4 bytes)")
        return None

    (uncompressed_size,) = struct.unpack("<i", data[:4])
    compressed = data[4:]

    return lz4.block.decompress(compressed, uncompressed_size=uncompressed_size)


def compress_lz4(data: bytes) -> Optional[bytes]:
    """Compress data with LZ4, prefixing 4-byte uncompressed size.

    Returns compressed bytes (with size prefix) or ``None`` if lz4 is
    not available.
    """
    if not _has_lz4:
        logger.warning("lz4 not installed — cannot compress LZ4")
        return None

    compressed = lz4.block.compress(data, store_size=False)
    return struct.pack("<i", len(data)) + compressed


# ── Dispatcher ───────────────────────────────────────────────────────────


def decompress_payload(
    data: bytes, compression: Compression, width: int = 0, height: int = 0,
) -> bytes:
    """Decompress *data* according to *compression* type.

    Returns the original bytes unchanged if compression is NONE or if the
    required library is missing (with a warning).
    """
    if compression == Compression.NONE:
        return data

    if compression == Compression.JPEG:
        result = decompress_jpeg(data)
        if result is not None:
            return result
        return data  # fallback: return compressed bytes

    if compression == Compression.LZ4:
        result = decompress_lz4(data)
        if result is not None:
            return result
        return data

    if compression == Compression.PNG:
        if not _has_pillow:
            logger.warning("Pillow not installed — cannot decompress PNG")
            return data
        img = Image.open(io.BytesIO(data))
        img = img.convert("RGBA")
        r, g, b, a = img.split()
        img = Image.merge("RGBA", (b, g, r, a))
        return img.tobytes()

    logger.warning("Unknown compression type: %s", compression)
    return data
