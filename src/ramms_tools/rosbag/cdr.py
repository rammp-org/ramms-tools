"""XCDR1 little-endian decoders and encoders for the ROS 2 types RAMMP bags record.

A rosbag2 sqlite3 bag stores each message as the bytes its publisher
serialized, and a Humble bag (storage schema v3) carries no copy of the .msg
definitions: there is no ``message_definitions`` table. So a decoder has to
bring the layouts with it. A generic one would parse .msg files, which would
then have to be vendored -- Apache-2.0 for the ROS packages, and no licence at
all for ``rammp_perception_interfaces``. Nine types cover every topic the RAMMP
perception bags record, and writing each layout out as code is shorter than
that parser would be.

What keeps hand-written layouts honest is the leftover rule in :func:`decode`:
a layout that differs from the publisher's either runs off the end of the
bytes or stops well short of it, and both raise. The opt-in tests run every
message of two real bags through it, and write each one back with
:func:`encode` to compare byte for byte.

The wire format, as rmw_fastrtps (Fast-CDR) writes it in Humble:

- A 4-byte encapsulation header: ``00 01`` is CDR little-endian, ``00 00``
  big-endian; the two option bytes are 0.
- Each primitive aligns to its own size, counted from the end of that header,
  not from the start of the buffer: a float64 sits 8-aligned from byte 4.
- ``string``: uint32 length *including* a NUL terminator, the UTF-8 bytes,
  the NUL.
- ``T[]``: uint32 count, then the elements; ``T[N]``: N elements, no count.
  An empty sequence is only its count -- Fast-CDR adds no padding for
  elements it does not write.
- ``bool`` and ``uint8`` are one byte; nested messages are laid out inline.
- The payload is zero-padded to a multiple of 4. Messages written by other
  tools (the vo bag's re-processed topics) are not, so up to 3 trailing bytes
  are padding, never content.

Decoded messages are plain dicts with the ROS field names, so a player can
index them without importing anything ROS-shaped. The shapes are documented
on :func:`decode`.
"""

from __future__ import annotations

import struct

__all__ = ["BagError", "BagDecodeError", "SUPPORTED_TYPES", "decode", "encode"]


class BagError(Exception):
    """A bag cannot be opened or read: missing, zipped, compressed, or not rosbag2 sqlite3."""


class BagDecodeError(BagError):
    """A message's bytes do not have the layout of the type it was recorded as."""


# Bytes a CDR payload may end with that belong to no field (see the module docstring).
MAX_PADDING = 3

_CDR_LE = b"\x00\x01"
_CDR_BE = b"\x00\x00"
_HEADER_LEN = 4
_NS_PER_S = 1_000_000_000

_U16 = struct.Struct("<H")
_I32 = struct.Struct("<i")
_U32 = struct.Struct("<I")
_F32 = struct.Struct("<f")
_F64 = struct.Struct("<d")


# ── the byte-level reader and writer ────────────────────────────────

class _Reader:
    """Reads one CDR payload front to back; every read is bounds-checked.

    Bounds-checking every read (rather than letting ``struct`` raise) is what
    turns a truncated or mis-laid-out message into a :class:`BagDecodeError`
    that names the type, instead of a ``struct.error`` from somewhere inside.
    """

    __slots__ = ("buf", "pos", "type_name")

    def __init__(self, type_name: str, buf: bytes):
        self.type_name = type_name
        self.buf = buf
        self.pos = _HEADER_LEN

    def _fail(self, what: str) -> BagDecodeError:
        return BagDecodeError(f"{self.type_name}: {what}")

    def _take(self, n: int) -> int:
        """Claim the next *n* bytes and return the offset they start at."""
        start = self.pos
        end = start + n
        if end > len(self.buf):
            raise self._fail(
                f"truncated: a field at byte {start} needs {n} bytes, but the message has only {len(self.buf)}"
            )
        self.pos = end
        return start

    def _align(self, size: int) -> None:
        # Relative to the end of the encapsulation header, not to byte 0.
        rem = (self.pos - _HEADER_LEN) % size
        if rem:
            self.pos += size - rem

    def u8(self) -> int:
        return self.buf[self._take(1)]

    def boolean(self) -> bool:
        return self.buf[self._take(1)] != 0

    def u16(self) -> int:
        self._align(2)
        return _U16.unpack_from(self.buf, self._take(2))[0]

    def i32(self) -> int:
        self._align(4)
        return _I32.unpack_from(self.buf, self._take(4))[0]

    def u32(self) -> int:
        self._align(4)
        return _U32.unpack_from(self.buf, self._take(4))[0]

    def f32(self) -> float:
        self._align(4)
        return _F32.unpack_from(self.buf, self._take(4))[0]

    def f32s(self, n: int) -> list[float]:
        if not n:
            return []
        self._align(4)
        return list(struct.unpack_from(f"<{n}f", self.buf, self._take(4 * n)))

    def f64s(self, n: int) -> list[float]:
        if not n:
            return []  # no alignment for an empty run (see the module docstring)
        self._align(8)
        return list(struct.unpack_from(f"<{n}d", self.buf, self._take(8 * n)))

    def count(self, min_element_size: int) -> int:
        """Read a sequence count, refusing one the remaining bytes cannot hold.

        Without this, a corrupt count of ~4 billion would loop for a long time
        before the first out-of-bounds read stopped it.
        """
        n = self.u32()
        left = len(self.buf) - self.pos
        if n * min_element_size > left:
            raise self._fail(f"truncated: a sequence of {n} elements cannot fit in the {left} bytes left")
        return n

    def f64_seq(self) -> list[float]:
        return self.f64s(self.count(8))

    def string(self) -> str:
        n = self.u32()
        start = self._take(n)
        end = start + n
        # Drop the NUL terminator. Like Fast-CDR, accept a string without one
        # (and a 0 length) rather than fail a message over it.
        if n and self.buf[end - 1] == 0:
            end -= 1
        # surrogateescape: bytes that are not UTF-8 survive, and encode() writes them back unchanged.
        return self.buf[start:end].decode("utf-8", "surrogateescape")

    def blob(self, keep: bool) -> bytes | None:
        """A ``uint8[]``: the bytes, or None (still bounds-checked) when not kept."""
        n = self.u32()
        start = self._take(n)
        return self.buf[start:start + n] if keep else None

    def seq(self, read_one, min_element_size: int = 1) -> list:
        return [read_one(self) for _ in range(self.count(min_element_size))]


class _Writer:
    """Builds one CDR payload, mirroring :class:`_Reader` call for call."""

    __slots__ = ("out",)

    def __init__(self):
        self.out = bytearray(_CDR_LE + b"\x00\x00")

    def _align(self, size: int) -> None:
        rem = (len(self.out) - _HEADER_LEN) % size
        if rem:
            self.out += bytes(size - rem)

    def u8(self, v: int) -> None:
        self.out.append(v)

    def boolean(self, v) -> None:
        self.out.append(1 if v else 0)

    def u16(self, v: int) -> None:
        self._align(2)
        self.out += _U16.pack(v)

    def i32(self, v: int) -> None:
        self._align(4)
        self.out += _I32.pack(v)

    def u32(self, v: int) -> None:
        self._align(4)
        self.out += _U32.pack(v)

    def f32(self, v: float) -> None:
        self._align(4)
        self.out += _F32.pack(v)

    def _fixed(self, values, n: int | None) -> list:
        values = list(values)
        if n is not None and len(values) != n:
            raise ValueError(f"a fixed array of {n} values was given {len(values)}")
        return values

    def f32s(self, values, n: int | None = None) -> None:
        values = self._fixed(values, n)
        if values:
            self._align(4)
            self.out += struct.pack(f"<{len(values)}f", *values)

    def f64s(self, values, n: int | None = None) -> None:
        values = self._fixed(values, n)
        if values:
            self._align(8)
            self.out += struct.pack(f"<{len(values)}d", *values)

    def f64_seq(self, values) -> None:
        values = list(values)
        self.u32(len(values))
        self.f64s(values)

    def string(self, s: str) -> None:
        raw = s.encode("utf-8", "surrogateescape")
        self.u32(len(raw) + 1)
        self.out += raw
        self.out.append(0)

    def blob(self, data) -> None:
        if data is None:
            raise ValueError("a payload is None: it was decoded with keep_data=False, so its bytes are gone")
        if not isinstance(data, (bytes, bytearray)):
            data = memoryview(data).tobytes()  # TypeError for an int or str, which bytes() would accept
        self.u32(len(data))
        self.out += data

    def seq(self, items, write_one) -> None:
        items = list(items)
        self.u32(len(items))
        for item in items:
            write_one(self, item)

    def finish(self) -> bytes:
        # Zero-pad to a multiple of 4, as rmw_fastrtps does, so a payload built
        # here is byte-identical to one recorded on the robot.
        self.out += bytes(-len(self.out) % 4)
        return bytes(self.out)


# ── shared pieces ───────────────────────────────────────────────────
# Each decoder has its encoder right below it, field for field, so a layout
# change is made to both in one place. Dict displays evaluate left to right,
# which is what makes ``{"a": r.u32(), "b": r.string()}`` read in field order.

def _dec_header(r: _Reader) -> dict:
    sec = r.i32()
    nanosec = r.u32()
    return {"stamp_ns": sec * _NS_PER_S + nanosec, "frame_id": r.string()}


def _enc_header(w: _Writer, h: dict) -> None:
    # floor division keeps a pre-1970 stamp in ROS's form: negative sec, nanosec in [0, 1e9).
    sec, nanosec = divmod(h["stamp_ns"], _NS_PER_S)
    w.i32(sec)
    w.u32(nanosec)
    w.string(h["frame_id"])


def _dec_duration(r: _Reader) -> dict:
    return {"sec": r.i32(), "nanosec": r.u32()}


def _enc_duration(w: _Writer, d: dict) -> None:
    w.i32(d["sec"])
    w.u32(d["nanosec"])


def _dec_vec3(r: _Reader) -> dict:
    x, y, z = r.f64s(3)
    return {"x": x, "y": y, "z": z}


def _enc_vec3(w: _Writer, v: dict) -> None:
    w.f64s((v["x"], v["y"], v["z"]))


def _dec_quat(r: _Reader) -> dict:
    x, y, z, qw = r.f64s(4)
    return {"x": x, "y": y, "z": z, "w": qw}


def _enc_quat(w: _Writer, q: dict) -> None:
    w.f64s((q["x"], q["y"], q["z"], q["w"]))


def _dec_pose(r: _Reader) -> dict:
    return {"position": _dec_vec3(r), "orientation": _dec_quat(r)}


def _enc_pose(w: _Writer, p: dict) -> None:
    _enc_vec3(w, p["position"])
    _enc_quat(w, p["orientation"])


def _dec_color(r: _Reader) -> dict:
    cr, cg, cb, ca = r.f32s(4)
    return {"r": cr, "g": cg, "b": cb, "a": ca}


def _enc_color(w: _Writer, c: dict) -> None:
    w.f32s((c["r"], c["g"], c["b"], c["a"]))


# ── sensor_msgs ─────────────────────────────────────────────────────

def _dec_image(r: _Reader, keep: bool) -> dict:
    return {
        "header": _dec_header(r),
        "height": r.u32(),
        "width": r.u32(),
        "encoding": r.string(),
        "is_bigendian": r.u8(),
        "step": r.u32(),
        "data": r.blob(keep),
    }


def _enc_image(w: _Writer, m: dict) -> None:
    _enc_header(w, m["header"])
    w.u32(m["height"])
    w.u32(m["width"])
    w.string(m["encoding"])
    w.u8(m["is_bigendian"])
    w.u32(m["step"])
    w.blob(m["data"])


def _dec_compressed_image(r: _Reader, keep: bool) -> dict:
    return {"header": _dec_header(r), "format": r.string(), "data": r.blob(keep)}


def _enc_compressed_image(w: _Writer, m: dict) -> None:
    _enc_header(w, m["header"])
    w.string(m["format"])
    w.blob(m["data"])


def _dec_camera_info(r: _Reader, keep: bool) -> dict:
    return {
        "header": _dec_header(r),
        "height": r.u32(),
        "width": r.u32(),
        "distortion_model": r.string(),
        "d": r.f64_seq(),
        "k": r.f64s(9),
        "r": r.f64s(9),
        "p": r.f64s(12),
        "binning_x": r.u32(),
        "binning_y": r.u32(),
        "roi": {
            "x_offset": r.u32(),
            "y_offset": r.u32(),
            "height": r.u32(),
            "width": r.u32(),
            "do_rectify": r.boolean(),
        },
    }


def _enc_camera_info(w: _Writer, m: dict) -> None:
    _enc_header(w, m["header"])
    w.u32(m["height"])
    w.u32(m["width"])
    w.string(m["distortion_model"])
    w.f64_seq(m["d"])
    w.f64s(m["k"], 9)
    w.f64s(m["r"], 9)
    w.f64s(m["p"], 12)
    w.u32(m["binning_x"])
    w.u32(m["binning_y"])
    roi = m["roi"]
    w.u32(roi["x_offset"])
    w.u32(roi["y_offset"])
    w.u32(roi["height"])
    w.u32(roi["width"])
    w.boolean(roi["do_rectify"])


def _dec_joint_state(r: _Reader, keep: bool) -> dict:
    return {
        "header": _dec_header(r),
        "name": r.seq(_Reader.string, 4),
        "position": r.f64_seq(),
        "velocity": r.f64_seq(),
        "effort": r.f64_seq(),
    }


def _enc_joint_state(w: _Writer, m: dict) -> None:
    _enc_header(w, m["header"])
    w.seq(m["name"], _Writer.string)
    w.f64_seq(m["position"])
    w.f64_seq(m["velocity"])
    w.f64_seq(m["effort"])


# ── nav_msgs / tf2_msgs ─────────────────────────────────────────────

def _dec_odometry(r: _Reader, keep: bool) -> dict:
    return {
        "header": _dec_header(r),
        "child_frame_id": r.string(),
        "pose": {"pose": _dec_pose(r), "covariance": r.f64s(36)},
        "twist": {"twist": {"linear": _dec_vec3(r), "angular": _dec_vec3(r)}, "covariance": r.f64s(36)},
    }


def _enc_odometry(w: _Writer, m: dict) -> None:
    _enc_header(w, m["header"])
    w.string(m["child_frame_id"])
    _enc_pose(w, m["pose"]["pose"])
    w.f64s(m["pose"]["covariance"], 36)
    _enc_vec3(w, m["twist"]["twist"]["linear"])
    _enc_vec3(w, m["twist"]["twist"]["angular"])
    w.f64s(m["twist"]["covariance"], 36)


def _dec_transform_stamped(r: _Reader) -> dict:
    return {
        "header": _dec_header(r),
        "child_frame_id": r.string(),
        "transform": {"translation": _dec_vec3(r), "rotation": _dec_quat(r)},
    }


def _enc_transform_stamped(w: _Writer, t: dict) -> None:
    _enc_header(w, t["header"])
    w.string(t["child_frame_id"])
    _enc_vec3(w, t["transform"]["translation"])
    _enc_quat(w, t["transform"]["rotation"])


def _dec_tf_message(r: _Reader, keep: bool) -> dict:
    return {"transforms": r.seq(_dec_transform_stamped, 8)}


def _enc_tf_message(w: _Writer, m: dict) -> None:
    w.seq(m["transforms"], _enc_transform_stamped)


# ── visualization_msgs (Humble Marker: texture, uv_coordinates, mesh_file) ──

def _dec_marker(r: _Reader, keep: bool) -> dict:
    return {
        "header": _dec_header(r),
        "ns": r.string(),
        "id": r.i32(),
        "type": r.i32(),
        "action": r.i32(),
        "pose": _dec_pose(r),
        "scale": _dec_vec3(r),
        "color": _dec_color(r),
        "lifetime": _dec_duration(r),
        "frame_locked": r.boolean(),
        "points": r.seq(_dec_vec3, 24),
        "colors": r.seq(_dec_color, 16),
        "texture_resource": r.string(),
        "texture": _dec_compressed_image(r, keep),
        "uv_coordinates": r.seq(lambda r_: dict(zip(("u", "v"), r_.f32s(2))), 8),
        "text": r.string(),
        "mesh_resource": r.string(),
        "mesh_file": {"filename": r.string(), "data": r.blob(keep)},
        "mesh_use_embedded_materials": r.boolean(),
    }


def _enc_marker(w: _Writer, m: dict) -> None:
    _enc_header(w, m["header"])
    w.string(m["ns"])
    w.i32(m["id"])
    w.i32(m["type"])
    w.i32(m["action"])
    _enc_pose(w, m["pose"])
    _enc_vec3(w, m["scale"])
    _enc_color(w, m["color"])
    _enc_duration(w, m["lifetime"])
    w.boolean(m["frame_locked"])
    w.seq(m["points"], _enc_vec3)
    w.seq(m["colors"], _enc_color)
    w.string(m["texture_resource"])
    _enc_compressed_image(w, m["texture"])
    w.seq(m["uv_coordinates"], lambda w_, uv: w_.f32s((uv["u"], uv["v"])))
    w.string(m["text"])
    w.string(m["mesh_resource"])
    w.string(m["mesh_file"]["filename"])
    w.blob(m["mesh_file"]["data"])
    w.boolean(m["mesh_use_embedded_materials"])


def _dec_marker_array(r: _Reader, keep: bool) -> dict:
    return {"markers": r.seq(lambda r_: _dec_marker(r_, keep), 8)}


def _enc_marker_array(w: _Writer, m: dict) -> None:
    w.seq(m["markers"], _enc_marker)


# ── rammp_perception_interfaces 2.2.0 (DetectedObject has `size`) ────

def _dec_detected_object(r: _Reader) -> dict:
    return {
        "instance_id": r.string(),
        "perception_class": r.string(),
        "score": r.f32(),
        "state": r.u8(),          # then 3 pad bytes: box is float32
        "box": r.f32s(4),         # [x_min, y_min, x_max, y_max], source pixels
        "mask_id": r.u16(),
        "has_pose": r.boolean(),  # then 1 or 5 pad bytes: Pose is float64
        "pose": _dec_pose(r),
        "size": _dec_vec3(r),
    }


def _enc_detected_object(w: _Writer, o: dict) -> None:
    w.string(o["instance_id"])
    w.string(o["perception_class"])
    w.f32(o["score"])
    w.u8(o["state"])
    w.f32s(o["box"], 4)
    w.u16(o["mask_id"])
    w.boolean(o["has_pose"])
    _enc_pose(w, o["pose"])
    _enc_vec3(w, o["size"])


def _dec_object_detections(r: _Reader, keep: bool) -> dict:
    return {
        "header": _dec_header(r),
        "objects": r.seq(_dec_detected_object, 8),
        "label_image": _dec_image(r, keep),
    }


def _enc_object_detections(w: _Writer, m: dict) -> None:
    _enc_header(w, m["header"])
    w.seq(m["objects"], _enc_detected_object)
    _enc_image(w, m["label_image"])


# ── std_msgs ────────────────────────────────────────────────────────

def _dec_string(r: _Reader, keep: bool) -> dict:
    return {"data": r.string()}


def _enc_string(w: _Writer, m: dict) -> None:
    w.string(m["data"])


_CODECS = {
    "sensor_msgs/msg/Image": (_dec_image, _enc_image),
    "sensor_msgs/msg/CompressedImage": (_dec_compressed_image, _enc_compressed_image),
    "sensor_msgs/msg/CameraInfo": (_dec_camera_info, _enc_camera_info),
    "sensor_msgs/msg/JointState": (_dec_joint_state, _enc_joint_state),
    "nav_msgs/msg/Odometry": (_dec_odometry, _enc_odometry),
    "tf2_msgs/msg/TFMessage": (_dec_tf_message, _enc_tf_message),
    "visualization_msgs/msg/MarkerArray": (_dec_marker_array, _enc_marker_array),
    "rammp_perception_interfaces/msg/ObjectDetections": (_dec_object_detections, _enc_object_detections),
    "std_msgs/msg/String": (_dec_string, _enc_string),
}

#: The message types :func:`decode` and :func:`encode` handle.
SUPPORTED_TYPES = tuple(_CODECS)


def decode(type_name: str, data: bytes, keep_data: bool = True) -> dict:
    """Decode one CDR message of *type_name* into a dict.

    *keep_data* False skips the bytes of image-like payloads (``Image.data``,
    ``CompressedImage.data``, ``ObjectDetections.label_image.data``, a
    Marker's ``texture.data`` and ``mesh_file.data``): they come back as None,
    which makes a header-only look at a 768 KB frame cheap. The skipped bytes
    are still bounds-checked, so a truncated message raises either way.

    Raises :class:`BagDecodeError` for a type not in :data:`SUPPORTED_TYPES`,
    an encapsulation other than CDR little-endian, a message that ends before
    its last field, or one with more than 3 bytes left after it -- a layout
    mismatch, since only padding may follow the last field.

    Shapes (every header is ``{"stamp_ns": int, "frame_id": str}`` with
    ``stamp_ns = sec * 1e9 + nanosec``; a Vector3/Point is ``{x, y, z}``, a
    Quaternion ``{x, y, z, w}``, a Pose ``{"position", "orientation"}``)::

        Image            {header, height, width, encoding, is_bigendian, step, data}
        CompressedImage  {header, format, data}
        CameraInfo       {header, height, width, distortion_model, d, k[9], r[9], p[12],
                          binning_x, binning_y, roi: {x_offset, y_offset, height, width, do_rectify}}
        JointState       {header, name, position, velocity, effort}
        Odometry         {header, child_frame_id, pose: {pose, covariance[36]},
                          twist: {twist: {linear, angular}, covariance[36]}}
        TFMessage        {transforms: [{header, child_frame_id, transform: {translation, rotation}}]}
        MarkerArray      {markers: [{header, ns, id, type, action, pose, scale, color: {r, g, b, a},
                          lifetime: {sec, nanosec}, frame_locked, points, colors, texture_resource,
                          texture: CompressedImage, uv_coordinates: [{u, v}], text, mesh_resource,
                          mesh_file: {filename, data}, mesh_use_embedded_materials}]}
        ObjectDetections {header, objects: [{instance_id, perception_class, score, state, box[4],
                          mask_id, has_pose, pose, size}], label_image: Image}
        String           {data}
    """
    codec = _CODECS.get(type_name)
    if codec is None:
        raise BagDecodeError(f"no decoder for {type_name!r}; this decoder reads {', '.join(SUPPORTED_TYPES)}")
    if not isinstance(data, bytes):
        data = bytes(data)
    if len(data) < _HEADER_LEN:
        raise BagDecodeError(f"{type_name}: truncated: {len(data)} bytes is shorter than the 4-byte CDR header")
    kind = data[:2]
    if kind != _CDR_LE:
        what = "big-endian CDR" if kind == _CDR_BE else "not plain CDR"
        raise BagDecodeError(
            f"{type_name}: encapsulation {kind.hex()} is {what}; this decoder reads CDR little-endian (0001) only"
        )
    r = _Reader(type_name, data)
    msg = codec[0](r, keep_data)
    left = len(data) - r.pos
    if left > MAX_PADDING:
        raise BagDecodeError(
            f"{type_name}: {left} bytes left over after the last field -- more than the {MAX_PADDING} bytes of "
            "padding a CDR message may end with, so the bytes have a different layout than this decoder's"
        )
    return msg


def encode(type_name: str, msg: dict) -> bytes:
    """Serialize *msg* (a dict in :func:`decode`'s shape) as CDR little-endian.

    The exact inverse of :func:`decode`: ``decode(t, encode(t, m)) == m``.
    float32 fields (scores, boxes, colours) hold only float32 values, so a
    value such as 0.1 comes back as its nearest float32. The payload is
    zero-padded to a multiple of 4, as rmw_fastrtps does, so re-encoding a
    message recorded on the robot reproduces its bytes. Raises ValueError for
    an unsupported type, a missing or ill-typed field, or a payload that was
    decoded with ``keep_data=False``.
    """
    codec = _CODECS.get(type_name)
    if codec is None:
        raise ValueError(f"no encoder for {type_name!r}; this encoder writes {', '.join(SUPPORTED_TYPES)}")
    w = _Writer()
    try:
        codec[1](w, msg)
    except KeyError as exc:
        raise ValueError(f"cannot encode {type_name}: missing field {exc.args[0]!r}") from exc
    except (TypeError, ValueError, OverflowError, struct.error) as exc:
        raise ValueError(f"cannot encode {type_name}: {exc}") from exc
    return w.finish()
