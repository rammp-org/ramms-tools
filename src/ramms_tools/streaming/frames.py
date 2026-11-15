"""Decode RMSS camera frames into numpy arrays.

A camera with base channel C streams up to four roles, on the conventions
ramms-tools' StreamSender already uses toward UE and that UE now produces too:

    role    message           channel   stream_id                fmt      array
    color   FrameRGB   0x01   C         camera/<name>/color      bgra8    (H, W, 3) uint8 RGB
    depth   FrameDepth 0x02   C + 100   camera/<name>/depth      float32  (depth_h, depth_w) float32, cm
    motion  FrameMotion 0x04  C + 200   motion/<name>            rg32f    (motion_h, motion_w, 2) float32, px / frame
    mask    FrameData  0x11   C + 300   mask/<name>              mono8    (H, W) uint8 class values

Every frame's metadata carries ``stream_id``, ``group`` (the camera name),
``role``, ``name`` and ``fmt``, plus ``actor`` (the camera's actor path: stream
ids carry only the camera name, so pick a robot's camera by actor) and
``frame`` (the capture's frame number, the same for every role of one capture);
a mask's adds ``classes`` ({"<value>": "<class>"}), ``reserved`` (the stencil
values the sim keeps for itself; newer sims) and the value count
``{"NumSegmentIDs": n}`` under ``MaterialScalarParameters`` (read first) and
``material_params`` -- on newer sims high enough to cover the reserved values
and the frame's highest value too.

    from ramms_tools.streaming.frames import decode_message
    frame = decode_message(msg)          # Frame(role='depth', array=..., camera='front_camera', ...)

An IMU stream (Sensing.start_imu_stream) is FrameData with fmt ``imu_f64x18``:
every message carries the physics-step samples since the last one, ``w`` of
them (``h`` 1), each 18 little-endian float64 in ``IMU_FIELDS`` order (seq,
sim_time, acceleration cm/s^2 and angular velocity deg/s and angular
acceleration deg/s^2 in the sensor frame, velocity cm/s and orientation x, y,
z, w in the world). It decodes to role ``imu``, an (n, 18) float64 array, its
metadata naming ``actor``, ``target``, ``body``, ``source``, ``first_seq``,
``count`` and ``dropped``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from ramms_tools.streaming.protocol import Compression, MessageType, StreamMessage

ROLES = ("color", "depth", "motion", "mask")
ROLE_CHANNEL_OFFSETS = {"color": 0, "depth": 100, "motion": 200, "mask": 300}
ROLE_MESSAGE_TYPES = {
    "color": MessageType.FRAME_RGB,
    "depth": MessageType.FRAME_DEPTH,
    "motion": MessageType.FRAME_MOTION,
    "mask": MessageType.FRAME_DATA,
}
ROLE_FORMATS = {"color": "bgra8", "depth": "float32", "motion": "rg32f", "mask": "mono8"}
IMU_FORMAT = "imu_f64x18"
IMU_FIELDS = ("seq", "sim_time", "ax", "ay", "az", "gx", "gy", "gz", "alpha_x", "alpha_y", "alpha_z", "vx", "vy", "vz",
              "qx", "qy", "qz", "qw")

__all__ = [
    "ROLES", "ROLE_CHANNEL_OFFSETS", "ROLE_MESSAGE_TYPES", "ROLE_FORMATS", "Frame",
    "camera_stream_ids", "camera_channels", "message_role", "decode_message", "decode_rgbd",
    "decode_color", "decode_depth", "decode_motion", "decode_mask", "mask_classes", "decode_imu", "IMU_FORMAT", "IMU_FIELDS",
]


def camera_stream_ids(camera: str) -> dict[str, str]:
    """The stream id of each role of a camera."""
    return {"color": f"camera/{camera}/color", "depth": f"camera/{camera}/depth",
            "motion": f"motion/{camera}", "mask": f"mask/{camera}"}


def camera_channels(base: int) -> dict[str, int]:
    """The RMSS channel of each role of a camera with base channel `base`."""
    return {role: int(base) + off for role, off in ROLE_CHANNEL_OFFSETS.items()}


def _role_from_stream_id(sid: str) -> str:
    if sid.startswith("motion/"):
        return "motion"
    if sid.startswith("mask/"):
        return "mask"
    if sid.startswith("imu/"):
        return "imu"
    if sid.startswith("camera/"):
        tail = sid.rsplit("/", 1)[-1]
        return tail if tail in ("color", "depth") else ""
    return ""


def message_role(msg: StreamMessage, meta: Optional[dict] = None) -> str:
    """A frame's role: its metadata's `role`, else what its stream id or message type says ("" if unknown)."""
    meta = msg.get_metadata_json() if meta is None and msg.metadata else (meta or {})
    role = str(meta.get("role", "") or "").lower()
    if role in ROLES or role == "imu":
        return role
    if str(meta.get("fmt", "")).lower() == IMU_FORMAT:
        return "imu"
    role = _role_from_stream_id(str(meta.get("stream_id", "") or ""))
    if role:
        return role
    mt = msg.header.message_type
    if mt in (MessageType.FRAME_RGB, MessageType.IMAGE_DATA):
        return "color"
    if mt == MessageType.FRAME_DEPTH:
        return "depth"
    if mt == MessageType.FRAME_MOTION:
        return "motion"
    if mt == MessageType.FRAME_DATA and str(meta.get("fmt", "")).lower() in ("mono8", "r8", "gray8", "u8"):
        return "mask"
    return ""


@dataclass
class Frame:
    """One decoded camera frame."""
    role: str                       # color, depth, motion, mask
    array: np.ndarray               # see the module docstring for shapes and units
    meta: dict = field(default_factory=dict, repr=False)
    channel: int = 0
    sequence: int = 0
    timestamp_us: int = 0
    stream_id: str = ""
    camera: str = ""                # the metadata's group: the camera name
    fmt: str = ""
    classes: dict[int, str] = field(default_factory=dict)   # mask frames: class value -> fine class name
    num_segment_ids: int = 0
    actor: str = ""                 # the metadata's actor path ("" when the sender gives none)
    reserved: tuple[int, ...] = ()  # mask frames: the values the sim reserves (metadata "reserved"; () when absent)
    frame_number: Optional[int] = None   # the capture's frame number (metadata "frame"): equal across one capture's roles

    @property
    def height(self) -> int:
        return int(self.array.shape[0])

    @property
    def width(self) -> int:
        return int(self.array.shape[1])

    def __repr__(self) -> str:
        return (f"Frame(role={self.role!r}, camera={self.camera!r}, shape={tuple(self.array.shape)}, "
                f"dtype={self.array.dtype}, channel={self.channel}, seq={self.sequence})")


def _dims(meta: dict, prefix: str = "") -> tuple[int, int]:
    """(w, h) for a role: depth_w / motion_w when present, else w / h."""
    w = meta.get(f"{prefix}_w") if prefix else None
    h = meta.get(f"{prefix}_h") if prefix else None
    w = w if w else meta.get("w", meta.get("width", 0))
    h = h if h else meta.get("h", meta.get("height", 0))
    return int(w or 0), int(h or 0)


def _check(payload: bytes, need: int, what: str) -> None:
    if len(payload) != need:
        raise ValueError(f"{what}: payload is {len(payload)} bytes, expected {need}")


def decode_color(payload: bytes, meta: dict) -> np.ndarray:
    """A colour frame as (H, W, 3) uint8 RGB, from bgra8 (UE's own), rgba8, rgb8 or bgr8."""
    w, h = _dims(meta)
    fmt = str(meta.get("fmt", "bgra8")).lower()
    if fmt in ("bgra8", "rgba8", "bgra", "rgba"):
        _check(payload, w * h * 4, f"{fmt} {w}x{h}")
        a = np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 4)
        return a[:, :, [2, 1, 0]].copy() if fmt.startswith("bgr") else a[:, :, :3].copy()
    if fmt in ("rgb8", "bgr8", "rgb", "bgr"):
        _check(payload, w * h * 3, f"{fmt} {w}x{h}")
        a = np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 3)
        return a[:, :, ::-1].copy() if fmt.startswith("bgr") else a.copy()
    raise ValueError(f"colour format {fmt!r} is not one this decoder knows")


def decode_depth(payload: bytes, meta: dict) -> np.ndarray:
    """A depth frame as (h, w) float32 centimetres: float32 cm as UE sends it, or 16-bit millimetres."""
    w, h = _dims(meta, "depth")
    fmt = str(meta.get("fmt", "float32")).lower()
    if fmt in ("float32", "32fc1", "r32f"):
        _check(payload, w * h * 4, f"depth float32 {w}x{h}")
        return np.frombuffer(payload, dtype="<f4").reshape(h, w).astype(np.float32, copy=True)
    if fmt in ("16uc1", "uint16", "mono16", "g16"):
        _check(payload, w * h * 2, f"depth uint16 {w}x{h}")
        mm = np.frombuffer(payload, dtype="<u2").reshape(h, w)
        return (mm.astype(np.float32) * 0.1)
    raise ValueError(f"depth format {fmt!r} is not one this decoder knows")


def decode_motion(payload: bytes, meta: dict) -> np.ndarray:
    """Motion vectors as (h, w, 2) float32: x, y in pixels per frame."""
    w, h = _dims(meta, "motion")
    _check(payload, w * h * 8, f"motion rg32f {w}x{h}")
    return np.frombuffer(payload, dtype="<f4").reshape(h, w, 2).astype(np.float32, copy=True)


def decode_mask(payload: bytes, meta: dict) -> np.ndarray:
    """A segmentation mask as (h, w) uint8 class values (float32 masks stay float32)."""
    w, h = _dims(meta, "mask")
    fmt = str(meta.get("fmt", "mono8")).lower()
    if fmt == "float32":
        _check(payload, w * h * 4, f"mask float32 {w}x{h}")
        return np.frombuffer(payload, dtype="<f4").reshape(h, w).astype(np.float32, copy=True)
    if len(payload) == w * h * 4:
        # A PNG-compressed mask comes out of the generic decompressor as BGRA (L, L, L, 255): one channel is it.
        return np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 4)[:, :, 0].copy()
    _check(payload, w * h, f"mask mono8 {w}x{h}")
    return np.frombuffer(payload, dtype=np.uint8).reshape(h, w).copy()


def decode_imu(payload: bytes, meta: dict) -> np.ndarray:
    """An IMU stream message as an (n, 18) float64 array, one row per physics-step sample in IMU_FIELDS order."""
    fmt = str(meta.get("fmt", IMU_FORMAT)).lower()
    if fmt != IMU_FORMAT:
        raise ValueError(f"IMU format {fmt!r} is not one this decoder knows")
    n = meta.get("w", meta.get("count"))
    n = int(n) if n is not None else len(payload) // (18 * 8)
    _check(payload, n * 18 * 8, f"imu {n} samples")
    return np.frombuffer(payload, dtype="<f8").reshape(n, 18).astype(np.float64, copy=True)


def mask_classes(meta: dict) -> dict[int, str]:
    """A mask frame's class table: {"3": "Car"} -> {3: "Car"} (also accepts a list of {"value", "name"})."""
    raw = meta.get("classes") or {}
    out: dict[int, str] = {}
    if isinstance(raw, dict):
        for k, v in raw.items():
            try:
                out[int(k)] = str(v.get("name", v) if isinstance(v, dict) else v)
            except (TypeError, ValueError):
                continue
    elif isinstance(raw, list):
        for e in raw:
            if isinstance(e, dict) and "value" in e:
                out[int(e["value"])] = str(e.get("name", ""))
    return out


def _reserved(meta: dict) -> tuple[int, ...]:
    out = []
    for v in meta.get("reserved") or []:
        try:
            out.append(int(v))
        except (TypeError, ValueError):
            continue
    return tuple(out)


def _frame_number(meta: dict) -> Optional[int]:
    v = meta.get("frame")
    try:
        return int(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def _num_segment_ids(meta: dict) -> int:
    # The sim writes both keys; MaterialScalarParameters is the one StreamSender and rammp-ui's bridge use.
    for key in ("MaterialScalarParameters", "material_params"):
        mp = meta.get(key)
        if isinstance(mp, dict) and "NumSegmentIDs" in mp:
            try:
                return int(mp["NumSegmentIDs"])
            except (TypeError, ValueError):
                pass
    return 0


_DECODERS = {"color": decode_color, "depth": decode_depth, "motion": decode_motion, "mask": decode_mask, "imu": decode_imu}


def decode_message(msg: StreamMessage, role: Optional[str] = None) -> Frame:
    """
    Decode a frame message into a Frame. The role comes from the metadata (or the stream id / message type)
    unless given. Raises ValueError for a frame that is still compressed (its codec is not installed:
    pip install ramms-tools[compression]), of an unknown role or format, or of the wrong size.
    """
    meta = msg.get_metadata_json() if msg.metadata else {}
    if msg.header.get_compression() != Compression.NONE:
        raise ValueError(f"channel {msg.header.channel_id}: frame still {msg.header.get_compression().name}-compressed "
                         "(pip install ramms-tools[compression])")
    if msg.header.message_type == MessageType.FRAME_RGBD and role is None:
        raise ValueError("a combined RGBD frame holds two roles: use decode_rgbd")
    role = role or message_role(msg, meta)
    if role not in _DECODERS:
        raise ValueError(f"channel {msg.header.channel_id}: no camera role for {msg.header.message_type!r} "
                         f"(role {meta.get('role')!r}, fmt {meta.get('fmt')!r})")
    array = _DECODERS[role](bytes(msg.payload), meta)
    return Frame(role=role, array=array, meta=meta, channel=msg.header.channel_id, sequence=msg.header.sequence_num,
                 timestamp_us=msg.header.timestamp, stream_id=str(meta.get("stream_id", "") or ""),
                 camera=str(meta.get("group", "") or ""), fmt=str(meta.get("fmt", "") or ""),
                 classes=mask_classes(meta) if role == "mask" else {},
                 num_segment_ids=_num_segment_ids(meta) if role == "mask" else 0,
                 actor=str(meta.get("actor", "") or ""), frame_number=_frame_number(meta),
                 reserved=_reserved(meta) if role == "mask" else ())


def decode_rgbd(msg: StreamMessage) -> tuple[Frame, Frame]:
    """A combined FRAME_RGBD (BGRA8 colour then float32 depth, split at the metadata's rgb_size) as two Frames."""
    meta = msg.get_metadata_json() if msg.metadata else {}
    if msg.header.get_compression() != Compression.NONE:
        raise ValueError("RGBD frame still compressed (pip install ramms-tools[compression])")
    payload = bytes(msg.payload)
    w, h = _dims(meta)
    rgb_size = int(meta.get("rgb_size", w * h * 4))
    color = decode_color(payload[:rgb_size], dict(meta, fmt="bgra8"))
    depth = decode_depth(payload[rgb_size:], dict(meta, fmt="float32"))
    common = dict(channel=msg.header.channel_id, sequence=msg.header.sequence_num, timestamp_us=msg.header.timestamp,
                  stream_id=str(meta.get("stream_id", "") or ""), camera=str(meta.get("group", "") or ""),
                  actor=str(meta.get("actor", "") or ""), frame_number=_frame_number(meta))
    return (Frame("color", color, meta, fmt="bgra8", **common), Frame("depth", depth, meta, fmt="float32", **common))
