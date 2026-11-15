"""The RammsSensingBridge over Remote Control: sensors on an actor, their readings, and the camera streams."""

from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field, fields
from typing import Any, Iterable, Optional, Union

from ramms_tools._rc import as_bool, as_float, as_json, name as _fname, return_value, rotator, short_name, vec3
from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError

BRIDGE = "/Script/RammsCore.Default__RammsSensingBridge"
KINDS = ("ToF", "Sonar", "IMU", "Camera")
_KIND_BY_LOWER = {k.lower(): k for k in KINDS}
_ROLE_OFFSETS = {"color": 0, "depth": 100, "motion": 200, "mask": 300, "rgbd": 0}


def same_actor(a: str, b: str) -> bool:
    """Do two actor references name the same actor: equal paths, or a name equal to the other's last part (the
    bridge takes either; frame metadata and stream status carry the full path)."""
    a, b = str(a or ""), str(b or "")
    if not a or not b:
        return False
    if a == b:
        return True
    return short_name(a).lower() == short_name(b).lower() and ("." not in a or "." not in b)


def _sim_time(d: dict) -> Optional[float]:
    """An IMU sample's engine time: None when the sim leaves it out, sends null, or -1 (no sample yet)."""
    v = d.get("sim_time")
    if v is None:
        return None
    f = as_float(v, -1.0)
    return f if f >= 0.0 else None


class SensingError(UnrealRemoteError):
    """The sensing bridge answered {"ok": false, "error": ...}, or a mount / option is not valid."""


# What the bridge accepts as a sensor name (SENSING_SPEC 3.2): it is the component's name and the camera's stream name.
_NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,64}$")
_RESERVED_SUFFIXES = ("_depth", "_dmv", "_seg", "_stream")       # the sim's own helper components per camera
MAX_CAMERA_SIZE = 4096


def check_sensor_name(name: str) -> str:
    """A sensor name the bridge takes: 1..64 of A-Z a-z 0-9 _ - . and not ending in a reserved helper suffix
    (_depth, _dmv, _seg, _stream). Returns it; raises SensingError."""
    n = str(name).strip()
    if not _NAME_RE.match(n):
        raise SensingError(f"sensor name {name!r}: 1..64 characters of A-Z a-z 0-9 _ - .")
    if n.lower().endswith(_RESERVED_SUFFIXES):
        raise SensingError(f"sensor name {name!r} ends in a suffix the sim keeps for its helpers ({', '.join(_RESERVED_SUFFIXES)})")
    return n


def canonical_kind(kind: str) -> str:
    """"tof" -> "ToF", "imu" -> "IMU" (the bridge takes any case; this is for display and checks)."""
    k = _KIND_BY_LOWER.get(str(kind).strip().lower())
    if k is None:
        raise SensingError(f"sensor kind must be one of {', '.join(KINDS)}, not {kind!r}")
    return k


# ── mounts ────────────────────────────────────────────────────────────

_COMMON_KEYS = ("name", "kind", "parent", "socket", "location", "rotation")
_KIND_KEYS = {
    "ToF": ("rows", "columns", "h_fov_deg", "v_fov_deg", "max_range_cm"),
    "Sonar": ("sonar_max_range_cm", "sonar_cone_deg"),
    "IMU": (),
    "Camera": ("width", "height", "fov_deg", "color", "depth", "motion", "segmentation", "channel"),
}


def _triple(v: Any, keys: tuple[str, str, str]) -> tuple[float, float, float]:
    if isinstance(v, dict):
        return tuple(as_float(v.get(k, v.get(k.capitalize(), v.get(k.upper(), 0.0)))) for k in keys)  # type: ignore[return-value]
    if isinstance(v, str):
        v = [p for p in v.replace(",", " ").split() if p]
    if isinstance(v, (list, tuple)) and len(v) == 3:
        return (float(v[0]), float(v[1]), float(v[2]))
    raise SensingError(f"expected three numbers ({', '.join(keys)}), got {v!r}")


@dataclass
class SensorMount:
    """
    Where and what to mount (FRammsSensorMount). Every sensor traces / looks along its local +X.

    location is cm relative to the parent component (and socket); rotation is (pitch, yaw, roll) degrees.
    parent "" attaches to the actor's root; on a MuJoCo pawn name the body component (e.g. "chassis").
    sonar_cone_deg is the beam's HALF-angle (15 = a 30 deg beam).
    A camera streams over RMSS on channel `channel` (-1 = the next free one: the sim keeps clear of every stream
    source in the world, each of which takes C, C+100, C+200 and C+300) and +100 / +200 / +300 for depth, motion
    and mask; its stream ids are camera/<name>/color, camera/<name>/depth, motion/<name>, mask/<name>, and its
    stream source component is <name>_stream. Stream ids carry only the name: two robots' "front_camera" share
    them, so a camera is picked by actor (and channel), never by name alone. The sim clamps width / height to
    1..4096 (settings report what it used). Names: 1..64 of A-Z a-z 0-9 _ - . (check_sensor_name).
    """
    name: str
    kind: str = "ToF"
    parent: str = ""
    socket: str = ""
    location: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # ToF
    rows: int = 8
    columns: int = 8
    h_fov_deg: float = 45.0
    v_fov_deg: float = 45.0
    max_range_cm: float = 400.0
    # Sonar
    sonar_max_range_cm: float = 500.0
    sonar_cone_deg: float = 15.0            # half-angle of the beam (BeamHalfAngle): 15 = 30 deg in all
    # Camera
    width: int = 640
    height: int = 480
    fov_deg: float = 87.0
    color: bool = True
    depth: bool = True
    motion: bool = True
    segmentation: bool = True
    channel: int = -1

    def __post_init__(self) -> None:
        if not str(self.name).strip():
            raise SensingError("a sensor mount needs a name")
        self.name = check_sensor_name(self.name)
        self.kind = canonical_kind(self.kind)
        self.location = _triple(self.location, ("x", "y", "z"))
        self.rotation = _triple(self.rotation, ("pitch", "yaw", "roll"))

    @classmethod
    def from_dict(cls, d: dict) -> "SensorMount":
        """From the spec's snake_case mount JSON (location / rotation as lists or {x,y,z} / {pitch,yaw,roll}).
        An unknown key raises SensingError, so a typo does not silently take a default."""
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(d) - known)
        if unknown:
            raise SensingError(f"unknown mount keys: {', '.join(unknown)} (known: {', '.join(sorted(known))})")
        return cls(**d)

    def to_dict(self, all_keys: bool = False) -> dict:
        """The mount JSON for AddSensorJson: the common keys plus the kind's own (all_keys=True: every key)."""
        keys = [f.name for f in fields(self)] if all_keys else list(_COMMON_KEYS) + list(_KIND_KEYS[self.kind])
        out = {}
        for k in keys:
            v = getattr(self, k)
            out[k] = list(v) if isinstance(v, tuple) else v
        return out

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    def stream_ids(self) -> dict[str, str]:
        """For a camera: the stream id of each enabled role."""
        if self.kind != "Camera":
            return {}
        from ramms_tools.streaming.frames import camera_stream_ids
        on = {"color": self.color, "depth": self.depth, "motion": self.motion, "mask": self.segmentation}
        return {r: sid for r, sid in camera_stream_ids(self.name).items() if on[r]}


def as_mount(m: Union[SensorMount, dict]) -> SensorMount:
    return m if isinstance(m, SensorMount) else SensorMount.from_dict(dict(m))


# ── what the bridge reports ───────────────────────────────────────────


@dataclass
class StreamInfo:
    """One RMSS stream of a camera."""
    role: str                        # color, depth, motion, mask, rgbd (a combined frame), imu
    stream_id: str
    channel: int
    message: str = ""
    fmt: str = ""
    camera: str = ""                 # "" for an IMU stream
    actor: str = ""                  # the actor's path
    target: str = ""                 # an IMU stream's sensor or body
    port: Optional[int] = None       # the RMSS server's, when the answer says (start_imu_stream)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict, camera: str = "", actor: str = "") -> "StreamInfo":
        port = int(as_float(d.get("port"), 0)) or None
        return cls(str(d.get("role", "")), str(d.get("stream_id", "")), int(as_float(d.get("channel"), -1)),
                   str(d.get("message", "")), str(d.get("fmt", "")), str(d.get("camera", camera) or camera),
                   str(d.get("actor", actor) or actor), str(d.get("target", "") or ""), port, raw=d)


@dataclass
class SensorInfo:
    """A sensor on an actor (ListSensorsJson entry). location / rotation are world-space (cm, degrees)."""
    name: str
    kind: str
    actor: str = ""
    actor_name: str = ""
    component_path: str = ""
    parent: str = ""
    socket: str = ""
    rig: bool = True                 # made by a sensor rig (False: a component placed by hand)
    location: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)   # pitch, yaw, roll
    settings: dict = field(default_factory=dict)
    streams: list[StreamInfo] = field(default_factory=list)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "SensorInfo":
        name = str(d.get("name", ""))
        return cls(name=name, kind=str(d.get("kind", "")), actor=str(d.get("actor", "")),
                   actor_name=str(d.get("actor_name", "") or short_name(d.get("actor", ""))),
                   component_path=str(d.get("component_path", "")), parent=_fname(d.get("parent", "")),
                   socket=_fname(d.get("socket", "")), rig=as_bool(d.get("rig", True)),
                   location=vec3(d.get("location")), rotation=rotator(d.get("rotation")),
                   settings=dict(d.get("settings") or {}),
                   streams=[StreamInfo.from_json(s, camera=name, actor=str(d.get("actor", "") or ""))
                            for s in (d.get("streams") or [])], raw=d)

    @property
    def channel(self) -> Optional[int]:
        """A camera's base RMSS channel actually in use (auto-assigned when -1 was asked); None for other kinds, and
        for a camera that streams on no channel (one placed by hand that no stream source forwards: the sim reports
        -1)."""
        if self.kind != "Camera":
            return None
        if "channel" in self.settings and self.settings["channel"] is not None:
            ch = int(as_float(self.settings["channel"], -1))
            return ch if ch >= 0 else None
        for st in self.streams:
            if st.channel >= 0 and st.role in _ROLE_OFFSETS:
                return st.channel - _ROLE_OFFSETS[st.role]
        return None

    def summary(self) -> str:
        s = self.settings
        if self.kind == "ToF":
            what = f"{s.get('rows', '?')}x{s.get('columns', '?')} {s.get('h_fov_deg', '?')}x{s.get('v_fov_deg', '?')} deg {s.get('max_range_cm', '?')} cm"
        elif self.kind == "Sonar":
            rng = s.get("max_range_cm", s.get("sonar_max_range_cm", "?"))
            cone = s.get("cone_deg", s.get("sonar_cone_deg", "?"))
            what = f"{rng} cm, cone half-angle {cone} deg"
        elif self.kind == "Camera":
            what = f"{s.get('width', '?')}x{s.get('height', '?')} fov {s.get('fov_deg', '?')} ch {s.get('channel', '?')} " + \
                   ",".join(st.role for st in self.streams)
        else:
            what = ""
        return what.strip()


_ACCEL_TO_MS2 = {"cm/s^2": 0.01, "cm/s2": 0.01, "cm/s/s": 0.01, "m/s^2": 1.0, "m/s2": 1.0, "m/s/s": 1.0, "g": 9.80665}
_GYRO_TO_DEGS = {"deg/s": 1.0, "degrees/s": 1.0, "rad/s": 57.29577951308232}


@dataclass
class SensorReading:
    """
    One sensor's latest sample (ReadSensorJson / an entry of ReadSensorsJson's readings).

    IMU values are in the component's own units, unconverted (`units` says which: the sim reports
    "cm/s^2" and "deg/s"), acceleration and angular velocity in the sensor's frame, orientation in the world;
    linear_acceleration_ms2() / angular_velocity_degs() convert.
    """
    name: str
    kind: str
    ok: bool = True
    error: str = ""
    timestamp: float = 0.0
    actor: str = ""
    has_sample: Optional[bool] = None       # False: nothing measured yet (the values are the component's defaults)
    # ToF
    rows: int = 0
    columns: int = 0
    distances_cm: list[float] = field(default_factory=list)     # row-major, -1 = no hit
    min_distance_cm: Optional[float] = None
    any_hit: bool = False
    # Sonar
    distance_cm: Optional[float] = None
    hit: bool = False
    hit_location: Optional[tuple[float, float, float]] = None
    hit_normal: Optional[tuple[float, float, float]] = None
    hit_actor: str = ""
    max_range_cm: Optional[float] = None    # sims that report it with the reading
    # IMU
    linear_acceleration: Optional[tuple[float, float, float]] = None
    angular_velocity: Optional[tuple[float, float, float]] = None
    orientation: Optional[tuple[float, float, float, float]] = None     # x, y, z, w
    euler: Optional[tuple[float, float, float]] = None                  # roll, pitch, yaw (deg)
    units: dict = field(default_factory=dict)
    includes_gravity: Optional[bool] = None                             # IMU: the acceleration includes gravity
    # IMU: where the numbers come from -- "mujoco" / "chaos" (the physics engine, sampled every physics step at the
    # sensor's point) or "kinematic" (the component's transform differenced at the game rate); "" from older sims.
    source: str = ""
    sim_time: Optional[float] = None    # the engine's time of the sample (s); None when absent, null or -1 (no sample)
    has_acceleration: Optional[bool] = None                             # False: a body IMU's first reads
    linear_velocity: Optional[tuple[float, float, float]] = None        # body IMU: world cm/s at the point
    angular_acceleration: Optional[tuple[float, float, float]] = None   # body IMU: sensor frame, deg/s^2
    body: str = ""                                                      # body IMU: the body it reads
    bone: str = ""
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "SensorReading":
        d = d or {}
        r = cls(name=str(d.get("name", "")), kind=str(d.get("kind", "")), ok=as_bool(d.get("ok", True)),
                error=str(d.get("error", "") or ""), timestamp=as_float(d.get("timestamp")),
                actor=str(d.get("actor", "") or ""),
                has_sample=as_bool(d["has_sample"]) if "has_sample" in d else None, raw=d)
        if "distances_cm" in d or "rows" in d:
            r.rows, r.columns = int(as_float(d.get("rows"))), int(as_float(d.get("columns")))
            r.distances_cm = [as_float(v, -1.0) for v in (d.get("distances_cm") or [])]
            r.min_distance_cm = as_float(d["min_distance_cm"]) if d.get("min_distance_cm") is not None else None
            r.any_hit = as_bool(d.get("any_hit", False))
        if "distance_cm" in d:
            r.distance_cm = as_float(d.get("distance_cm"))
            r.max_range_cm = as_float(d["max_range_cm"]) if d.get("max_range_cm") is not None else None
            r.hit = as_bool(d.get("hit", False))
            r.hit_location = vec3(d["hit_location"]) if d.get("hit_location") else None
            r.hit_normal = vec3(d["hit_normal"]) if d.get("hit_normal") else None
            r.hit_actor = str(d.get("hit_actor", "") or "")
        if "linear_acceleration" in d or "angular_velocity" in d:
            r.linear_acceleration = vec3(d.get("linear_acceleration"))
            r.angular_velocity = vec3(d.get("angular_velocity"))
            q = d.get("orientation") or {}
            r.orientation = tuple(as_float(q.get(k, 0.0 if k != "w" else 1.0)) for k in "xyzw") if isinstance(q, dict) else None  # type: ignore[assignment]
            e = d.get("euler") or {}
            r.euler = (as_float(e.get("roll")), as_float(e.get("pitch")), as_float(e.get("yaw"))) if isinstance(e, dict) else None
            r.units = dict(d.get("units") or {})
            r.includes_gravity = as_bool(d["includes_gravity"]) if "includes_gravity" in d else None
            r.source = str(d.get("source", "") or "")
            r.sim_time = _sim_time(d)
            r.has_acceleration = as_bool(d["has_acceleration"]) if "has_acceleration" in d else None
            r.linear_velocity = vec3(d["linear_velocity"]) if d.get("linear_velocity") else None
            r.angular_acceleration = vec3(d["angular_acceleration"]) if d.get("angular_acceleration") else None
            r.body = str(d.get("body", "") or "")
            r.bone = str(d.get("bone", "") or "")
        return r

    def linear_acceleration_ms2(self) -> Optional[tuple[float, float, float]]:
        """An IMU's linear acceleration in m/s^2 (converted from `units`; the sim reports cm/s^2)."""
        if self.linear_acceleration is None:
            return None
        unit = str(self.units.get("linear_acceleration", "cm/s^2")).replace(" ", "").lower()
        k = _ACCEL_TO_MS2.get(unit)
        if k is None:
            raise SensingError(f"{self.name}: cannot convert acceleration in {unit!r} to m/s^2")
        return tuple(v * k for v in self.linear_acceleration)  # type: ignore[return-value]

    def angular_velocity_degs(self) -> Optional[tuple[float, float, float]]:
        """An IMU's angular velocity in deg/s (converted from `units`)."""
        if self.angular_velocity is None:
            return None
        unit = str(self.units.get("angular_velocity", "deg/s")).replace(" ", "").lower()
        k = _GYRO_TO_DEGS.get(unit)
        if k is None:
            raise SensingError(f"{self.name}: cannot convert angular velocity in {unit!r} to deg/s")
        return tuple(v * k for v in self.angular_velocity)  # type: ignore[return-value]

    def grid(self):
        """A ToF reading as a (rows, columns) float32 numpy array, NaN where nothing was hit."""
        import numpy as np
        a = np.asarray(self.distances_cm, dtype=np.float32)
        if self.rows and self.columns and a.size == self.rows * self.columns:
            a = a.reshape(self.rows, self.columns)
        a = a.copy()
        a[a < 0] = np.nan
        return a

    @property
    def value(self) -> Optional[float]:
        """The headline number: a ToF's nearest hit, a sonar's distance (None without a hit)."""
        if self.kind == "ToF":
            return self.min_distance_cm if self.any_hit else None
        if self.kind == "Sonar":
            return self.distance_cm if self.hit else None
        return None

    def flat(self) -> dict:
        """One flat row (for CSV / table output)."""
        row: dict[str, Any] = {"name": self.name, "kind": self.kind, "timestamp": self.timestamp, "ok": self.ok}
        if not self.ok:
            row["error"] = self.error
        if self.kind == "ToF":
            row.update(min_distance_cm=self.min_distance_cm, any_hit=self.any_hit, rows=self.rows, columns=self.columns)
        elif self.kind == "Sonar":
            row.update(distance_cm=self.distance_cm, hit=self.hit, hit_actor=self.hit_actor)
        elif self.kind == "IMU":
            ax, ay, az = self.linear_acceleration or (math.nan,) * 3
            gx, gy, gz = self.angular_velocity or (math.nan,) * 3
            roll, pitch, yaw = self.euler or (math.nan,) * 3
            row.update(ax=ax, ay=ay, az=az, accel_units=self.units.get("linear_acceleration", ""), gx=gx, gy=gy, gz=gz,
                       gyro_units=self.units.get("angular_velocity", ""), roll=roll, pitch=pitch, yaw=yaw,
                       source=self.source, sim_time=self.sim_time)
        return row

    def summary(self) -> str:
        if not self.ok:
            return f"error: {self.error}"
        if self.has_sample is False:
            return "no sample yet"
        if self.kind == "ToF":
            near = f"{self.min_distance_cm:.1f} cm" if self.any_hit and self.min_distance_cm is not None else "no hit"
            hits = sum(1 for v in self.distances_cm if v >= 0)
            return f"nearest {near} ({hits}/{len(self.distances_cm)} zones hit)"
        if self.kind == "Sonar":
            return (f"{self.distance_cm:.1f} cm" + (f" ({self.hit_actor})" if self.hit_actor else "")) if self.hit else "no echo"
        if self.kind == "IMU":
            a = self.linear_acceleration or (0, 0, 0)
            g = self.angular_velocity or (0, 0, 0)
            e = self.euler or (0, 0, 0)
            ua = self.units.get("linear_acceleration", "")
            ug = self.units.get("angular_velocity", "")
            si = ""
            if ua and ua.replace(" ", "").lower() not in ("m/s^2", "m/s2"):
                try:
                    m = self.linear_acceleration_ms2()
                    si = f" = ({m[0]:.2f}, {m[1]:.2f}, {m[2]:.2f}) m/s^2" if m else ""
                except SensingError:
                    si = ""
            acc = "acc (no acceleration yet)" if self.has_acceleration is False else \
                f"acc ({a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f}) {ua}{si}"
            src = f"  [{self.source}]" if self.source else ""
            return (f"{acc}  gyro ({g[0]:.2f}, {g[1]:.2f}, {g[2]:.2f}) {ug}  "
                    f"rpy ({e[0]:.1f}, {e[1]:.1f}, {e[2]:.1f}){src}").strip()
        return ""


@dataclass
class StreamStatus:
    """The RMSS server and camera capture (GetStreamStatusJson / StartCameraStreamsJson)."""
    server: dict = field(default_factory=dict)
    capture: dict = field(default_factory=dict)
    streams: list[StreamInfo] = field(default_factory=list)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "StreamStatus":
        d = d or {}
        return cls(dict(d.get("server") or {}), dict(d.get("capture") or {}),
                   [StreamInfo.from_json(s) for s in (d.get("streams") or [])], raw=d)

    @property
    def running(self) -> bool:
        return as_bool(self.server.get("running", False))

    @property
    def port(self) -> Optional[int]:
        """The RMSS server's port, or None when it is not running (the sim then reports "running": false and port 0,
        or no server at all)."""
        port = int(as_float(self.server.get("port"), 0))
        return port if self.running and port > 0 else None

    @property
    def capturing(self) -> bool:
        return as_bool(self.capture.get("capturing", False))

    def cameras(self, actor: Optional[str] = None) -> list[str]:
        return sorted({s.camera for s in self.streams if s.camera and (actor is None or same_actor(s.actor, actor))})

    def camera_actors(self, camera: str) -> list[str]:
        """The actors with a camera of this name streaming (stream ids carry only the name)."""
        return sorted({s.actor for s in self.streams if s.camera == camera and s.role != "imu"})

    def camera_streams(self, camera: str, actor: Optional[str] = None) -> list[StreamInfo]:
        """A camera's streams; with `actor` (a path or name), only that actor's."""
        return [s for s in self.streams if s.camera == camera and s.role != "imu"
                and (actor is None or same_actor(s.actor, actor))]

    def imu_streams(self, actor: Optional[str] = None) -> list[StreamInfo]:
        return [s for s in self.streams if s.role == "imu" and (actor is None or same_actor(s.actor, actor))]


@dataclass
class SegmentationClass:
    value: int
    name: str
    coarse: str = ""
    coarse_id: int = 0


@dataclass
class SegmentationTable:
    """The fine segmentation classes (mask values) and their coarse groups (GetSegmentationClassesJson; needs a
    play world)."""
    classes: list[SegmentationClass] = field(default_factory=list)
    coarse: list[dict] = field(default_factory=list)
    reserved: Any = None                     # the values the sim keeps for itself, as it reports them
    num_segment_ids: int = 0                 # highest class value + 1 (what mask frames carry as NumSegmentIDs)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "SegmentationTable":
        d = d or {}
        classes = [SegmentationClass(int(as_float(c.get("value"))), str(c.get("name", "")), str(c.get("coarse", "")),
                                     int(as_float(c.get("coarse_id")))) for c in (d.get("classes") or [])]
        n = int(as_float(d.get("num_segment_ids"), 0)) or (max((c.value for c in classes), default=-1) + 1)
        return cls(classes, [dict(c) for c in (d.get("coarse") or [])], d.get("reserved"), n, raw=d)

    def names(self) -> dict[int, str]:
        return {c.value: c.name for c in self.classes}


# ── the bridge ────────────────────────────────────────────────────────


_STREAM_OPTIONS = ("port", "every_n_frames", "color", "depth", "motion", "segmentation", "save_to_disk")


class Sensing:
    """
    The sensing bridge (URammsSensingBridge, a static function library allow-listed for Remote Control).
    Actors are addressed by full object path; "" means every actor where the bridge allows it.

    Args:
        ue: an existing UnrealRemote, or None to connect to host:port.
    """

    def __init__(self, ue: Optional[UnrealRemote] = None, host: str = "127.0.0.1", port: int = 30010,
                 timeout: float = 10.0):
        self.ue = ue or UnrealRemote(host=host, http_port=port, timeout=timeout)

    def __repr__(self) -> str:
        return f"Sensing({self.ue!r})"

    def _call(self, func: str, **params: Any) -> Any:
        return return_value(self.ue._call_function(BRIDGE, func, params if params else None))

    def _json(self, func: str, allow_fail: bool = False, **params: Any) -> dict:
        res = as_json(self._call(func, **params)) or {}
        if not isinstance(res, dict):
            raise SensingError(f"{func}: unexpected answer {res!r}")
        if not allow_fail and not as_bool(res.get("ok", False)):
            raise SensingError(f"{func}: {res.get('error', 'failed')}")
        return res

    def for_actor(self, actor_path: str) -> "ActorSensors":
        return ActorSensors(self, actor_path)

    # ── sensors ─────────────────────────────────────────────────────

    def list(self, actor: str = "") -> list[SensorInfo]:
        """Sensors on an actor ("" = every actor): rig-made ones and components placed by hand."""
        return [SensorInfo.from_json(s) for s in (self._json("ListSensorsJson", ActorPath=actor).get("sensors") or [])]

    def add(self, actor: str, mount: Union[SensorMount, dict]) -> dict:
        """Add (or replace, by name) a sensor, through the actor's sensor rig (one named from "SensorRig" is
        created when it has none). Returns {"ok", "name", "kind", "component_path", "rig_path", "built"}, plus
        "channel" (in use) and "streams" for a camera. A parent component the actor does not have is an error.
        "built" false (older sims, in the editor world): the actor has not begun play, so the mount is kept and
        built at BeginPlay; sims from the fix round refuse sensing changes outside a play world instead."""
        m = as_mount(mount)
        return self._json("AddSensorJson", ActorPath=actor, MountJson=m.to_json())

    def remove(self, actor: str, name: str) -> bool:
        """Remove a rig sensor (False for an unknown name, or a sensor placed by hand: those stay)."""
        return as_bool(self._call("RemoveSensor", ActorPath=actor, Name=name))

    def read(self, actor: str, name: str, measure: bool = False) -> SensorReading:
        """A sensor's latest sample (measure=True: trace now). Raises SensingError for a camera or an unknown name."""
        res = self._json("ReadSensorJson", allow_fail=True, ActorPath=actor, Name=name, bMeasureNow=bool(measure))
        if not as_bool(res.get("ok", False)):
            raise SensingError(f"{name}: {res.get('error', 'read failed')}")
        return SensorReading.from_json(res)

    def read_all(self, actor: str) -> list[SensorReading]:
        """Every ToF, sonar and IMU on the actor, latest samples."""
        return [SensorReading.from_json(r) for r in (self._json("ReadSensorsJson", ActorPath=actor).get("readings") or [])]

    # ── IMU from the physics engine, no component needed ────────────

    def body_imu(self, actor: str, body: str = "", offset: Optional[tuple[float, float, float]] = None,
                 rotation: Optional[tuple[float, float, float]] = None, source: str = "auto",
                 wait: float = 0.0) -> SensorReading:
        """
        An IMU reading for any body of an actor, straight from the physics engine that simulates it, with no IMU
        component (ReadBodyImuJson). body: a MuJoCo body name, a component name or a bone ("" = the actor's main
        simulated body); offset: the virtual sensor's position in the body frame (cm); rotation: its (pitch, yaw,
        roll) in degrees; source: auto | mujoco | chaos | kinematic. MuJoCo and Chaos are sampled every physics
        step at the point (lever arm included; acceleration as an accelerometer reads it, +g up at rest).
        The first read starts tracking the point, so it carries no sample yet (has_sample / has_acceleration
        False): wait > 0 re-reads for up to that many seconds until it has one. A point not read for ~10 s is
        dropped.
        """
        opts: dict[str, Any] = {"source": source}
        if offset is not None:
            opts["offset"] = [float(v) for v in offset]
        if rotation is not None:
            opts["rotation"] = [float(v) for v in rotation]
        deadline = time.monotonic() + max(0.0, wait)
        while True:
            res = self._json("ReadBodyImuJson", allow_fail=True, ActorPath=actor, Body=body, OptionsJson=json.dumps(opts))
            if not as_bool(res.get("ok", False)):
                raise SensingError(f"{body or 'body'}: {res.get('error', 'read failed')}")
            ready = as_bool(res.get("has_sample", True)) and as_bool(res.get("has_acceleration", True))
            if ready or time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        res.setdefault("kind", "IMU")
        res.setdefault("name", res.get("body", body) or "body")
        return SensorReading.from_json(res)

    def imu_bodies(self, actor: str) -> list[dict]:
        """The bodies of an actor an IMU can be read from (ListImuBodiesJson): [{name, engine, component,
        component_path, bone, default}]."""
        return list(self._json("ListImuBodiesJson", ActorPath=actor).get("bodies") or [])

    # ── IMU samples at the physics rate (ramms_tools.sensing.imu) ───

    @staticmethod
    def _imu_options(offset, rotation, source) -> dict[str, Any]:
        opts: dict[str, Any] = {}
        if offset is not None:
            opts["offset"] = [float(v) for v in offset]
        if rotation is not None:
            opts["rotation"] = [float(v) for v in rotation]
        if source and source != "auto":
            opts["source"] = source
        return opts

    def read_imu_samples(self, actor: str, target: str = "", since: int = -1, max_samples: int = 1024,
                         offset: Optional[tuple[float, float, float]] = None,
                         rotation: Optional[tuple[float, float, float]] = None, source: str = "auto"):
        """
        The physics-step samples of an IMU target since a cursor (ReadImuSamplesJson): an ImuSamples with the samples
        oldest first, `next` (pass it back as `since`), `dropped` (lost before the oldest kept) and `rate_hz`.
        since=-1 starts now (no samples, only the cursor). target: an IMU sensor's name, else a body ("" = the main
        one); offset / rotation / source as body_imu, for body targets.
        """
        from .imu import ImuSamples
        opts = self._imu_options(offset, rotation, source)
        opts["since"] = int(since)
        opts["max"] = int(max_samples)
        res = self._json("ReadImuSamplesJson", allow_fail=True, ActorPath=actor, Target=target, OptionsJson=json.dumps(opts))
        if not as_bool(res.get("ok", False)):
            raise SensingError(f"{target or 'body'}: {res.get('error', 'ReadImuSamplesJson failed')}")
        return ImuSamples.from_json(res)

    def follow_imu_samples(self, actor: str, target: str = "", period: float = 0.1, duration: Optional[float] = None,
                           should_stop=None, offset: Optional[tuple[float, float, float]] = None,
                           rotation: Optional[tuple[float, float, float]] = None, source: str = "auto"):
        """Every physics-step sample of an IMU target from now on, polled every `period` s with the cursor (an
        ImuSampleFollower: iterate it; its `dropped` counts samples lost to a late poll). Stops after `duration` s or
        when should_stop() is true. The sim keeps ~2 s of samples: a period well under that loses none."""
        from .imu import ImuSampleFollower

        def read(since: int):
            return self.read_imu_samples(actor, target, since=since, offset=offset, rotation=rotation, source=source)
        return ImuSampleFollower(read, period=period, duration=duration, should_stop=should_stop)

    def start_imu_stream(self, actor: str, target: str = "", channel: int = -1, port: Optional[int] = None,
                         offset: Optional[tuple[float, float, float]] = None,
                         rotation: Optional[tuple[float, float, float]] = None, source: str = "auto") -> StreamInfo:
        """
        Stream an IMU target's physics-step samples over RMSS (StartImuStreamJson; needs a play world; starts the RMSS
        server if it is not running): one FrameData (fmt imu_f64x18) per game tick with new samples, on `channel`
        (-1 = the next free one >= 1000). Returns its StreamInfo (channel, stream_id "imu/<actor name>/<target>",
        port when the sim says); read it with ImuStream.from_info. Stop it with stop_imu_stream.
        """
        opts = self._imu_options(offset, rotation, source)
        opts["channel"] = int(channel)
        if port is not None:
            opts["port"] = int(port)
        res = self._json("StartImuStreamJson", ActorPath=actor, Target=target, OptionsJson=json.dumps(opts))
        port_now = int(as_float((res.get("server") or {}).get("port"), 0)) or res.get("port") or port
        entry = res.get("stream") if isinstance(res.get("stream"), dict) else None
        if entry is None and ("channel" in res or "stream_id" in res):
            entry = res
        if entry is None:
            entry = next((s for s in (res.get("streams") or []) if s.get("role") == "imu"
                          and str(s.get("target", "")) == target and same_actor(str(s.get("actor", "")), actor)), None)
        if entry is None:
            raise SensingError(f"StartImuStreamJson: no IMU stream in the answer {res!r}")
        info = StreamInfo.from_json(dict(entry, role=entry.get("role", "imu")), actor=str(res.get("actor", "") or actor))
        info.target = info.target or str(res.get("target", "") or target)
        info.port = info.port or (int(port_now) if port_now else None)
        return info

    def stop_imu_stream(self, actor: str, target: str = "") -> bool:
        """Stop an IMU stream (False when there was none)."""
        return as_bool(self._call("StopImuStream", ActorPath=actor, Target=target))

    # ── camera streams ──────────────────────────────────────────────

    def start_streams(self, **options: Any) -> StreamStatus:
        """
        Start the RMSS server if needed and the cameras' capture and streams (needs a play world: PIE or a
        game). Options (all optional): port, every_n_frames, color, depth, motion, segmentation, save_to_disk
        (default false). Options left None are not sent (the bridge's defaults apply).
        """
        unknown = sorted(set(options) - set(_STREAM_OPTIONS))
        if unknown:
            raise SensingError(f"unknown stream options: {', '.join(unknown)} (known: {', '.join(_STREAM_OPTIONS)})")
        opts = {k: v for k, v in options.items() if v is not None}
        return StreamStatus.from_json(self._json("StartCameraStreamsJson", OptionsJson=json.dumps(opts)))

    def stop_streams(self) -> bool:
        """Stop the capture (the RMSS server stays up)."""
        return as_bool(self._call("StopCameraStreams"))

    def status(self) -> StreamStatus:
        return StreamStatus.from_json(self._json("GetStreamStatusJson"))

    def segmentation_classes(self) -> SegmentationTable:
        return SegmentationTable.from_json(self._json("GetSegmentationClassesJson"))

    # ── rigs ────────────────────────────────────────────────────────

    def apply_rig(self, actor: str, mounts: Iterable[Union[SensorMount, dict]], replace: bool = False) -> list[dict]:
        """Add every mount to the actor (each replaces a sensor of its name). With replace=True, rig sensors not in
        `mounts` are removed first. Returns the AddSensorJson results."""
        ms = [as_mount(m) for m in mounts]
        if replace:
            keep = {m.name.lower() for m in ms}
            for s in self.list(actor):           # this actor's sensors (the actor may be given by name)
                if s.rig and s.name.lower() not in keep:
                    self.remove(actor, s.name)
        return [self.add(actor, m) for m in ms]


class ActorSensors:
    """The sensing bridge bound to one actor (Robot.sensors)."""

    def __init__(self, sensing: Sensing, actor_path: str):
        self.sensing = sensing
        self.actor_path = actor_path

    def __repr__(self) -> str:
        return f"ActorSensors({self.actor_path!r})"

    def list(self) -> list[SensorInfo]:
        return self.sensing.list(self.actor_path)

    def add(self, mount: Union[SensorMount, dict]) -> dict:
        return self.sensing.add(self.actor_path, mount)

    def remove(self, name: str) -> bool:
        return self.sensing.remove(self.actor_path, name)

    def read(self, name: str, measure: bool = False) -> SensorReading:
        return self.sensing.read(self.actor_path, name, measure)

    def read_all(self) -> list[SensorReading]:
        return self.sensing.read_all(self.actor_path)

    def body_imu(self, body: str = "", offset: Optional[tuple[float, float, float]] = None,
                 rotation: Optional[tuple[float, float, float]] = None, source: str = "auto",
                 wait: float = 0.0) -> SensorReading:
        """See Sensing.body_imu: an IMU on any body of this robot, from its physics engine."""
        return self.sensing.body_imu(self.actor_path, body, offset, rotation, source, wait=wait)

    def imu_bodies(self) -> list[dict]:
        return self.sensing.imu_bodies(self.actor_path)

    def read_imu_samples(self, target: str = "", since: int = -1, **kw):
        """See Sensing.read_imu_samples."""
        return self.sensing.read_imu_samples(self.actor_path, target, since=since, **kw)

    def follow_imu_samples(self, target: str = "", **kw):
        """See Sensing.follow_imu_samples."""
        return self.sensing.follow_imu_samples(self.actor_path, target, **kw)

    def start_imu_stream(self, target: str = "", **kw) -> StreamInfo:
        """See Sensing.start_imu_stream."""
        return self.sensing.start_imu_stream(self.actor_path, target, **kw)

    def stop_imu_stream(self, target: str = "") -> bool:
        return self.sensing.stop_imu_stream(self.actor_path, target)

    def apply_rig(self, preset: Union[str, Iterable[Union[SensorMount, dict]]], replace: bool = False) -> list[dict]:
        """Apply a rig preset by name ("rammp_gen1", "chair"), a JSON file of mounts, or a list of mounts."""
        from .rigs import rig_mounts
        mounts = rig_mounts(preset) if isinstance(preset, str) else list(preset)
        return self.sensing.apply_rig(self.actor_path, mounts, replace=replace)
