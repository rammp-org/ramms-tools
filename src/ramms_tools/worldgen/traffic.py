"""
Moving traffic on generated streets (RammsCrowd's Mass traffic) over Remote Control.

A traffic level has an ``ARammsTrafficSpawner`` (spawns the cars when play
begins) and an ``ARammsTrafficSignalClock`` (one signal phase for every
signal: green / amber / red per axis, walk / flashing don't walk); each car
is an ``ARammsTrafficVehicleActor``. Both actor classes must be allow-listed
for Remote Control (Config/DefaultRemoteControl.ini), and the car poses use the
allow-listed ``K2_GetActorLocation`` / ``K2_GetActorRotation``.

    from ramms_tools.worldgen import Traffic

    t = Traffic()
    print(t.count(), t.signal_state())        # 24, SignalState(light_x='Green', light_y='Red', ...)
    t.despawn(); t.spawn()
    for car in t.cars(limit=5):
        print(car.name, car.location, car.yaw)

Cars only exist while a play world runs (PIE or a game).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

from ramms_tools._rc import as_bool, as_float, enum_name, return_value, rotator, short_name, vec3
from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError

SPAWNER_CLASS = "RammsTrafficSpawner"
CLOCK_CLASS = "RammsTrafficSignalClock"
VEHICLE_CLASS = "RammsTrafficVehicleActor"


class TrafficError(UnrealRemoteError):
    pass


@dataclass
class SignalState:
    """FRammsSignalState: the lights for traffic along world X / Y and the walk signals (pedestrians walking along X /
    Y, i.e. crossing the roads that run along Y / X)."""
    light_x: str = ""               # Red, Amber, Green
    light_y: str = ""
    walk_along_x: str = ""          # DontWalk, FlashingDontWalk, Walk
    walk_along_y: str = ""
    seconds_to_change_x: float = 0.0
    seconds_to_change_y: float = 0.0
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: Optional[dict]) -> "SignalState":
        d = d or {}
        return cls(enum_name(d.get("LightX")), enum_name(d.get("LightY")), enum_name(d.get("WalkAlongX")),
                   enum_name(d.get("WalkAlongY")), as_float(d.get("SecondsToChangeX")), as_float(d.get("SecondsToChangeY")),
                   raw=d)

    def __str__(self) -> str:
        return (f"X {self.light_x} ({self.seconds_to_change_x:.1f} s)  Y {self.light_y} ({self.seconds_to_change_y:.1f} s)  "
                f"walk along X {self.walk_along_x}, along Y {self.walk_along_y}")

    def as_dict(self) -> dict:
        return {"light_x": self.light_x, "light_y": self.light_y, "walk_along_x": self.walk_along_x,
                "walk_along_y": self.walk_along_y, "seconds_to_change_x": self.seconds_to_change_x,
                "seconds_to_change_y": self.seconds_to_change_y}


@dataclass
class CarPose:
    path: str
    location: tuple[float, float, float]          # cm, world
    rotation: tuple[float, float, float]          # pitch, yaw, roll (deg)

    @property
    def name(self) -> str:
        return short_name(self.path)

    @property
    def yaw(self) -> float:
        return self.rotation[1]


class Traffic:
    """The level's traffic spawners, signal clocks and cars."""

    def __init__(self, ue: Optional[UnrealRemote] = None, host: str = "127.0.0.1", port: int = 30010,
                 timeout: float = 10.0):
        self.ue = ue or UnrealRemote(host=host, http_port=port, timeout=timeout)

    def __repr__(self) -> str:
        return f"Traffic({self.ue!r})"

    def _find(self, cls: str) -> list[str]:
        return [a.object_path for a in self.ue.find_actors(class_filter=cls)]

    def _call(self, path: str, func: str, **params: Any) -> Any:
        return return_value(self.ue._call_function(path, func, params if params else None))

    def _pick(self, paths: list[str], which: str, what: str) -> list[str]:
        if not which:
            return paths
        hit = [p for p in paths if which == p or which.lower() in short_name(p).lower()]
        if not hit:
            raise TrafficError(f"no {what} matching {which!r} ({', '.join(short_name(p) for p in paths) or 'none'})")
        return hit

    # ── discovery ───────────────────────────────────────────────────

    def spawners(self) -> list[str]:
        """Actor paths of the traffic spawners in the world."""
        return self._find(SPAWNER_CLASS)

    def clocks(self) -> list[str]:
        """Actor paths of the signal clocks in the world."""
        return self._find(CLOCK_CLASS)

    # ── cars ────────────────────────────────────────────────────────

    def spawn(self, spawner: str = "") -> dict[str, bool]:
        """Spawn the cars now (replacing any a spawner spawned before), on every spawner or the one named.
        False for a spawner with no vehicle lanes yet, or busy (it spawns on its own next tick)."""
        paths = self._pick(self.spawners(), spawner, "traffic spawner")
        if not paths:
            raise TrafficError("no traffic spawner in the world (a traffic level, in PIE or a game)")
        return {p: as_bool(self._call(p, "SpawnTraffic")) for p in paths}

    def despawn(self, spawner: str = "") -> int:
        """Remove the cars; returns how many spawners were told to."""
        paths = self._pick(self.spawners(), spawner, "traffic spawner")
        for p in paths:
            self._call(p, "DespawnTraffic")
        return len(paths)

    def counts(self) -> dict[str, int]:
        return {p: int(as_float(self._call(p, "GetSpawnedCount"))) for p in self.spawners()}

    def count(self) -> int:
        """Cars spawned, over every spawner."""
        return sum(self.counts().values())

    def cars(self, limit: int = 0) -> list[CarPose]:
        """Where the cars are (K2_GetActorLocation / K2_GetActorRotation on each vehicle actor); limit > 0 reads only
        the first `limit`."""
        out: list[CarPose] = []
        for p in self._find(VEHICLE_CLASS):
            if 0 < limit <= len(out):
                break
            out.append(CarPose(p, vec3(self._call(p, "K2_GetActorLocation")), rotator(self._call(p, "K2_GetActorRotation"))))
        return out

    # ── signals ─────────────────────────────────────────────────────

    def signal_state(self, clock: str = "") -> Optional[SignalState]:
        """The signals now (the first clock, or the one named); None when the level has no signal clock."""
        paths = self._pick(self.clocks(), clock, "signal clock")
        if not paths:
            return None
        return SignalState.from_json(self._call(paths[0], "GetSignalState"))

    def controlled_crossings(self, clock: str = "") -> int:
        """Crowd crossing lanes the clock opens and closes."""
        paths = self._pick(self.clocks(), clock, "signal clock")
        return int(as_float(self._call(paths[0], "GetControlledCrossingCount"))) if paths else 0

    def status(self) -> dict:
        spawners, clocks = self.spawners(), self.clocks()
        out: dict[str, Any] = {"spawners": len(spawners), "clocks": len(clocks),
                               "cars": sum(int(as_float(self._call(p, "GetSpawnedCount"))) for p in spawners)}
        if clocks:
            out["signals"] = SignalState.from_json(self._call(clocks[0], "GetSignalState")).as_dict()
            out["controlled_crossings"] = int(as_float(self._call(clocks[0], "GetControlledCrossingCount")))
        return out
