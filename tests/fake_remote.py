"""An offline stand-in for the Remote Control API, answering from recorded responses.

FakeRemote subclasses UnrealRemote and replaces only the HTTP transport
(``_request``), so everything above it -- ``_call_function``, ``call_full``,
``find_actors_by_component``, ``find_components`` -- runs as it does against
the engine. The RAMMP Gen1 robot is built from live captures of the public
curb pawn (tests/fixtures/gen1); the RAMMP Gen2 (lift-drive) robot from
captures of the same shape with neutral names and made-up geometry
(tests/fixtures/gen2: the real robot is private).
"""
from __future__ import annotations

import copy
import json
import time
from pathlib import Path
from typing import Any, Callable

from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError

FIXTURES = Path(__file__).resolve().parent / "fixtures"
CORE_BRIDGE = "/Script/RammsCore.Default__RammsCoreBridge"
LEVEL = "/Game/Maps/Map_Test.Map_Test:PersistentLevel."


def load(gen: str, name: str) -> Any:
    return json.loads((FIXTURES / gen / name).read_text(encoding="utf-8"))


class FakeSurface:
    """
    A control surface with state: values, targets and texts per control id, following
    URammsRobotControlSurfaceComponent:

    - deadman=True (sims from the 2026-11-15 fix round): a Continuous control set by Remote / Autonomy springs back
      to its default when not set again by that source within `hold_seconds`;
    - deadman=False (older sims): it latches -- the value stays until released or set again.

    Position / Velocity / Enum controls hold their target until released either way. `latched()` says which
    Continuous controls are still acting (off their default).
    """

    def __init__(self, surface: dict, deadman: bool = True, hold_seconds: float = 0.3):
        self.surface = surface
        self.deadman = deadman
        self.hold_seconds = hold_seconds
        self.values: dict[str, float] = {a["id"]: float(a["defaultValue"]) for a in surface["axes"]}
        self.targets: dict[str, float] = {}
        self.texts: dict[str, str] = {"curb.status": "Ready"}
        self.triggered: list[str] = []
        self.released: list[str] = []
        self.sets: list[tuple[str, float, str]] = []
        self.set_times: list[tuple[float, str, float]] = []     # (monotonic time, id, value) of every accepted set
        self.release_times: list[tuple[float, str]] = []
        self.events: list[tuple[str, str]] = []                  # ("set" | "release", id), in the order they landed
        self.lapsed: list[str] = []                              # Continuous controls the deadman sprang back
        self.refuse: set[str] = set()                            # ids SetControl answers False for
        self.on_call: Callable[[str, dict], None] | None = None  # runs before every call (delays, phase changes)
        self._last: dict[str, tuple[float, str]] = {}            # id -> (time, source) of its last set

    def axis(self, cid):
        return next((a for a in self.surface["axes"] if a["id"].lower() == cid.lower()), None)

    def _expire(self) -> None:
        if not self.deadman:
            return
        now = time.monotonic()
        for cid, (t, source) in list(self._last.items()):
            a = self.axis(cid)
            if a is not None and a["kind"] == "Continuous" and source in ("Remote", "Autonomy") and now - t > self.hold_seconds:
                self.values[cid] = float(a["defaultValue"])
                self.targets[cid] = float(a["defaultValue"])
                self.lapsed.append(cid)
                del self._last[cid]

    def latched(self) -> dict[str, float]:
        """The Continuous controls off their default now (still acting on the robot)."""
        self._expire()
        return {a["id"]: self.values[a["id"]] for a in self.surface["axes"]
                if a["kind"] == "Continuous" and self.values.get(a["id"], 0.0) != float(a["defaultValue"])}

    def __call__(self, func: str, p: dict) -> dict:
        if self.on_call is not None:
            self.on_call(func, p)
        self._expire()
        if func == "GetControlSurfaceJson":
            return {"ReturnValue": json.dumps(self.surface)}
        cid = p.get("Id", "")
        a = self.axis(cid)
        if func == "SetControl":
            if a is None or a["bReadOnly"] or a["kind"] in ("Action", "Text") or cid in self.refuse:
                return {"ReturnValue": False}
            self.sets.append((cid, p["Value"], p["Source"]))
            v = min(max(p["Value"], a["range"]["x"]), a["range"]["y"])
            self.values[cid] = v
            self.targets[cid] = v
            self._last[cid] = (time.monotonic(), p["Source"])
            self.set_times.append((time.monotonic(), cid, v))
            self.events.append(("set", cid))
            return {"ReturnValue": True}
        if func == "TriggerControl":
            self.triggered.append(cid)
            return {"ReturnValue": a is not None}
        if func == "ReleaseControl":
            self.released.append(cid)
            self.release_times.append((time.monotonic(), cid))
            self.events.append(("release", cid))
            self.targets.pop(cid, None)
            self._last.pop(cid, None)
            if a is not None and a["kind"] == "Continuous":
                self.values[cid] = float(a["defaultValue"])
            return {"ReturnValue": a is not None}
        if func == "GetControlValue":
            return {"ReturnValue": self.values.get(cid, 0.0)}
        if func == "GetControlTarget":
            if cid in self.targets:
                return {"OutTarget": self.targets[cid], "ReturnValue": True}
            return {"OutTarget": 0, "ReturnValue": False}
        if func == "GetControlText":
            return {"ReturnValue": self.texts.get(cid, "")}
        raise UnrealRemoteError(f"HTTP 400: no function {func}")


class FakeComponent:
    """Answers from a {function: response} table; callables and overrides make it stateful."""

    def __init__(self, responses: dict, handlers: dict[str, Callable[[dict], dict]] | None = None):
        self.responses = copy.deepcopy(responses)
        self.handlers = dict(handlers or {})

    def __call__(self, func: str, p: dict) -> dict:
        if func in self.handlers:
            return self.handlers[func](p)
        if func in self.responses:
            return copy.deepcopy(self.responses[func])
        raise UnrealRemoteError(f"HTTP 400: Function {func} not found")


class FakeRemote(UnrealRemote):
    """Remote Control without an engine: actors with components, plus static bridges."""

    def __init__(self):
        super().__init__(host="fake", http_port=1)
        self.actors: dict[str, dict[str, tuple[str, Callable]]] = {}
        self.actor_classes: dict[str, str] = {}
        self.objects: dict[str, Callable] = {}      # bridge CDOs / any object path -> handler
        self.calls: list[tuple[str, str, dict]] = []
        self.bridge_down = False                    # RammsCoreBridge refuses (not allow-listed / not loaded)

    def add_actor(self, name: str, components: dict[str, tuple[str, Callable]] | None = None, cls: str = "",
                  handler: Callable | None = None) -> str:
        """An actor with components; `handler` answers calls on the actor itself."""
        path = LEVEL + name
        self.actors[path] = components or {}
        self.actor_classes[path] = cls or name.rsplit("_C_", 1)[0]
        if handler is not None:
            self.objects[path] = handler
        return path

    def calls_to(self, func: str) -> list[dict]:
        return [p for _, f, p in self.calls if f == func]

    # ── transport ───────────────────────────────────────────────────

    def _request(self, method: str, endpoint: str, body: dict | None = None) -> Any:
        if endpoint == "/remote/info":
            return {"HttpRoutes": []}
        if endpoint != "/remote/object/call":
            raise UnrealRemoteError(f"HTTP 404 on {method} {endpoint}")
        path, func, params = body["objectPath"], body["functionName"], body.get("parameters", {})
        self.calls.append((path, func, params))
        if path == CORE_BRIDGE:
            if self.bridge_down:
                raise UnrealRemoteError("HTTP 400 on PUT /remote/object/call: function not allowed")
            return self._core_bridge(func, params)
        if path in self.objects:
            return self.objects[path](func, params)
        actor, _, comp = path.rpartition(".")
        if actor in self.actors and comp in self.actors[actor]:
            return self.actors[actor][comp][1](func, params)
        if path in self.actors:
            raise UnrealRemoteError(f"HTTP 400: {func} on actor {path}")
        raise UnrealRemoteError(f"HTTP 400: object not found {path}")

    def _core_bridge(self, func: str, p: dict) -> dict:
        if func == "FindActorsByComponent":
            f = p["ComponentFilter"].lower()
            return {"ReturnValue": [f"{a}|{n}:{cls}" for a, comps in self.actors.items()
                                    for n, (cls, _) in comps.items() if f in cls.lower() or f in n.lower()]}
        if func == "FindComponents":
            comps = self.actors.get(p["ActorPath"], {})
            f = p.get("ClassNameFilter", "").lower()
            return {"ReturnValue": [f"{n}:{cls}" for n, (cls, _) in comps.items() if not f or f in cls.lower() or f in n.lower()]}
        if func == "FindActors":
            f = p["ClassNameFilter"].lower()
            return {"ReturnValue": [a for a, cls in self.actor_classes.items() if f in cls.lower()]}
        if func == "GetAllActorPaths":
            return {"ReturnValue": list(self.actors)}
        raise UnrealRemoteError(f"HTTP 400: no bridge function {func}")


class ModeSelector(FakeComponent):
    """The drive-mode selector, switching at once (or after `switch_polls` polls of IsSwitchingMode)."""

    def __init__(self, responses: dict, switch_polls: int = 0, refuse: set[str] | None = None):
        super().__init__(responses)
        self.active = responses["GetActiveModeId"]["ReturnValue"]
        self.ids = responses["GetAvailableModeIds"]["ReturnValue"]
        self.pending: str | None = None
        self.polls_left = 0
        self.switch_polls = switch_polls
        self.refuse = refuse or set()
        self.status = responses["GetSwitchStatus"]["ReturnValue"]
        self.handlers.update(SetActiveMode=self._set, GetActiveModeId=lambda p: {"ReturnValue": self.active},
                             IsSwitchingMode=self._switching, GetSwitchStatus=lambda p: {"ReturnValue": self.status},
                             CycleMode=self._cycle)

    def _set(self, p):
        m = p["ModeId"]
        if m not in self.ids:
            return {"ReturnValue": False}
        if m in self.refuse:
            self.pending, self.polls_left = m, 1
            self.status = f"waiting to hand over to {m}"
            return {"ReturnValue": True}
        if self.switch_polls:
            self.pending, self.polls_left = m, self.switch_polls
            self.status = f"waiting to hand over to {m}"
        else:
            self.active, self.status = m, f"{m} at once"
        return {"ReturnValue": True}

    def _switching(self, p):
        if self.pending is not None:
            self.polls_left -= 1
            if self.polls_left <= 0:
                if self.pending in self.refuse:
                    self.status = f"refused {self.pending}: the robot is still moving"
                else:
                    self.active, self.status = self.pending, f"{self.pending} after the stance"
                self.pending = None
                return {"ReturnValue": False}
            return {"ReturnValue": True}
        return {"ReturnValue": False}

    def _cycle(self, p):
        i = self.ids.index(self.active)
        nxt = self.ids[(i + 1) % len(self.ids)]
        if nxt in self.refuse:
            self.pending, self.polls_left = nxt, 1
            self.status = f"waiting to hand over to {nxt}"
            return {"ReturnValue": True}
        self.active = nxt
        return {"ReturnValue": True}


def robot_world(gen: str = "gen1", name: str | None = None, deadman: bool = True, **selector_kw
                ) -> tuple[FakeRemote, str, dict]:
    """A world holding one robot built from the fixtures. Returns (remote, actor path, components by name).
    deadman=False makes the control surface latch Continuous commands, as sims before the deadman did."""
    ue = FakeRemote()
    surface = load(gen, "surface.json")
    responses = load(gen, "responses.json")
    comps: dict[str, tuple[str, Callable]] = {
        "ControlSurface": ("RammsRobotControlSurfaceComponent", FakeSurface(surface, deadman=deadman))}
    for comp, table in responses.items():
        cls = table.pop("_class")
        handler = ModeSelector(table, **selector_kw) if comp == "DriveModeSelector" else FakeComponent(table)
        if "GetPhase" in table:
            handler.responses.setdefault("Abort", {})          # void: an empty response
        comps[comp] = (cls, handler)
    path = ue.add_actor(name or surface["robotName"], comps)
    return ue, path, {n: h for n, (_, h) in comps.items()}
