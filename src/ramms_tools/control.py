"""
Drive any RAMMS robot through its control surface over the Remote Control API.

A robot pawn carries a ``URammsRobotControlSurfaceComponent`` ("ControlSurface")
that gathers every control its controllers offer -- drive axes, the drive
mode, curb crossing, seat, linkages, arm, gripper, cameras, sim -- into one
list of dotted ids (``drive.forward``, ``curb.assist``, ``seat.height`` ...).
A remote client needs nothing robot-specific to describe and drive it:

    from ramms_tools.control import ControlSurface

    cs = ControlSurface.find(ue)[0]                  # first robot in the world
    print(cs.robot_name, cs.groups())
    for a in cs.axes("Curb"):
        print(a.id, a.kind, a.units, a.range, a.enum_labels)

    cs.set_enum("curb.assist", "On")                 # an Enum by label (or index)
    print(cs.get_enum("curb.phase"))                 # "Idle"
    print(cs.status("curb.status"))                  # StatusText(text='Ready', severity='Info')
    cs.trigger("camera.next")                        # an Action
    cs.drive(forward=0.4, turn=0.0, seconds=3)       # held at 10 Hz for 3 s, then released

Held commands
-------------
A ``Remote`` (or ``Autonomy``) command outranks local input (keyboard,
gamepad, touch) for ``EXTERNAL_HOLD_SECONDS`` (0.3 s) after it is sent. What
happens to a **Continuous** control (``drive.forward`` ...) that is not sent
again depends on the sim:

- sims with the control-surface deadman (RammsCore, 2026-11-15 on): a
  Continuous control set by Remote / Autonomy springs back to its default
  0.3 s after its last send, as if released;
- older sims **latch** it: the robot keeps doing the last command until it is
  released, set again, or local input moves that control.

So keep re-sending a held command at >= 10 Hz -- ``hold()``, ``keep_alive()``
or ``drive(seconds=)`` do -- and release it at the end. This module releases
what it holds when a hold ends, raises or is interrupted, and at interpreter
exit it stops every running ``KeepAlive`` and releases every Continuous
control set once (``set()``, ``drive(once=True)``) that was not released
(``release_all()``; the CLIs also do it on SIGINT / SIGTERM / SIGHUP). Nothing
can release after a hard kill (SIGKILL, a crash of the interpreter, power or
network loss): a latching sim then keeps the last command, a sim with the
deadman stops within 0.3 s. Position / Velocity / Enum controls are not
affected by the deadman on any sim: they hold their target until released.

Control kinds (``ControlAxis.kind``): Continuous (springs back to its default
on release), Position, Velocity, Action (``trigger``), Enum (value = index
into ``enum_labels``; trigger = next choice), Text (read ``text``/``status``;
its value is the severity: 0 Info, 1 Attention, 2 Fault).
"""

from __future__ import annotations

import atexit
import logging
import math
import threading
import time
import weakref
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, NamedTuple, Optional, Union

from ramms_tools._rc import as_bool, as_float, as_json, name as _fname, return_value, short_name, vec2
from ramms_tools.unreal_remote import RemoteObjectProxy, UnrealRemote, UnrealRemoteError

logger = logging.getLogger(__name__)

__all__ = [
    "ControlSurface", "ControlAxis", "SurfaceDescription", "StatusText", "KeepAlive", "ControlError",
    "COMPONENT_CLASS", "KINDS", "SOURCES", "SEVERITIES", "EXTERNAL_HOLD_SECONDS", "severity_name", "release_all",
]

COMPONENT_CLASS = "RammsRobotControlSurfaceComponent"
KINDS = ("Continuous", "Position", "Velocity", "Action", "Enum", "Text")
SOURCES = ("Keyboard", "Gamepad", "Touch", "Remote", "Autonomy", "Script")
SEVERITIES = ("Info", "Attention", "Fault")
# How long a Remote / Autonomy command outranks local input (ExternalHoldSeconds on the sim side); with the deadman,
# also how long a Continuous one lasts without being sent again.
EXTERNAL_HOLD_SECONDS = 0.3
DEFAULT_HOLD_RATE = 10.0
DRIVE_IDS = ("drive.forward", "drive.turn", "drive.strafe")

ChoiceLike = Union[str, int, float]


class ControlError(UnrealRemoteError):
    """A control that does not exist, or cannot take what was asked of it."""


class StatusText(NamedTuple):
    """A Text control (or a controller's status line): what it says and how much it needs the reader."""
    text: str
    severity: str = "Info"      # Info, Attention or Fault

    def __str__(self) -> str:
        return self.text if self.severity == "Info" else f"[{self.severity}] {self.text}"


def severity_name(value: Any) -> str:
    """A severity as a name: 0/1/2 (a Text control's value) or a name ("Attention",
    "ERammsControlTextSeverity::Fault") -> "Info" / "Attention" / "Fault". Anything unreadable (NaN, None, a
    malformed answer) is "Info"."""
    if isinstance(value, str):
        s = value.rsplit("::", 1)[-1].strip()
        for sev in SEVERITIES:
            if s.lower() == sev.lower():
                return sev
        try:
            value = float(s)
        except ValueError:
            return s or "Info"
    f = as_float(value, 0.0)
    if not math.isfinite(f):
        return "Info"
    i = int(round(f))
    return SEVERITIES[max(0, min(i, len(SEVERITIES) - 1))]


def _norm(s: str) -> str:
    return "".join(ch for ch in str(s).lower() if ch.isalnum())


@dataclass
class ControlAxis:
    """One control a robot exposes (FRammsControlAxis)."""
    id: str
    group: str = ""
    display_name: str = ""
    kind: str = "Continuous"        # Continuous, Position, Velocity, Action, Enum, Text
    units: str = "Normalized"       # Normalized, Radians, Degrees, Centimeters, Meters, RadiansPerSecond, CentimetersPerSecond, None
    range: tuple[float, float] = (-1.0, 1.0)
    default: float = 0.0
    readback: bool = True           # the robot reports its live value
    read_only: bool = False         # reported, not commanded (gripper.closed, curb.phase, curb.status)
    order: int = 0
    paired_axis: str = ""           # partner of a 2-D control ("drive.turn" for drive.forward); "" when none
    region_outline: list[tuple[float, float]] = field(default_factory=list)   # reachable (horizontal, vertical) region of a pair
    enum_labels: list[str] = field(default_factory=list)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "ControlAxis":
        g = lambda *keys, default=None: next((d[k] for k in keys if k in d), default)
        rng = vec2(g("range", "Range", default={"x": -1.0, "y": 1.0}))
        return cls(
            id=_fname(g("id", "Id", default="")),
            group=_fname(g("group", "Group", default="")),
            display_name=str(g("displayName", "DisplayName", default="") or ""),
            kind=str(g("kind", "Kind", default="Continuous") or "Continuous").rsplit("::", 1)[-1],
            units=str(g("units", "Units", default="Normalized") or "Normalized").rsplit("::", 1)[-1],
            range=rng,
            default=as_float(g("defaultValue", "DefaultValue", default=0.0)),
            readback=as_bool(g("bReadback", "readback", default=True)),
            read_only=as_bool(g("bReadOnly", "readOnly", default=False)),
            order=int(as_float(g("order", "Order", default=0))),
            paired_axis=_fname(g("pairedAxis", "PairedAxis", default="")),
            region_outline=[vec2(p) for p in (g("regionOutline", "RegionOutline", default=[]) or [])],
            enum_labels=[str(x) for x in (g("enumLabels", "EnumLabels", default=[]) or [])],
            raw=d,
        )

    # ── kinds ───────────────────────────────────────────────────────

    @property
    def is_action(self) -> bool:
        return self.kind == "Action"

    @property
    def is_enum(self) -> bool:
        return self.kind == "Enum"

    @property
    def is_text(self) -> bool:
        return self.kind == "Text"

    @property
    def is_continuous(self) -> bool:
        """Springs back to its default on release (and, on a sim with the deadman, 0.3 s after a Remote / Autonomy
        command that is not sent again): has to be held (re-sent) to keep acting."""
        return self.kind == "Continuous"

    @property
    def commandable(self) -> bool:
        """Can be set (SetControl): not read-only, not an Action or a Text."""
        return not self.read_only and self.kind not in ("Action", "Text")

    @property
    def label(self) -> str:
        return self.display_name or self.id

    def clamp(self, value: float) -> float:
        lo, hi = self.range
        return min(max(value, lo), hi) if lo < hi else value

    # ── enum choices ────────────────────────────────────────────────

    def enum_index(self, choice: ChoiceLike) -> int:
        """The index of a choice given by label (case and spacing ignored, a unique prefix will do) or index."""
        labels = self.enum_labels
        if not isinstance(choice, str):
            i = int(choice)
            if 0 <= i < len(labels) or not labels:
                return i
            raise ControlError(f"{self.id}: no choice {i} (0..{len(labels) - 1}: {', '.join(labels)})")
        s = choice.strip()
        for i, lab in enumerate(labels):
            if lab.lower() == s.lower():
                return i
        key = _norm(s)
        for i, lab in enumerate(labels):
            if _norm(lab) == key:
                return i
        if s.lstrip("-").isdigit():
            return self.enum_index(int(s))
        starts = [i for i, lab in enumerate(labels) if key and _norm(lab).startswith(key)]
        if len(starts) == 1:
            return starts[0]
        raise ControlError(f"{self.id}: no choice {choice!r} (choices: {', '.join(labels)})")

    def enum_label(self, value: Any) -> str:
        """The label of an Enum value (an index, as GetControlValue reports it); the value as text when it is not
        one (NaN from an unreadable answer, or out of range)."""
        f = as_float(value, -1.0)
        i = int(round(f)) if math.isfinite(f) else -1
        return self.enum_labels[i] if 0 <= i < len(self.enum_labels) else str(value)


@dataclass
class SurfaceDescription:
    """A robot's control surface (FRammsControlSurface): its name, groups and controls."""
    robot_name: str = ""
    groups: list[str] = field(default_factory=list)
    axes: list[ControlAxis] = field(default_factory=list)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, data: Union[str, dict, None]) -> "SurfaceDescription":
        d = as_json(data) if not isinstance(data, dict) else data
        d = d or {}
        return cls(
            robot_name=str(d.get("robotName", d.get("RobotName", "")) or ""),
            groups=[_fname(g) for g in (d.get("groups", d.get("Groups", [])) or [])],
            axes=[ControlAxis.from_json(a) for a in (d.get("axes", d.get("Axes", [])) or [])],
            raw=d,
        )

    def axis(self, control_id: str) -> Optional[ControlAxis]:
        cid = control_id.lower()     # FNames compare case-insensitively
        return next((a for a in self.axes if a.id.lower() == cid), None)

    def in_group(self, group: Optional[str] = None) -> list[ControlAxis]:
        """Controls of a group (all with None), in the surface's order within each group."""
        if group is None:
            return list(self.axes)
        g = group.lower()
        return sorted((a for a in self.axes if a.group.lower() == g), key=lambda a: a.order)


# ── releasing what this process holds ───────────────────────────────────

_LIVE_KEEPALIVES: "weakref.WeakSet[KeepAlive]" = weakref.WeakSet()


def _release_ids(proxy: RemoteObjectProxy, once: dict[str, str]) -> None:
    """Release every control in `once` ({id: source}), as its source; used by finalizers, so no `self`."""
    for cid, source in list(once.items()):
        try:
            proxy.call("ReleaseControl", Id=cid, Source=source)
        except Exception as e:          # at exit: never raise, say what could not be released
            logger.warning("release %s at exit failed: %s", cid, e)
        once.pop(cid, None)


def release_all() -> None:
    """
    Stop every running KeepAlive (releasing what it holds) and release every Continuous control set once that was
    not released. Runs at interpreter exit (atexit); the CLIs also run it on SIGINT / SIGTERM / SIGHUP. Safe to call
    more than once.
    """
    for ka in list(_LIVE_KEEPALIVES):
        try:
            ka.stop()
        except Exception as e:
            logger.warning("stopping a keep-alive at exit failed: %s", e)
    for cs in list(_SURFACES):
        try:
            cs.release_once()
        except Exception as e:
            logger.warning("releasing at exit failed: %s", e)


_SURFACES: "weakref.WeakSet[ControlSurface]" = weakref.WeakSet()
atexit.register(release_all)


class ControlSurface:
    """
    One robot's control surface (its ControlSurface component).

    Args:
        ue: the Remote Control client.
        component_path: the component's object path.
        actor_path: the pawn's actor path (for display, and for finding its other components).
        source: the ERammsControlSource commands are sent as (Remote, Autonomy, Script ...).
    """

    def __init__(self, ue: UnrealRemote, component_path: str, actor_path: str = "", source: str = "Remote"):
        if source not in SOURCES:
            raise ValueError(f"source must be one of {', '.join(SOURCES)}, not {source!r}")
        self.ue = ue
        self.component_path = component_path
        self.actor_path = actor_path or component_path.rsplit(".", 1)[0]
        self.source = source
        self._proxy = RemoteObjectProxy(ue, component_path)
        self._desc: Optional[SurfaceDescription] = None
        # Continuous controls set once (set(), drive(once=True)) and not released yet: {id: source}. Released by
        # release()/stop()/release_once(), and by a finalizer when this object is collected or the process exits.
        self._once: dict[str, str] = {}
        self._once_finalizer = weakref.finalize(self, _release_ids, self._proxy, self._once)
        _SURFACES.add(self)
        self.last_hold_refused: dict[str, int] = {}

    def __repr__(self) -> str:
        return f"ControlSurface({self.component_path!r})"

    # ── discovery ───────────────────────────────────────────────────

    @classmethod
    def find(cls, ue: UnrealRemote, source: str = "Remote") -> list["ControlSurface"]:
        """Every robot in the world that has a control surface (RammsCoreBridge.FindActorsByComponent). Raises
        ControlError when the bridge cannot be asked (not reachable, or not allow-listed) -- [] means no robot."""
        try:
            entries = ue.find_actors_by_component(COMPONENT_CLASS, strict=True)
        except UnrealRemoteError as e:
            raise ControlError(f"could not list the robots (RammsCoreBridge.FindActorsByComponent): {e}") from e
        return [cls(ue, e["component_path"], e["actor_path"], source=source)
                for e in entries if str(e.get("component_class", "")).endswith(COMPONENT_CLASS)]

    @classmethod
    def find_by_robot(cls, ue: UnrealRemote, robot: str, source: str = "Remote") -> Optional["ControlSurface"]:
        """
        The surface whose actor or robotName matches `robot`: an exact actor path / name or robotName first (case
        ignored), then a substring of the actor path or robotName. None when nothing matches; ControlError when a
        substring matches more than one robot (name one exactly).
        """
        surfaces = cls.find(ue, source=source)
        key = robot.strip().lower()

        def names(s: "ControlSurface") -> tuple[str, str, str]:
            try:
                rn = s.robot_name.lower()
            except UnrealRemoteError:
                rn = ""
            return s.actor_path.lower(), short_name(s.actor_path).lower(), rn

        table = [(s, names(s)) for s in surfaces]
        exact = [s for s, (p, n, rn) in table if key in (p, n, rn)]
        if exact:
            return exact[0]
        hits = [s for s, (p, n, rn) in table if key in p or key in rn]
        if len(hits) > 1:
            raise ControlError(f"{robot!r} matches {len(hits)} robots ({', '.join(short_name(s.actor_path) for s in hits)}): "
                               "name one exactly")
        return hits[0] if hits else None

    @property
    def proxy(self) -> RemoteObjectProxy:
        return self._proxy

    # ── description ─────────────────────────────────────────────────

    def describe(self, refresh: bool = False) -> SurfaceDescription:
        """The surface (cached; refresh=True to read it again, e.g. after a drive-mode switch changed it)."""
        if self._desc is None or refresh:
            self._desc = SurfaceDescription.from_json(self._proxy.call("GetControlSurfaceJson"))
        return self._desc

    @property
    def robot_name(self) -> str:
        return self.describe().robot_name or short_name(self.actor_path)

    def groups(self) -> list[str]:
        return list(self.describe().groups)

    def axes(self, group: Optional[str] = None) -> list[ControlAxis]:
        return self.describe().in_group(group)

    def ids(self, group: Optional[str] = None) -> list[str]:
        return [a.id for a in self.axes(group)]

    def has(self, control_id: str) -> bool:
        return self.describe().axis(control_id) is not None

    def axis(self, control_id: str) -> ControlAxis:
        """A control by id. The description is read again once before giving up, since a drive-mode switch
        changes which controls exist; raises ControlError for an id the robot does not have."""
        a = self.describe().axis(control_id)
        if a is None:
            a = self.describe(refresh=True).axis(control_id)
        if a is None:
            raise ControlError(f"{self.robot_name} has no control {control_id!r}")
        return a

    # ── commands ────────────────────────────────────────────────────

    def _src(self, source: Optional[str]) -> str:
        return source or self.source

    def _set_raw(self, control_id: str, value: float, source: Optional[str] = None) -> bool:
        return as_bool(self._proxy.call("SetControl", Id=control_id, Value=float(value), Source=self._src(source)))

    def set(self, control_id: str, value: Union[float, str], source: Optional[str] = None) -> bool:
        """
        Command a Continuous / Position / Velocity control (the surface clamps it to its range), or an Enum by
        index or label. False when the robot refused it.

        A Continuous control set this way is a single command: on a sim with the deadman it springs back 0.3 s
        later; an older sim latches it until it is released. It is remembered and released by release(), stop(),
        release_once() or at interpreter exit. To keep one acting, use hold() / keep_alive() / drive(seconds=).
        """
        a = self.axis(control_id)
        if a.is_action:
            raise ControlError(f"{a.id} is an Action: trigger it")
        if not a.commandable:
            raise ControlError(f"{a.id} is read-only ({a.kind})")
        if a.is_enum:
            value = a.enum_index(value)
        elif isinstance(value, str):
            try:
                value = float(value)
            except ValueError:
                raise ControlError(f"{a.id} takes a number, not {value!r}") from None
        ok = self._set_raw(a.id, float(value), source)
        if ok and a.is_continuous:
            self._once[a.id] = self._src(source)
        return ok

    def set_many(self, values: dict[str, Union[float, str]], source: Optional[str] = None) -> bool:
        """Set several controls; True when every one was taken."""
        ok = True
        for cid, v in values.items():
            ok = self.set(cid, v, source) and ok
        return ok

    def trigger(self, control_id: str, source: Optional[str] = None) -> bool:
        """Fire an Action (camera.next, curb.negotiate, sim.reset ...), or step an Enum to its next choice."""
        a = self.axis(control_id)
        if a.is_text or (a.read_only and not a.is_action):
            raise ControlError(f"{a.id} is read-only ({a.kind})")
        return as_bool(self._proxy.call("TriggerControl", Id=a.id, Source=self._src(source)))

    def release(self, control_id: str, source: Optional[str] = None) -> bool:
        """Let go: a Continuous control springs back to its default, a Position control stops holding. False when
        there was nothing to release (or the robot has no such control in this mode)."""
        ok = as_bool(self._proxy.call("ReleaseControl", Id=control_id, Source=self._src(source)))
        self._once.pop(control_id, None)
        return ok

    def release_many(self, ids: Iterable[str], source: Optional[str] = None) -> None:
        for cid in ids:
            try:
                self.release(cid, source)
            except UnrealRemoteError as e:
                logger.warning("release %s failed: %s", cid, e)

    def release_once(self) -> list[str]:
        """Release every Continuous control set once (set(), drive(once=True)) and not released since. Returns
        their ids."""
        held = dict(self._once)
        for cid, source in held.items():
            try:
                self.release(cid, source)
            except UnrealRemoteError as e:
                logger.warning("release %s failed: %s", cid, e)
        return list(held)

    # ── reads ───────────────────────────────────────────────────────

    def get(self, control_id: str) -> float:
        """The live value (readback) of a control: an Enum's index, a Text's severity; NaN for an unreadable answer."""
        v = return_value(self._proxy.call("GetControlValue", Id=control_id))
        return as_float(v, float("nan"))

    def target(self, control_id: str) -> Optional[float]:
        """The commanded target of a control (what a panel's slider shows), or None when there is none
        (nothing commanded yet, or released)."""
        r = self._proxy.call_full("GetControlTarget", Id=control_id)
        if not as_bool(r.get("ReturnValue", False)):
            return None
        return as_float(r.get("OutTarget"), float("nan"))

    def text(self, control_id: str) -> str:
        """What a Text control reports now ("" when nothing, or not a Text control)."""
        v = return_value(self._proxy.call("GetControlText", Id=control_id))
        return "" if v is None else str(v)

    def severity(self, control_id: str) -> str:
        """A Text control's severity: Info, Attention or Fault."""
        return severity_name(self.get(control_id))

    def status(self, control_id: str) -> StatusText:
        """A Text control's words and severity (curb.status)."""
        return StatusText(self.text(control_id), self.severity(control_id))

    def get_enum(self, control_id: str) -> str:
        """An Enum control's current choice, as its label."""
        return self.axis(control_id).enum_label(self.get(control_id))

    def set_enum(self, control_id: str, choice: ChoiceLike, source: Optional[str] = None) -> bool:
        """Set an Enum control by label ("On", "Rolling", "Holonomic"; case and spacing ignored) or index."""
        a = self.axis(control_id)
        if not a.is_enum:
            raise ControlError(f"{a.id} is not an Enum ({a.kind})")
        if a.read_only:
            raise ControlError(f"{a.id} is read-only")
        return self._set_raw(a.id, float(a.enum_index(choice)), source)

    def read(self, control_id: str) -> Any:
        """A control's value in its natural form: an Enum's label, a Text's StatusText, else the number."""
        a = self.axis(control_id)
        if a.is_text:
            return self.status(a.id)
        if a.is_enum:
            return a.enum_label(self.get(a.id))
        return self.get(a.id)

    def read_many(self, ids: Iterable[str]) -> dict[str, Any]:
        return {cid: self.read(cid) for cid in ids}

    def readback(self, group: Optional[str] = None) -> dict[str, Any]:
        """Every control that reports a value (readback, or a Text) in a group (all with None)."""
        return {a.id: self.read(a.id) for a in self.axes(group) if (a.readback or a.is_text) and not a.is_action}

    # ── holding ─────────────────────────────────────────────────────

    def hold(self, values: dict[str, float], seconds: float, rate: float = DEFAULT_HOLD_RATE, release: bool = True,
             should_stop: Optional[Callable[[], bool]] = None, on_tick: Optional[Callable[[float], None]] = None,
             source: Optional[str] = None) -> int:
        """
        Keep commanding `values` at `rate` Hz for `seconds`, then release them. Stops early when should_stop() is
        true; on_tick(elapsed) runs after each send. The release also runs when a send raises or the hold is
        interrupted (KeyboardInterrupt). release=False leaves them as they are: on a latching sim a Continuous
        control keeps acting, so it is remembered for release_once() / exit. Returns how many times they were sent;
        sends the robot refused (another source holds the control, or this drive mode does not take it) are
        counted in last_hold_refused ({id: n}) and logged. Enum values may be labels.
        """
        if rate <= 0:
            raise ValueError("rate must be > 0")
        if 1.0 / rate >= EXTERNAL_HOLD_SECONDS:
            logger.warning("hold at %.1f Hz: the command lapses between sends (it holds %.1f s)", rate, EXTERNAL_HOLD_SECONDS)
        resolved = self._resolve(values)
        refused: dict[str, int] = {}
        self.last_hold_refused = refused
        period, n, t0 = 1.0 / rate, 0, time.monotonic()
        try:
            while True:
                elapsed = time.monotonic() - t0
                if elapsed >= seconds or (should_stop is not None and should_stop()):
                    break
                for cid, v in resolved.items():
                    if not self._set_raw(cid, v, source):
                        refused[cid] = refused.get(cid, 0) + 1
                n += 1
                if on_tick is not None:
                    on_tick(elapsed)
                delay = min(t0 + n * period, t0 + seconds) - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        finally:
            if release:
                self.release_many(resolved, source)
            else:
                for cid in resolved:
                    a = self.describe().axis(cid)
                    if a is not None and a.is_continuous:
                        self._once[cid] = self._src(source)
            if refused:
                logger.warning("%s refused %s", self.robot_name,
                               ", ".join(f"{cid} {k} of {n} sends" for cid, k in refused.items()))
        return n

    def keep_alive(self, values: Optional[dict[str, float]] = None, rate: float = DEFAULT_HOLD_RATE,
                   source: Optional[str] = None) -> "KeepAlive":
        """A background sender that keeps `values` commanded until stopped (then releases them):

            with cs.keep_alive({"drive.forward": 0.3}) as ka:
                time.sleep(2); ka.update({"drive.turn": 0.5}); time.sleep(1)
        """
        return KeepAlive(self, values or {}, rate=rate, source=source)

    def _resolve(self, values: dict[str, Union[float, str]]) -> dict[str, float]:
        out: dict[str, float] = {}
        for cid, v in values.items():
            a = self.axis(cid)
            if not a.commandable:
                raise ControlError(f"{a.id} cannot be held ({'read-only' if a.read_only else a.kind})")
            out[a.id] = float(a.enum_index(v)) if a.is_enum else float(v)
        return out

    # ── driving ─────────────────────────────────────────────────────

    def drive_values(self, forward: float = 0.0, turn: float = 0.0, strafe: Optional[float] = None) -> dict[str, float]:
        """The drive controls for a command; strafe needs drive.strafe (holonomic modes)."""
        if not self.has("drive.forward"):
            self.describe(refresh=True)
            if not self.has("drive.forward"):
                raise ControlError(f"{self.robot_name} has no drive.forward control")
        out = {"drive.forward": float(forward), "drive.turn": float(turn)}
        if strafe is not None:
            if self.has("drive.strafe") or self.describe(refresh=True).axis("drive.strafe") is not None:
                out["drive.strafe"] = float(strafe)
            elif strafe != 0.0:
                raise ControlError(f"{self.robot_name} has no drive.strafe in this drive mode (a holonomic mode has it)")
        return out

    def drive(self, forward: float = 0.0, turn: float = 0.0, strafe: Optional[float] = None,
              seconds: Optional[float] = None, rate: float = DEFAULT_HOLD_RATE, source: Optional[str] = None,
              should_stop: Optional[Callable[[], bool]] = None, once: bool = False) -> bool:
        """
        Drive with normalized commands (-1..1), held at `rate` Hz for `seconds` and released at the end (also on an
        error or Ctrl+C). Returns True (False when the robot refused every send).

        once=True (and no seconds) sends the command a single time instead -- for a caller running its own loop at
        >= 10 Hz. Such a command springs back 0.3 s later on a sim with the deadman, and is LATCHED by an older sim
        until stop() / release / exit, which release it. Without seconds or once=True this raises ValueError.
        """
        values = self.drive_values(forward, turn, strafe)
        if seconds is None:
            if not once:
                raise ValueError("drive() needs seconds= (held, then released) or once=True (a single command: it "
                                 "springs back after 0.3 s on a sim with the deadman, older sims keep it until stop())")
            ok = False
            for cid, v in values.items():
                if self._set_raw(cid, v, source):
                    ok = True
                    self._once[cid] = self._src(source)
            return ok
        n = self.hold(values, seconds, rate=rate, source=source, should_stop=should_stop)
        return n == 0 or any(self.last_hold_refused.get(cid, 0) < n for cid in values)

    def stop(self, source: Optional[str] = None) -> None:
        """Release the drive controls (the robot comes to rest under local input, if any). drive.strafe is released
        too whether or not the cached description has it: another client may have switched to a holonomic mode."""
        ids = [cid for cid in DRIVE_IDS]
        for cid in ids:
            try:
                self.release(cid, source)
            except UnrealRemoteError as e:
                logger.warning("release %s failed: %s", cid, e)


class KeepAlive:
    """
    Re-sends a set of control values at a fixed rate on a background thread (see ControlSurface.keep_alive), and
    releases them when stopped. The sender releases them itself as its last act, so a release can never be
    followed by a late send; pause() releases and stops sending until resume(). Every started KeepAlive is stopped
    at interpreter exit (release_all). Sends the robot refused are counted in `refused` ({id: n}).
    """

    def __init__(self, surface: ControlSurface, values: dict[str, float], rate: float = DEFAULT_HOLD_RATE,
                 source: Optional[str] = None):
        if rate <= 0:
            raise ValueError("rate must be > 0")
        self.surface = surface
        self.rate = rate
        self.source = source
        self.sends = 0
        self.errors = 0
        self.refused: dict[str, int] = {}
        self._values = surface._resolve(values)
        self._touched: set[str] = set(self._values)
        self._lock = threading.Lock()           # the values
        self._send = threading.RLock()          # one send round, release or update at a time
        self._stop = threading.Event()
        self._paused = False
        self._release_on_stop = True
        self._released = False
        self._thread: Optional[threading.Thread] = None
        # Long enough for a send already in flight to come back (the HTTP timeout), so stop() normally returns
        # with the release done.
        self.join_timeout = max(5.0, float(getattr(surface.ue, "timeout", 5.0)) + 1.0)

    def start(self) -> "KeepAlive":
        if self._thread is None:
            self._stop.clear()
            self._released = False
            self._release_on_stop = True
            self._thread = threading.Thread(target=self._run, name="ramms-keepalive", daemon=True)
            _LIVE_KEEPALIVES.add(self)
            self._thread.start()
        return self

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def paused(self) -> bool:
        return self._paused

    def update(self, values: dict[str, float], replace: bool = False) -> None:
        """Change what is sent (merged into the current values, or replacing them). Controls dropped by
        replace=True are released at once -- between two send rounds, so they are not sent again after it."""
        resolved = self.surface._resolve(values)
        with self._send:
            with self._lock:
                dropped = set(self._values) - set(resolved) if replace else set()
                self._values = resolved if replace else {**self._values, **resolved}
                self._touched |= set(resolved)
            if dropped:
                self.surface.release_many(sorted(dropped), self.source)
                self._touched -= dropped

    @property
    def values(self) -> dict[str, float]:
        with self._lock:
            return dict(self._values)

    def pause(self, release: bool = True) -> None:
        """Stop sending (between two rounds) and, with release=True, release the controls now; resume() sends
        again."""
        with self._send:
            self._paused = True
            if release:
                self.surface.release_many(sorted(self._touched), self.source)

    def resume(self) -> None:
        with self._send:
            self._paused = False

    def _round(self) -> None:
        with self._send:
            if self._paused:
                return
            for cid, v in self.values.items():
                if self._stop.is_set() or self._paused:
                    return
                try:
                    if not self.surface._set_raw(cid, v, self.source):
                        n = self.refused.get(cid, 0) + 1
                        self.refused[cid] = n
                        if n == 1:
                            logger.warning("keep-alive %s: refused by the robot (another source holds it, or this "
                                           "drive mode does not take it)", cid)
                except UnrealRemoteError as e:
                    self.errors += 1
                    logger.warning("keep-alive %s: %s", cid, e)
            self.sends += 1

    def _run(self) -> None:
        period = 1.0 / self.rate
        try:
            while not self._stop.is_set():
                t = time.monotonic()
                self._round()
                self._stop.wait(max(0.0, period - (time.monotonic() - t)))
        finally:
            if self._release_on_stop:
                self._release()

    def _release(self) -> None:
        with self._send:
            if not self._released:
                self.surface.release_many(sorted(self._touched), self.source)
                self._released = True

    def stop(self, release: bool = True) -> None:
        """Stop sending and (release=True) release every control this keep-alive sent. Waits up to join_timeout
        for a send in flight; if the engine is slower than that, the sender releases when its call returns."""
        self._release_on_stop = release
        self._stop.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=self.join_timeout)
            if t.is_alive():
                logger.warning("keep-alive: a send is still waiting on the engine; it releases %s when it returns",
                               ", ".join(sorted(self._touched)))
            else:
                self._thread = None
        elif t is None and release:
            self._release()
        _LIVE_KEEPALIVES.discard(self)

    def __enter__(self) -> "KeepAlive":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
