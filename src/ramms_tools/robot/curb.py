"""
Curb crossings: the RAMMP Gen1 curb controller and the RAMMP Gen2 (lift-drive)
curb negotiation controller behind one interface.

Both offer the same essentials -- ``phase()``, ``status_line()`` (text and
severity: Info, Attention, Fault), curb assist on/off, ``start()``,
``abort()``, ``stats()``, ``wait_until_done()`` and ``cross()`` -- and each
adds its own:

- ``Gen1Curb`` (``URammsRammpGen1CurbController``, "CurbController"): the
  crossing style (``curb.style``: "Stop and step" or "Rolling") and
  ``crossing()`` (the step it is crossing and the chassis height it crosses at).
  It has no look-ahead sensing of its own.
- ``Gen2Curb`` (``URammsCurbNegotiationController``, "CurbNegotiation"):
  ``detect_step_ahead()`` / ``detect_terrain_ahead()`` (the step and terrain it
  senses ahead), the plan and schedule, the step log, and live measurements --
  tilt, base height, wheel progress / clearance / status, leg heights,
  stability margin and drive speed -- plus ``run_stats()``.

A crossing moves only while the rider's stick -- ``drive.forward``, which the
curb mode takes while it has the robot -- is held forward (both controllers'
``Stick.bRequired``, on by default); let go, it pauses. ``cross()`` starts one
and holds the stick for it, only while it is under way: the moment a poll sees
it Idle, Done, Aborted or Faulted the stick is released (the drive mode the
robot is handed back to takes the stick as it is held), and held again if it
starts once more (a Gen2 retry). A Gen2 crossing goes at the stick's share of
its planned speed; a Gen1 crossing at its planned speeds whatever the share.
Starting one selects the ``curb`` drive mode by itself: do not select it by
hand first (selecting it starts a crossing, or hands straight back).

Phases are the controllers' enum names (``"Idle"``, ``"FrontUp"``, ``"Lift"``,
``"Done"`` ...); ``phase_label()`` gives what the ``curb.phase`` control shows
("Front omni wheels up").
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, NamedTuple, Optional

from ramms_tools._rc import as_bool, as_float, enum_name, return_value, vec2, vec3
from ramms_tools.control import ControlError, ControlSurface, KeepAlive, StatusText, severity_name
from ramms_tools.unreal_remote import RemoteObjectProxy

# The stick a crossing is driven with. 0.5 is twice the controllers' engage threshold (FRammsCurbStick
# EngageFraction 0.25) and well clear of their release threshold (0.15), so it engages reliably; a Gen2 crossing then
# runs at half its planned speed, and whatever reaches the mode the robot is handed back to before the stick is
# released is a half-stick command, not a full one.
DEFAULT_CROSSING_STICK = 0.5


class CurbError(ControlError):
    """The robot has no such curb feature, or a curb command failed."""


class CurbStart(NamedTuple):
    """What start() / negotiate() got back: whether a crossing began, and why not."""
    ok: bool
    reason: str = ""


@dataclass
class CurbEvent:
    """A change seen while waiting on a crossing: the phase or the status line moved."""
    t: float                # seconds since the wait began
    phase: str              # enum name
    label: str              # as curb.phase shows it
    status: str
    severity: str

    def __str__(self) -> str:
        sev = "" if self.severity == "Info" else f" [{self.severity}]"
        return f"{self.t:6.1f} s  {self.label:28s} {self.status}{sev}"


@dataclass
class CurbRun:
    """How a wait_until_done() / cross() ended."""
    started: bool                    # the crossing was seen under way
    phase: str                       # final phase (enum name)
    status: StatusText
    seconds: float
    events: list[CurbEvent] = field(default_factory=list)
    stats: dict = field(default_factory=dict)
    timed_out: bool = False
    reason: str = ""                 # cross(): why the start was refused
    stick_refused: int = 0           # sends of the held stick the robot refused (another source held it)

    @property
    def succeeded(self) -> bool:
        return self.started and self.phase == "Done" and not self.timed_out


class CurbBase:
    """What both curb controllers offer. Use Robot.curb to get the right one."""

    kind = "curb"
    CLASS = ""
    INSTANCE = ""
    PHASE_NAMES: tuple[str, ...] = ()
    TERMINAL = frozenset({"Done", "Aborted"})
    STATS_FUNCTION = ""

    def __init__(self, surface: ControlSurface, component_path: str):
        self.surface = surface
        self.component_path = component_path
        self.proxy = RemoteObjectProxy(surface.ue, component_path)

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.component_path!r})"

    def _call(self, func: str, **params: Any) -> Any:
        return return_value(self.proxy.call(func, **params))

    def _full(self, func: str, **params: Any) -> dict:
        return self.proxy.call_full(func, **params)

    # ── state ───────────────────────────────────────────────────────

    def phase(self) -> str:
        """The phase as the controller's enum name ("Idle", "Approach", ... "Done")."""
        return enum_name(self._call("GetPhase"))

    def phase_label(self, phase: Optional[str] = None) -> str:
        """The phase as the curb.phase control shows it ("Front omni wheels up" for FrontUp)."""
        ph = self.phase() if phase is None else phase
        a = self.surface.describe().axis("curb.phase")
        if a is not None and ph in self.PHASE_NAMES and len(a.enum_labels) == len(self.PHASE_NAMES):
            return a.enum_labels[self.PHASE_NAMES.index(ph)]
        return ph

    def status_line(self) -> StatusText:
        """What curb.status shows: (text, severity) -- severity Info, Attention or Fault."""
        r = self._full("GetStatusLine")
        return StatusText(str(r.get("ReturnValue", "") or ""), severity_name(r.get("OutSeverity", "Info")))

    def status(self) -> str:
        """The controller's own one-line status."""
        v = self._call(self.STATUS_FUNCTION)
        return "" if v is None else str(v)

    STATUS_FUNCTION = "GetStatus"

    def is_active(self, phase: Optional[str] = None) -> bool:
        """A crossing is under way (not Idle and not finished)."""
        ph = self.phase() if phase is None else phase
        return ph != "Idle" and ph not in self.TERMINAL

    # ── assist ──────────────────────────────────────────────────────

    @property
    def assist(self) -> bool:
        """Curb assist (curb.assist): pushing forward at a curb starts a crossing."""
        if not self.surface.has("curb.assist"):
            return False
        return self.surface.get_enum("curb.assist").lower() == "on"

    def set_assist(self, on: bool) -> bool:
        if not self.surface.has("curb.assist"):
            raise CurbError(f"{self.surface.robot_name} has no curb.assist control")
        return self.surface.set_enum("curb.assist", "On" if on else "Off")

    @assist.setter
    def assist(self, on: bool) -> None:
        self.set_assist(on)

    # ── commands ────────────────────────────────────────────────────

    def start(self) -> CurbStart:
        raise NotImplementedError

    def negotiate(self) -> CurbStart:
        """Same as start(): cross the curb ahead."""
        return self.start()

    def abort(self, reason: str = "Stopped over Remote Control") -> None:
        raise NotImplementedError

    def stats(self) -> dict:
        """How the current or last crossing went (the controller's stats struct, as a dict)."""
        return dict(self._call(self.STATS_FUNCTION) or {})

    def detect(self) -> dict:
        """What the robot can say about the curb ahead (see each controller)."""
        raise NotImplementedError

    def snapshot(self) -> dict:
        """Phase, status and the key live readings, for a watch line."""
        st = self.status_line()
        ph = self.phase()
        return {"phase": ph, "label": self.phase_label(ph), "status": st.text, "severity": st.severity}

    # ── waiting ─────────────────────────────────────────────────────

    def _retry_grace(self, phase: str) -> float:
        return 0.0

    def wait_until_done(self, timeout: float = 180.0, poll: float = 0.2, start_grace: float = 3.0,
                        on_event: Optional[Callable[[CurbEvent], None]] = None,
                        should_stop: Optional[Callable[[], bool]] = None,
                        hold_forward: Optional[float] = None, source: Optional[str] = None) -> CurbRun:
        """
        Poll until the crossing ends (Done, Aborted, Faulted -- for Gen2, a fault it retries by itself is waited
        out), or it never starts within start_grace seconds, or `timeout`. on_event(CurbEvent) is called for
        every change of phase or status line. Returns a CurbRun with the log and the final stats.

        hold_forward: hold the rider's stick (drive.forward) at this value at 10 Hz while -- and only while -- the
        crossing is under way: released the moment a poll sees it not active (Idle, Done, Aborted, Faulted), and
        held again when it is active again (a Gen2 retry). Polls at <= 0.1 s while holding. Released at the end,
        on an error and on an interrupt. `source` is the ERammsControlSource of the hold (the surface's default).
        """
        ka: Optional[KeepAlive] = None
        if hold_forward is not None:
            ka = self.surface.keep_alive({"drive.forward": float(hold_forward)}, source=source)
            ka.pause(release=False)          # held from the first poll that sees the crossing under way
            ka.start()
            poll = min(poll, 0.1)
        t0 = time.monotonic()
        events: list[CurbEvent] = []
        last = None
        started = False
        settled_at: Optional[float] = None
        timed_out = False
        try:
            while True:
                now = time.monotonic() - t0
                ph = self.phase()
                active = self.is_active(ph)
                if ka is not None:
                    if active and ka.paused:
                        ka.resume()
                    elif not active and not ka.paused:
                        ka.pause(release=True)       # before anything else: the robot may be handed back now
                st = self.status_line()
                key = (ph, st.text, st.severity)
                if key != last:
                    last = key
                    ev = CurbEvent(now, ph, self.phase_label(ph), st.text, st.severity)
                    events.append(ev)
                    if on_event is not None:
                        on_event(ev)
                if active:
                    started, settled_at = True, None
                elif started:
                    # Finished (or back to Idle). A Gen2 fault before any wheel crossed is retried by itself once it
                    # stands at rest: give it the time to.
                    settled_at = now if settled_at is None else settled_at
                    if now - settled_at >= self._retry_grace(ph):
                        break
                elif now > start_grace:
                    break           # never started (still Idle, or the last crossing's Done)
                if now > timeout:
                    timed_out = True
                    break
                if should_stop is not None and should_stop():
                    break
                time.sleep(poll)
        finally:
            if ka is not None:
                ka.stop()
        try:
            stats = self.stats()
        except Exception:
            stats = {}
        return CurbRun(started, last[0] if last else "", StatusText(last[1], last[2]) if last else StatusText(""),
                       time.monotonic() - t0, events, stats, timed_out,
                       stick_refused=ka.refused.get("drive.forward", 0) if ka is not None else 0)

    def cross(self, forward: float = DEFAULT_CROSSING_STICK, timeout: float = 180.0, poll: float = 0.2,
              start_grace: float = 3.0, on_event: Optional[Callable[[CurbEvent], None]] = None,
              should_stop: Optional[Callable[[], bool]] = None, source: Optional[str] = None) -> CurbRun:
        """
        Cross the curb ahead: start() (the controller selects the curb drive mode itself), then wait_until_done()
        holding the stick forward at `forward` only while the crossing is under way. A start refused is a CurbRun
        with started False and the reason. A crossing already under way is joined rather than started again.
        """
        if not self.is_active():
            ok, reason = self.start()
            if not ok:
                time.sleep(0.3)             # a refusal can be the start already made by a mode switch
                if not self.is_active():
                    st = self.status_line()
                    return CurbRun(False, self.phase(), st, 0.0, reason=reason or st.text)
        return self.wait_until_done(timeout=timeout, poll=poll, start_grace=start_grace, on_event=on_event,
                                    should_stop=should_stop, hold_forward=forward, source=source)


# ── RAMMP Gen1 ─────────────────────────────────────────────────────────


@dataclass
class Gen1Crossing:
    """GetCrossing: the step being crossed and the chassis height it crosses at (0 before a crossing)."""
    step_cm: float = 0.0
    crossing_height_cm: float = 0.0


class Gen1Curb(CurbBase):
    """The RAMMP Gen1 curb controller (URammsRammpGen1CurbController)."""

    kind = "gen1"
    CLASS = "RammsRammpGen1CurbController"
    INSTANCE = "CurbController"
    STATS_FUNCTION = "GetStats"
    STATUS_FUNCTION = "GetStatus"
    PHASE_NAMES = ("Idle", "Level", "Approach", "FrontUp", "ToEdge", "FrontDown", "DriveUp", "DriveOver", "DriveDown",
                   "PullAft", "RearUp", "RearOver", "Lower", "RearDown", "ClearEdge", "Recentre", "Restore", "Done",
                   "Aborted")
    # The crossing styles' C++ names, in curb.style's order ("Stop and step", "Rolling").
    STYLE_NAMES = ("Stepped", "Blended")

    def start(self) -> CurbStart:
        """StartCrossing: begin crossing the curb ahead."""
        r = self._full("StartCrossing")
        return CurbStart(as_bool(r.get("ReturnValue", False)), str(r.get("OutReason", "") or ""))

    def abort(self, reason: str = "Stopped over Remote Control") -> None:
        """Stop where it is: the lifts hold, the drive wheels brake; it hands back once the drive wheels are down."""
        self.proxy.call("Abort", Reason=reason)

    def crossing(self) -> Gen1Crossing:
        r = self._full("GetCrossing")
        return Gen1Crossing(as_float(r.get("OutStepCm")), as_float(r.get("OutCrossingHeightCm")))

    @property
    def style(self) -> str:
        """The crossing style as curb.style shows it ("Stop and step" or "Rolling")."""
        if not self.surface.has("curb.style"):
            raise CurbError(f"{self.surface.robot_name} has no curb.style control")
        return self.surface.get_enum("curb.style")

    def set_style(self, style: str) -> bool:
        """Set the style by label ("Rolling"), C++ name ("Blended", "Stepped"), "step"/"roll", or index."""
        if not self.surface.has("curb.style"):
            raise CurbError(f"{self.surface.robot_name} has no curb.style control")
        alias = {"stepped": 0, "step": 0, "stop": 0, "blended": 1, "rolling": 1, "roll": 1}
        a = self.surface.axis("curb.style")
        key = str(style).strip().lower()
        if key in alias and len(a.enum_labels) == len(self.STYLE_NAMES):
            return self.surface.set_enum("curb.style", alias[key])
        return self.surface.set_enum("curb.style", style)

    @style.setter
    def style(self, style: str) -> None:
        self.set_style(style)

    def detect(self) -> dict:
        """RAMMP Gen1 has no look-ahead step detection: it finds the step as it crosses. Reports what it knows."""
        c = self.crossing()
        return {"available": False,
                "note": "RAMMP Gen1 has no look-ahead step detection; the step is found during the crossing",
                "step_cm": c.step_cm, "crossing_height_cm": c.crossing_height_cm, "status": self.status_line().text}

    def snapshot(self) -> dict:
        s = super().snapshot()
        c = self.crossing()
        s.update(step_cm=c.step_cm, crossing_height_cm=c.crossing_height_cm)
        return s


# ── RAMMP Gen2 (lift-drive) ────────────────────────────────────────────


def _xyz(v: tuple[float, float, float]) -> dict:
    return {"X": float(v[0]), "Y": float(v[1]), "Z": float(v[2])}


def _xy(v: tuple[float, float]) -> dict:
    return {"X": float(v[0]), "Y": float(v[1])}


@dataclass
class CurbPlane:
    """A fitted ground plane (near or far side of the edge)."""
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gradient: tuple[float, float] = (0.0, 0.0)
    fit_rms_cm: float = 0.0
    fit_max_cm: float = 0.0
    samples: int = 0
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: Optional[dict]) -> "CurbPlane":
        d = d or {}
        return cls(vec3(d.get("Origin")), vec2(d.get("Gradient")), as_float(d.get("FitRmsCm")),
                   as_float(d.get("FitMaxCm")), int(as_float(d.get("Samples"))), raw=d)

    def to_json(self) -> dict:
        """The FRammsCurbPlane to send: the fields as they are now, over what was read (fields not modelled kept)."""
        return {**self.raw, "Origin": _xyz(self.origin), "Gradient": _xy(self.gradient), "FitRmsCm": float(self.fit_rms_cm),
                "FitMaxCm": float(self.fit_max_cm), "Samples": int(self.samples)}


@dataclass
class CurbTerrain:
    """The terrain model the Gen2 crossing senses or plans on (FRammsCurbTerrain)."""
    valid: bool = False
    edge_point: tuple[float, float, float] = (0.0, 0.0, 0.0)
    edge_direction: tuple[float, float] = (0.0, 1.0)
    edge_normal: tuple[float, float] = (1.0, 0.0)
    approach_angle_deg: float = 0.0
    edge_fit_residual_cm: float = 0.0
    near: CurbPlane = field(default_factory=CurbPlane)
    far: CurbPlane = field(default_factory=CurbPlane)
    step_height_cm: float = 0.0
    step_height_range_cm: tuple[float, float] = (0.0, 0.0)
    far_depth_cm: float = 0.0
    far_depth_observed: bool = False
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: Optional[dict]) -> "CurbTerrain":
        d = d or {}
        return cls(
            valid=as_bool(d.get("bValid", False)), edge_point=vec3(d.get("EdgePoint")),
            edge_direction=vec2(d.get("EdgeDirection", {"X": 0, "Y": 1})), edge_normal=vec2(d.get("EdgeNormal", {"X": 1, "Y": 0})),
            approach_angle_deg=as_float(d.get("ApproachAngleDeg")), edge_fit_residual_cm=as_float(d.get("EdgeFitResidualCm")),
            near=CurbPlane.from_json(d.get("Near")), far=CurbPlane.from_json(d.get("Far")),
            step_height_cm=as_float(d.get("StepHeightCm")), step_height_range_cm=vec2(d.get("StepHeightRangeCm")),
            far_depth_cm=as_float(d.get("FarDepthCm")), far_depth_observed=as_bool(d.get("bFarDepthObserved", False)),
            raw=d)

    def to_json(self) -> dict:
        """The FRammsCurbTerrain to send: the fields as they are now (an edited detection is sent as edited)."""
        return {**self.raw, "bValid": bool(self.valid), "EdgePoint": _xyz(self.edge_point),
                "EdgeDirection": _xy(self.edge_direction), "EdgeNormal": _xy(self.edge_normal),
                "ApproachAngleDeg": float(self.approach_angle_deg), "EdgeFitResidualCm": float(self.edge_fit_residual_cm),
                "Near": self.near.to_json(), "Far": self.far.to_json(), "StepHeightCm": float(self.step_height_cm),
                "StepHeightRangeCm": _xy(self.step_height_range_cm), "FarDepthCm": float(self.far_depth_cm),
                "bFarDepthObserved": bool(self.far_depth_observed)}


@dataclass
class CurbProfile:
    """A step to cross (FRammsCurbProfile): height (positive up), distance to the edge, skew, approach angle."""
    step_height_cm: float = 0.0
    edge_distance_cm: float = 0.0
    edge_skew_cm: float = 0.0
    approach_angle_deg: float = 0.0
    terrain: CurbTerrain = field(default_factory=CurbTerrain)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: Optional[dict]) -> "CurbProfile":
        d = d or {}
        return cls(as_float(d.get("StepHeightCm")), as_float(d.get("EdgeDistanceCm")), as_float(d.get("EdgeSkewCm")),
                   as_float(d.get("ApproachAngleDeg")), CurbTerrain.from_json(d.get("Terrain")), raw=d)

    def to_json(self) -> dict:
        """The FRammsCurbProfile to send: the fields as they are now (a built or edited profile is sent as it is)."""
        return {**self.raw, "StepHeightCm": float(self.step_height_cm), "EdgeDistanceCm": float(self.edge_distance_cm),
                "EdgeSkewCm": float(self.edge_skew_cm), "ApproachAngleDeg": float(self.approach_angle_deg),
                "Terrain": self.terrain.to_json()}

    @property
    def direction(self) -> str:
        return "up" if self.step_height_cm >= 0 else "down"


@dataclass
class Detection:
    """What DetectStepAhead / DetectTerrainAhead found: whether there is a step to cross, and the reasoning."""
    found: bool
    reason: str = ""
    profile: Optional[CurbProfile] = None
    terrain: Optional[CurbTerrain] = None
    raw: dict = field(default_factory=dict, repr=False)


class Gen2Curb(CurbBase):
    """The RAMMP Gen2 (lift-drive) curb negotiation controller (URammsCurbNegotiationController)."""

    kind = "gen2"
    CLASS = "RammsCurbNegotiationController"
    INSTANCE = "CurbNegotiation"
    STATS_FUNCTION = "GetRunStats"
    STATUS_FUNCTION = "GetStatusText"
    PHASE_NAMES = ("Idle", "Approach", "Prepare", "Lift", "Shift", "Advance", "Plant", "Settle", "Done", "Aborted",
                   "Faulted", "Cross", "Hold", "Paused")
    TERMINAL = frozenset({"Done", "Aborted", "Faulted"})
    # A fault before any wheel crossed is retried once by itself after half a second at rest.
    RETRY_GRACE_SECONDS = 3.0

    def _retry_grace(self, phase: str) -> float:
        return self.RETRY_GRACE_SECONDS if phase == "Faulted" else 0.0

    def is_negotiating(self) -> bool:
        return as_bool(self._call("IsNegotiating"))

    def start(self) -> CurbStart:
        """NegotiateStepAhead: detect the step ahead and cross it."""
        r = self._full("NegotiateStepAhead")
        return CurbStart(as_bool(r.get("ReturnValue", False)), str(r.get("OutReason", "") or ""))

    def check_feasibility(self, profile: CurbProfile) -> CurbStart:
        """Plan a crossing of `profile` (as it is now: edits are sent) without moving: can it be done, and why not."""
        r = self._full("CheckFeasibility", Profile=profile.to_json())
        return CurbStart(as_bool(r.get("ReturnValue", False)), str(r.get("OutReason", "") or ""))

    def begin(self, profile: CurbProfile) -> CurbStart:
        """BeginNegotiation on a given profile (e.g. a detected one, edited: the fields as they are now are sent)."""
        r = self._full("BeginNegotiation", Profile=profile.to_json())
        return CurbStart(as_bool(r.get("ReturnValue", False)), str(r.get("OutReason", "") or ""))

    def begin_on_terrain(self, terrain: CurbTerrain) -> CurbStart:
        """BeginNegotiationOnTerrain: cross on a given terrain model (not detected: no automatic retry)."""
        r = self._full("BeginNegotiationOnTerrain", Terrain=terrain.to_json())
        return CurbStart(as_bool(r.get("ReturnValue", False)), str(r.get("OutReason", "") or ""))

    def abort(self, reason: str = "") -> None:
        """Stop: the robot holds where it is (Abort takes no reason on Gen2)."""
        self.proxy.call("Abort")

    # ── sensing ─────────────────────────────────────────────────────

    def detect_step_ahead(self) -> Detection:
        r = self._full("DetectStepAhead")
        prof = CurbProfile.from_json(r.get("OutProfile"))
        return Detection(as_bool(r.get("ReturnValue", False)), str(r.get("OutReason", "") or ""), prof, prof.terrain, raw=r)

    def detect_terrain_ahead(self) -> Detection:
        r = self._full("DetectTerrainAhead")
        return Detection(as_bool(r.get("ReturnValue", False)), str(r.get("OutReason", "") or ""), None,
                         CurbTerrain.from_json(r.get("OutTerrain")), raw=r)

    def detection_text(self) -> str:
        v = self._call("GetDetectionText")
        return "" if v is None else str(v)

    def active_terrain(self) -> CurbTerrain:
        return CurbTerrain.from_json(self._call("GetActiveTerrain"))

    def active_profile(self) -> CurbProfile:
        return CurbProfile.from_json(self._call("GetActiveProfile"))

    def detect(self) -> dict:
        step = self.detect_step_ahead()
        terrain = self.detect_terrain_ahead()
        p = step.profile or CurbProfile()
        return {"available": True, "step_found": step.found, "reason": step.reason,
                "step_height_cm": p.step_height_cm, "edge_distance_cm": p.edge_distance_cm,
                "edge_skew_cm": p.edge_skew_cm, "approach_angle_deg": p.approach_angle_deg,
                "terrain_found": terrain.found, "terrain_valid": bool(terrain.terrain and terrain.terrain.valid),
                "far_depth_cm": terrain.terrain.far_depth_cm if terrain.terrain else 0.0,
                "detection_text": self.detection_text()}

    # ── live measurements ───────────────────────────────────────────

    def phase_axle(self) -> str:
        return enum_name(self._call("GetPhaseAxle"))

    def elapsed(self) -> float:
        return as_float(self._call("GetElapsedSeconds"))

    def measured_tilt(self) -> tuple[float, float]:
        """(pitch nose up, roll +Y side up), degrees."""
        return vec2(self._call("GetMeasuredTiltDeg"))

    def planned_tilt(self) -> tuple[float, float]:
        """The largest base tilt the last per-wheel plan calls for: (pitch, roll), degrees."""
        return vec2(self._call("GetPlannedTiltDeg"))

    def base_height(self) -> tuple[float, float]:
        """(measured, commanded) chassis origin height over the planned ground, cm."""
        return vec2(self._call("GetBaseHeight"))

    def drive_speed(self) -> tuple[float, float]:
        """(commanded, measured) ground speed along the heading, cm/s."""
        return vec2(self._call("GetDriveSpeed"))

    def wheel_progress(self) -> list[float]:
        """Each leg's wheel position past the step edge (cm), in leg order."""
        return [as_float(v) for v in (self._call("GetWheelProgress") or [])]

    def leg_heights(self) -> list[tuple[float, float]]:
        """Each leg's wheel-centre height in the chassis frame: (commanded, measured) cm."""
        return [vec2(v) for v in (self._call("GetLegHeights") or [])]

    def wheel_clearance(self) -> list[float]:
        """Each wheel's bottom above the surface it follows (cm)."""
        return [as_float(v) for v in (self._call("GetWheelClearance") or [])]

    def wheel_status(self) -> list[dict]:
        """Each wheel's part in the crossing (FRammsCurbWheelStatus as dicts: Leg, State, Plane, PastEdgeCm ...)."""
        return [dict(v) for v in (self._call("GetWheelStatus") or []) if isinstance(v, dict)]

    def stability_margin(self) -> float:
        """How far inside its support polygon the centre of mass stands (cm; negative outside)."""
        return as_float(self._call("GetStabilityMargin"))

    def plan(self) -> list[str]:
        """The planned steps, one line each."""
        return [str(x) for x in (self._call("GetPlanDescription") or [])]

    def step_log(self) -> list[str]:
        return [str(x) for x in (self._call("GetStepLog") or [])]

    def schedule(self) -> list[str]:
        """The per-wheel gait's schedule lines, then its summary."""
        return [str(x) for x in (self._call("GetScheduleLines") or [])]

    def run_stats(self) -> dict:
        return dict(self._call("GetRunStats") or {})

    def snapshot(self) -> dict:
        s = super().snapshot()
        pitch, roll = self.measured_tilt()
        meas_h, cmd_h = self.base_height()
        cmd_v, meas_v = self.drive_speed()
        s.update(axle=self.phase_axle(), pitch_deg=pitch, roll_deg=roll, base_height_cm=meas_h,
                 base_height_cmd_cm=cmd_h, speed_cmd_cm_s=cmd_v, speed_cm_s=meas_v,
                 margin_cm=self.stability_margin(), elapsed_s=self.elapsed())
        return s


CURB_CLASSES = (Gen1Curb, Gen2Curb)
