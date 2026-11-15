"""
A robot in the sim, found by name (or the first one in the world), with its
control surface, drive modes, curb crossing, seat and sensors.

    from ramms_tools.robot import Robot

    robot = Robot.find()                        # first robot with a control surface
    print(robot.name, robot.generation)         # "BP_RammpGen1PoweredFront_Ramms_C_0", "gen1"

    print(robot.modes())                        # [DriveMode(id='curb', ...), DriveMode(id='differential', ..., active=True)]
    robot.set_mode("holonomic")                 # waits for the hand-over; ModeError if refused
    robot.drive(forward=0.3, strafe=0.2, seconds=2)

    curb = robot.curb                           # Gen1Curb or Gen2Curb (None when the robot has no curb controller)
    print(curb.phase(), curb.status_line())     # "Idle", StatusText(text='Ready', severity='Info')
    run = curb.cross(forward=0.5, on_event=print)   # start (the controller selects the curb mode), hold the stick
    print(run.succeeded, run.stats)             # forward while the crossing runs, release, CurbRun

A crossing moves only while the rider's stick (drive.forward) is held forward;
``cross()`` holds it for you, only while the crossing is under way. Do not
select the ``curb`` drive mode by hand to start one: selecting it starts (or,
with nothing to cross, refuses and hands back) a crossing by itself.

    if robot.seat:                              # RAMMP Gen2 only
        robot.seat.set_tilt(-5)

    for s in robot.sensors.list():              # ramms_tools.sensing, bound to this robot
        print(s.name, s.kind)

Every object keeps its raw Remote Control responses reachable (``.raw`` on the
dataclasses, ``.proxy`` / ``call_full`` on the controllers). The RAMMP Gen2
(lift-drive) robot is only in levels built with the private assets; everything
here works the same on whichever robot is in the world.
"""

from __future__ import annotations

import logging
from typing import Any, Optional, Union

from ramms_tools._rc import short_name
from ramms_tools.control import ControlError, ControlSurface, StatusText
from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError

from .curb import (CURB_CLASSES, CurbBase, CurbError, CurbEvent, CurbPlane, CurbProfile, CurbRun, CurbStart,
                   CurbTerrain, Detection, Gen1Crossing, Gen1Curb, Gen2Curb)
from .modes import SELECTOR_CLASS, DriveMode, DriveModes, DriveMotion, ModeError
from .seat import SEAT_CLASS, Seat, SeatState

logger = logging.getLogger(__name__)

__all__ = [
    "Robot", "RobotNotFound", "DriveModes", "DriveMode", "DriveMotion", "ModeError",
    "CurbBase", "Gen1Curb", "Gen2Curb", "CurbStart", "CurbRun", "CurbEvent", "CurbError", "CurbProfile",
    "CurbTerrain", "CurbPlane", "Detection", "Gen1Crossing", "Seat", "SeatState", "StatusText",
]


class RobotNotFound(ControlError):
    """No robot with a control surface (matching the name) in the world."""


class Robot:
    """
    One robot pawn: its control surface plus the controller components beside it.

    Args:
        surface: the robot's ControlSurface (Robot.find / Robot.all build it for you).
    """

    def __init__(self, surface: ControlSurface):
        self.surface = surface
        self.ue: UnrealRemote = surface.ue
        self.actor_path = surface.actor_path
        self._components: Optional[list[dict]] = None
        self._modes: Optional[DriveModes] = None
        self._curb: Any = False          # False = not looked up yet; None = none
        self._seat: Any = False
        self._sensors = None

    def __repr__(self) -> str:
        return f"Robot({self.actor_path!r})"

    # ── discovery ───────────────────────────────────────────────────

    @classmethod
    def all(cls, ue: Optional[UnrealRemote] = None, host: str = "127.0.0.1", port: int = 30010,
            timeout: float = 10.0, source: str = "Remote") -> list["Robot"]:
        """Every robot with a control surface in the world (play world when one runs, else the editor's)."""
        ue = ue or UnrealRemote(host=host, http_port=port, timeout=timeout)
        return [cls(s) for s in ControlSurface.find(ue, source=source)]

    @classmethod
    def find(cls, name: str = "", ue: Optional[UnrealRemote] = None, host: str = "127.0.0.1", port: int = 30010,
             timeout: float = 10.0, source: str = "Remote") -> "Robot":
        """A robot by actor name / path or robotName (exact, then a case-insensitive substring); "" = the first
        one in the world. Raises RobotNotFound, or ControlError when a substring matches several robots."""
        ue = ue or UnrealRemote(host=host, http_port=port, timeout=timeout)
        if name:
            s = ControlSurface.find_by_robot(ue, name, source=source)
            if s is None:
                names = [short_name(x.actor_path) for x in ControlSurface.find(ue, source=source)]
                raise RobotNotFound(f"no robot matching {name!r} (robots: {', '.join(names) or 'none'})")
            return cls(s)
        surfaces = ControlSurface.find(ue, source=source)
        if not surfaces:
            raise RobotNotFound("no robot with a control surface in the world (is PIE / the game running?)")
        return cls(surfaces[0])

    @property
    def name(self) -> str:
        """The actor's name (the surface's robotName when it has one)."""
        try:
            return self.surface.robot_name
        except UnrealRemoteError:
            return short_name(self.actor_path)

    def components(self, refresh: bool = False) -> list[dict]:
        """The pawn's components: [{"name", "class_name", "path"}] (RammsCoreBridge.FindComponents)."""
        if self._components is None or refresh:
            self._components = self.ue.find_components(self.actor_path, "")
        return self._components

    def find_component(self, class_name: str, instance: str = "") -> Optional[str]:
        """
        The object path of the pawn's component of a class (a Blueprint subclass counts), else one named
        `instance`. Looks on the pawn first, then in the world for one whose actor is this pawn -- or, when the
        world holds exactly one, that one (a controller on an actor the pawn carries).
        """
        comps = self.components()
        for c in comps:
            if str(c.get("class_name", "")).endswith(class_name):
                return c["path"]
        if instance:
            for c in comps:
                if str(c.get("name", "")).lower() == instance.lower():
                    return c["path"]
        entries = [e for e in self.ue.find_actors_by_component(class_name)
                   if str(e.get("component_class", "")).endswith(class_name)]
        mine = [e for e in entries if e["actor_path"] == self.actor_path]
        if mine:
            return mine[0]["component_path"]
        if len(entries) == 1 and len(ControlSurface.find(self.ue)) <= 1:
            return entries[0]["component_path"]
        return None

    # ── driving ─────────────────────────────────────────────────────

    def drive(self, forward: float = 0.0, turn: float = 0.0, strafe: Optional[float] = None,
              seconds: Optional[float] = None, rate: float = 10.0, once: bool = False) -> bool:
        """See ControlSurface.drive: normalized commands, held at `rate` Hz for `seconds` then released; once=True
        (no seconds) sends a single command, which an older sim latches until stop()."""
        return self.surface.drive(forward, turn, strafe, seconds=seconds, rate=rate, once=once)

    def stop(self) -> None:
        """Release the drive controls (forward, turn and strafe)."""
        self.surface.stop()

    # ── drive modes ─────────────────────────────────────────────────

    @property
    def drive_modes(self) -> DriveModes:
        if self._modes is None:
            self._modes = DriveModes(self.surface, self.find_component(SELECTOR_CLASS, "DriveModeSelector"))
        return self._modes

    def modes(self) -> list[DriveMode]:
        return self.drive_modes.modes()

    @property
    def mode(self) -> str:
        """The active drive mode's id."""
        return self.drive_modes.active()

    def set_mode(self, mode: Union[str, int], wait: bool = True, timeout: float = 15.0, on_status=None) -> str:
        """Switch drive mode by id, label or index (see DriveModes.set). Returns the switch status line."""
        return self.drive_modes.set(mode, wait=wait, timeout=timeout, on_status=on_status)

    def cycle_mode(self, wait: bool = False) -> str:
        return self.drive_modes.cycle(wait=wait)

    def measured_motion(self) -> Optional[DriveMotion]:
        return self.drive_modes.measured_motion()

    # ── curb, seat, sensors ─────────────────────────────────────────

    @property
    def curb(self) -> Optional[CurbBase]:
        """The curb controller: Gen1Curb (RAMMP Gen1) or Gen2Curb (RAMMP Gen2, lift-drive); None without one."""
        if self._curb is False:
            self._curb = None
            for cls in CURB_CLASSES:
                path = self.find_component(cls.CLASS)
                if path:
                    self._curb = cls(self.surface, path)
                    break
        return self._curb

    @property
    def generation(self) -> str:
        """"gen1" or "gen2" by its curb controller, "" for any other robot."""
        c = self.curb
        return c.kind if c is not None else ""

    @property
    def seat(self) -> Optional[Seat]:
        """The Gen2 seat mechanism (None on a robot without one)."""
        if self._seat is False:
            path = self.find_component(SEAT_CLASS)
            self._seat = Seat(self.surface, path) if path else None
        return self._seat

    @property
    def sensors(self):
        """This robot's sensors (ramms_tools.sensing.ActorSensors): list / add / remove / read / apply_rig."""
        if self._sensors is None:
            from ramms_tools.sensing import ActorSensors, Sensing
            self._sensors = ActorSensors(Sensing(self.ue), self.actor_path)
        return self._sensors

    def status(self) -> dict:
        """A one-call summary: name, drive mode, curb phase and status line (when it has a curb controller)."""
        out: dict[str, Any] = {"robot": self.name, "actor": self.actor_path, "generation": self.generation}
        try:
            out["mode"] = self.mode
        except UnrealRemoteError as e:
            out["mode"] = f"? ({e})"
        if self.curb is not None:
            st = self.curb.status_line()
            out.update(phase=self.curb.phase(), status=st.text, severity=st.severity)
        return out
