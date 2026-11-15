"""
Drive modes: the robot's URammsDriveModeSelector ("DriveModeSelector"), or its
``drive.mode`` Enum control when it has no selector.

RAMMP Gen1 offers ``curb`` and ``differential``; RAMMP Gen2 (lift-drive)
``curb``, ``differential``, ``holonomic`` and ``low_level``. A switch is not
instant: the outgoing mode brings any motion the incoming one cannot carry to
zero and the stance changes (wheels down / up) before the hand-over, or the
switch is refused after the selector's hand-over timeout. ``set()`` with
wait=True polls ``IsSwitchingMode`` / ``GetActiveModeId`` until it lands and
raises ModeError, with the selector's ``GetSwitchStatus`` line, when refused.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Union

from ramms_tools._rc import as_bool, as_float, name as _fname, return_value
from ramms_tools.control import ControlError, ControlSurface, _norm
from ramms_tools.unreal_remote import RemoteObjectProxy

SELECTOR_CLASS = "RammsDriveModeSelector"
MODE_CONTROL = "drive.mode"


class ModeError(ControlError):
    """No such drive mode, or the switch was refused / did not finish."""


@dataclass
class DriveMode:
    id: str                 # "curb", "differential", "holonomic", "low_level"
    label: str              # as the drive.mode control shows it: "Curb crossing", "Holonomic", "Low level"
    active: bool = False

    def __str__(self) -> str:
        return f"{self.id} ({self.label})" + (" *" if self.active else "")


@dataclass
class DriveMotion:
    """The chassis' motion as the selector measures it (FRammsDriveMotion)."""
    forward_cm_s: float = 0.0
    right_cm_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "DriveMotion":
        d = d or {}
        return cls(as_float(d.get("ForwardCmPerSecond")), as_float(d.get("RightCmPerSecond")),
                   as_float(d.get("YawRateDegPerSecond")), raw=d)

    @property
    def speed_cm_s(self) -> float:
        return math.hypot(self.forward_cm_s, self.right_cm_s)


def _label_from_id(mode_id: str) -> str:
    s = mode_id.replace("_", " ").strip()
    return s[:1].upper() + s[1:]


class DriveModes:
    """
    A robot's drive modes.

    Args:
        surface: the robot's control surface (for the drive.mode labels, and as the fallback).
        selector_path: the DriveModeSelector component's object path, or None when the robot has none.
    """

    def __init__(self, surface: ControlSurface, selector_path: Optional[str] = None):
        self.surface = surface
        self.selector_path = selector_path
        self._selector = RemoteObjectProxy(surface.ue, selector_path) if selector_path else None

    def __repr__(self) -> str:
        return f"DriveModes({self.selector_path or MODE_CONTROL!r})"

    @property
    def has_selector(self) -> bool:
        return self._selector is not None

    @property
    def available(self) -> bool:
        return self.has_selector or self.surface.has(MODE_CONTROL)

    # ── listing ─────────────────────────────────────────────────────

    def _control_labels(self) -> list[str]:
        a = self.surface.describe().axis(MODE_CONTROL)
        return list(a.enum_labels) if a is not None else []

    def ids(self) -> list[str]:
        """Mode ids in the order the robot offers them."""
        if self._selector is not None:
            return [_fname(x) for x in (return_value(self._selector.call("GetAvailableModeIds")) or [])]
        return [_norm_id(lab) for lab in self._control_labels()]

    def labels(self) -> dict[str, str]:
        """id -> display label (the drive.mode control's labels, which come in the same order)."""
        ids = self.ids()
        labs = self._control_labels()
        if len(labs) != len(ids):
            labs = [_label_from_id(i) for i in ids]
        return dict(zip(ids, labs))

    def active(self) -> str:
        """The id of the mode active now (during a switch, the outgoing one until the hand-over)."""
        if self._selector is not None:
            return _fname(return_value(self._selector.call("GetActiveModeId")))
        if not self.surface.has(MODE_CONTROL):
            return ""
        return _norm_id(self.surface.get_enum(MODE_CONTROL))

    def modes(self) -> list[DriveMode]:
        active = self.active().lower()
        return [DriveMode(i, lab, i.lower() == active) for i, lab in self.labels().items()]

    def resolve(self, choice: Union[str, int]) -> str:
        """A mode id from an id, a label ("Curb crossing", "low level"), an index or a unique prefix."""
        labels = self.labels()
        ids = list(labels)
        if not ids:
            raise ModeError(f"{self.surface.robot_name} has no drive modes")
        if isinstance(choice, int) or (isinstance(choice, str) and choice.strip().isdigit()):
            i = int(choice)
            if 0 <= i < len(ids):
                return ids[i]
            raise ModeError(f"no drive mode {i} (0..{len(ids) - 1})")
        key = _norm(choice)
        for i in ids:
            if _norm(i) == key or _norm(labels[i]) == key:
                return i
        starts = [i for i in ids if key and (_norm(i).startswith(key) or _norm(labels[i]).startswith(key))]
        if len(starts) == 1:
            return starts[0]
        raise ModeError(f"no drive mode {choice!r} on {self.surface.robot_name} (modes: {', '.join(ids)})")

    # ── switching ───────────────────────────────────────────────────

    def is_switching(self) -> bool:
        return as_bool(return_value(self._selector.call("IsSwitchingMode"))) if self._selector is not None else False

    def switch_status(self) -> str:
        """One line on the switch in progress or the last one ("" without a selector)."""
        if self._selector is None:
            return ""
        v = return_value(self._selector.call("GetSwitchStatus"))
        return "" if v is None else str(v)

    def set(self, choice: Union[str, int], wait: bool = True, timeout: float = 15.0, poll: float = 0.1,
            on_status: Optional[Callable[[str], None]] = None) -> str:
        """
        Switch to a mode (id, label or index). With wait=True, block until it is active (and the stance has
        finished changing); raises ModeError when there is no such mode, the switch is refused, or it has not
        finished within `timeout`. Returns the selector's switch status line. The control surface's
        description is re-read afterwards, since the controls on offer depend on the mode (drive.strafe).
        """
        mode_id = self.resolve(choice)
        if self._selector is None:
            if not self.surface.has(MODE_CONTROL):
                raise ModeError(f"{self.surface.robot_name} has neither a drive-mode selector nor a {MODE_CONTROL} control")
            self.surface.set_enum(MODE_CONTROL, self.labels()[mode_id])
            if wait:
                self._wait(lambda: self.active().lower() == mode_id.lower(), timeout, poll, mode_id)
            self.surface.describe(refresh=True)
            return ""
        if not as_bool(return_value(self._selector.call("SetActiveMode", ModeId=mode_id))):
            raise ModeError(f"{self.surface.robot_name} refused drive mode {mode_id!r}: {self.switch_status() or 'no such mode'}")
        status = self.switch_status()
        if wait:
            status = self._wait_selector(mode_id, timeout, poll, on_status)
        self.surface.describe(refresh=True)
        return status

    def _wait(self, done: Callable[[], bool], timeout: float, poll: float, mode_id: str) -> None:
        t_end = time.monotonic() + timeout
        while not done():
            if time.monotonic() > t_end:
                raise ModeError(f"drive mode {mode_id!r} not active after {timeout:.0f} s")
            time.sleep(poll)

    def _wait_selector(self, mode_id: str, timeout: float, poll: float,
                       on_status: Optional[Callable[[str], None]]) -> str:
        t0 = time.monotonic()
        grace = min(0.5, timeout)      # a switch can take a tick to show as under way
        last = None
        while True:
            switching = self.is_switching()
            active = self.active()
            status = self.switch_status()
            if status != last:
                last = status
                if on_status is not None and status:
                    on_status(status)
            elapsed = time.monotonic() - t0
            if not switching and active.lower() == mode_id.lower():
                return status
            if not switching and elapsed > grace:
                raise ModeError(f"switch to {mode_id!r} refused (still {active!r}): {status}")
            if elapsed > timeout:
                raise ModeError(f"switch to {mode_id!r} not finished after {timeout:.0f} s: {status}")
            time.sleep(poll)

    def cycle(self, wait: bool = False, timeout: float = 15.0, poll: float = 0.1) -> str:
        """
        The next mode in the robot's cycle (the selector's CycleModeIds, which the sim does not report: the target is
        not known here until it is active). Returns the id of the mode active afterwards: without wait, the outgoing
        one while a switch is still waiting to hand over (is_switching()); with wait, the one it landed in -- and
        ModeError when the switch was refused (the robot still in the mode it started from) or did not finish within
        `timeout`.
        """
        before = self.active()
        if self._selector is not None:
            if not as_bool(return_value(self._selector.call("CycleMode"))):
                raise ModeError(f"no other mode to cycle to from {before!r}: {self.switch_status()}")
        elif self.surface.has(MODE_CONTROL):
            self.surface.trigger(MODE_CONTROL)
        else:
            raise ModeError(f"{self.surface.robot_name} has no drive modes")
        if wait:
            t0 = time.monotonic()
            grace = min(0.5, timeout)
            while True:
                switching, now = self.is_switching(), self.active()
                elapsed = time.monotonic() - t0
                if not switching and now.lower() != before.lower():
                    break
                if not switching and elapsed > grace:
                    raise ModeError(f"cycling from {before!r} refused: {self.switch_status() or 'still ' + repr(now)}")
                if elapsed > timeout:
                    raise ModeError(f"cycling from {before!r} not finished after {timeout:.0f} s: {self.switch_status()}")
                time.sleep(poll)
        self.surface.describe(refresh=True)
        return self.active()

    def measured_motion(self) -> Optional[DriveMotion]:
        """The chassis' measured motion (None without a selector)."""
        if self._selector is None:
            return None
        return DriveMotion.from_json(return_value(self._selector.call("GetMeasuredMotion")) or {})


def _norm_id(label: str) -> str:
    """A drive.mode label as an id when there is no selector to ask: "Low level" -> "low_level"."""
    return "_".join(str(label).lower().split())
