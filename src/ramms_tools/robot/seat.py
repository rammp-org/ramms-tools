"""
The RAMMP Gen2 (lift-drive) seat: a 5-bar that walks the seat platform's
pivot around a reachable region in the chassis' fore/aft-height plane, and a
tilt actuator (URammsSeatMechanismController, "Seat").

    seat = robot.seat
    print(seat.pivot(), seat.tilt())          # (-18.0, 39.5) cm, 1.1 deg
    if seat.contains(0.0, 45.0):
        seat.set_pivot(0.0, 45.0)              # fore/aft, height (cm, chassis x-z)
    seat.set_tilt(-5.0)                         # nose down 5 deg
    seat.reset()

The same mechanism is also on the control surface as seat.height /
seat.fore_aft (Position, paired; seat.height carries the reachable outline),
seat.tilt, seat.jog_up / seat.jog_fore_aft and seat.reset.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ramms_tools._rc import as_bool, as_float, return_value, vec2
from ramms_tools.control import ControlSurface
from ramms_tools.unreal_remote import RemoteObjectProxy

SEAT_CLASS = "RammsSeatMechanismController"


@dataclass
class SeatState:
    pivot: Optional[tuple[float, float]]    # (fore/aft, height) cm, chassis x-z; None when unknown
    tilt_deg: Optional[float]               # platform pitch relative to the chassis, nose up positive


def point_in_polygon(x: float, y: float, poly: list[tuple[float, float]]) -> bool:
    """Even-odd test; the polygon is closed implicitly."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


class Seat:
    """The Gen2 seat mechanism controller."""

    CLASS = SEAT_CLASS
    INSTANCE = "Seat"

    def __init__(self, surface: ControlSurface, component_path: str):
        self.surface = surface
        self.component_path = component_path
        self.proxy = RemoteObjectProxy(surface.ue, component_path)

    def __repr__(self) -> str:
        return f"Seat({self.component_path!r})"

    def pivot(self) -> Optional[tuple[float, float]]:
        """The platform pivot where the motors stand: (fore/aft, height) cm in the chassis x-z plane."""
        r = self.proxy.call_full("GetPivot")
        return vec2(r.get("ReturnValue")) if as_bool(r.get("bValid", False)) else None

    def tilt(self) -> Optional[float]:
        """The platform's pitch relative to the chassis (deg, nose up positive)."""
        r = self.proxy.call_full("GetTiltDeg")
        return as_float(r.get("ReturnValue")) if as_bool(r.get("bValid", False)) else None

    def outline(self) -> list[tuple[float, float]]:
        """The pivot's reachable region, a closed polygon of (fore/aft, height) cm; [] when nothing is reachable."""
        r = self.proxy.call_full("GetReachableOutline")
        if not as_bool(r.get("bValid", False)):
            return []
        return [vec2(p) for p in (r.get("ReturnValue") or [])]

    def contains(self, fore_aft_cm: float, height_cm: float) -> bool:
        """Is the point inside the reachable outline?"""
        poly = self.outline()
        return bool(poly) and point_in_polygon(fore_aft_cm, height_cm, poly)

    def state(self) -> SeatState:
        return SeatState(self.pivot(), self.tilt())

    def set_pivot(self, fore_aft_cm: float, height_cm: float) -> bool:
        """Walk the pivot there, holding the tilt. False (nothing changed) when out of reach or busy."""
        return as_bool(return_value(self.proxy.call("SetPivotTarget", TargetXZ={"X": float(fore_aft_cm), "Y": float(height_cm)})))

    def set_tilt(self, deg: float) -> bool:
        """Turn the platform to `deg` relative to the chassis (clamped to its limit), the pivot where it is."""
        return as_bool(return_value(self.proxy.call("SetTiltDeg", Deg=float(deg))))

    def reset(self) -> bool:
        """Back to the rest pivot and tilt."""
        return as_bool(return_value(self.proxy.call("ResetSeat")))

    # Through the control surface (Remote source: these hold over local input while commanded).

    def set_height(self, cm: float) -> bool:
        return self.surface.set("seat.height", cm)

    def set_fore_aft(self, cm: float) -> bool:
        return self.surface.set("seat.fore_aft", cm)
