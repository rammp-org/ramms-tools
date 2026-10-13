"""
Ground-truth manifest produced by the RammsWorldGen plugin.

A generated world comes with a manifest that says where the pedestrian routes
are, how wide they really are after hazards were placed, and which features
(curb ramps, entrances, doors, thresholds, A-frames, ...) break the
accessibility profile. This module parses that JSON into plain dataclasses
and offers the queries an episode runner or dataset builder needs — no engine
connection required, so it also works on exported files.

Units are Unreal units (centimetres); angles are degrees.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator, Optional


def _vec(d: Any) -> tuple[float, float, float]:
    if isinstance(d, dict):
        return (float(d.get("x", d.get("X", 0.0))), float(d.get("y", d.get("Y", 0.0))), float(d.get("z", d.get("Z", 0.0))))
    if isinstance(d, (list, tuple)) and len(d) >= 3:
        return (float(d[0]), float(d[1]), float(d[2]))
    return (0.0, 0.0, 0.0)


def _quat_yaw(q: dict) -> float:
    """Yaw (degrees) of a UE quaternion {x,y,z,w}."""
    x, y, z, w = (float(q.get(k, q.get(k.upper(), 0.0))) for k in ("x", "y", "z", "w"))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


@dataclass
class Feature:
    """One accessibility-relevant object in the world."""
    kind: str                       # CurbRamp, Entrance, Door, Threshold, Hazard, Prop, Step, Ramp, Tree, ...
    subtype: str = ""               # e.g. Flared, Stepped, AFrame, Scooter
    location: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    size: tuple[float, float, float] = (0.0, 0.0, 0.0)     # full size of the feature's footprint
    compliant: bool = True
    issue: str = "None"             # ERammsComplianceIssue name
    measure: float = 0.0            # the quantity that was checked (slope, rise, width, ...)
    threshold: float = 0.0          # what it was compared against
    route: str = ""                 # route this feature sits on, if any
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "Feature":
        loc, yaw = (0.0, 0.0, 0.0), 0.0
        if "location" in d or "Location" in d:
            loc = _vec(d.get("location") or d.get("Location"))
            rot = d.get("rotation") or d.get("Rotation") or {}
            if isinstance(rot, dict):
                yaw = _quat_yaw(rot) if ("w" in rot or "W" in rot) else float(rot.get("yaw", rot.get("Yaw", 0.0)))
        else:
            xf = d.get("transform") or d.get("Transform")
            if isinstance(xf, dict):
                loc = _vec(xf.get("translation") or xf.get("Translation") or {})
                rot = xf.get("rotation") or xf.get("Rotation") or {}
                if isinstance(rot, dict):
                    yaw = _quat_yaw(rot) if ("w" in rot or "W" in rot) else float(rot.get("yaw", rot.get("Yaw", 0.0)))
        g = lambda *names, default=None: next((d[n] for n in names if n in d), default)
        return cls(
            kind=str(g("kind", "Kind", default="")),
            subtype=str(g("subtype", "Subtype", default="") or ""),
            location=loc, yaw=yaw,
            size=_vec(g("size", "Size", "extent", "Extent", default={}) or {}),
            compliant=bool(g("bCompliant", "compliant", "Compliant", default=True)),
            issue=str(g("issue", "Issue", default="None")),
            measure=float(g("measure", "Measure", default=0.0) or 0.0),
            threshold=float(g("threshold", "Threshold", default=0.0) or 0.0),
            route=str(g("route", "Route", default="") or ""),
            raw=d,
        )

    def distance_to(self, p: tuple[float, float, float]) -> float:
        return math.dist(self.location[:2], p[:2])


@dataclass
class Route:
    """A pedestrian route the generator intends to be traversable."""
    name: str
    points: list[tuple[float, float, float]]
    nominal_width: float = 0.0
    min_clear_width: float = 0.0
    compliant: bool = True
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "Route":
        g = lambda *names, default=None: next((d[n] for n in names if n in d), default)
        return cls(
            name=str(g("name", "Name", default="")),
            points=[_vec(p) for p in (g("points", "Points", default=[]) or [])],
            nominal_width=float(g("nominalWidth", "NominalWidth", "nominal_width", default=0.0) or 0.0),
            min_clear_width=float(g("minClearWidth", "MinClearWidth", "min_clear_width", default=0.0) or 0.0),
            compliant=bool(g("bCompliant", "compliant", "Compliant", default=True)),
            raw=d,
        )

    @property
    def length(self) -> float:
        return sum(math.dist(a[:2], b[:2]) for a, b in zip(self.points, self.points[1:]))

    @property
    def start(self) -> tuple[float, float, float]:
        return self.points[0] if self.points else (0.0, 0.0, 0.0)

    @property
    def end(self) -> tuple[float, float, float]:
        return self.points[-1] if self.points else (0.0, 0.0, 0.0)

    def waypoints(self, spacing: float = 100.0) -> list[tuple[float, float, float]]:
        """Resample the polyline at roughly `spacing` cm (always includes both ends)."""
        if len(self.points) < 2:
            return list(self.points)
        out = [self.points[0]]
        carry = 0.0
        for a, b in zip(self.points, self.points[1:]):
            seg = math.dist(a, b)
            if seg <= 1e-6:
                continue
            t = (spacing - carry) / seg
            while t <= 1.0:
                out.append(tuple(a[i] + (b[i] - a[i]) * t for i in range(3)))
                t += spacing / seg
            carry = (seg - (t - spacing / seg) * seg) if t > 1.0 else 0.0
            carry = max(0.0, min(carry, spacing))
        if out[-1] != self.points[-1]:
            out.append(self.points[-1])
        return out


@dataclass
class Manifest:
    seed: int = 0
    preset: str = ""
    scene_kind: str = ""
    bounds_min: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bounds_max: tuple[float, float, float] = (0.0, 0.0, 0.0)
    features: list[Feature] = field(default_factory=list)
    routes: list[Route] = field(default_factory=list)
    suggested_start: tuple[float, float, float] = (0.0, 0.0, 0.0)
    suggested_start_yaw: float = 0.0
    raw: dict = field(default_factory=dict, repr=False)

    # ── construction ────────────────────────────────────────────────

    @classmethod
    def from_json(cls, data: str | dict) -> "Manifest":
        d = json.loads(data) if isinstance(data, str) else data
        g = lambda *names, default=None: next((d[n] for n in names if n in d), default)
        bounds = g("bounds", "Bounds", default={}) or {}
        start = g("suggestedStart", "SuggestedStart", "suggested_start", default={}) or {}
        start_loc, start_yaw = (0.0, 0.0, 0.0), 0.0
        if isinstance(start, dict):
            start_loc = _vec(start.get("translation") or start.get("Translation") or start)
            rot = start.get("rotation") or start.get("Rotation") or {}
            if isinstance(rot, dict) and rot:
                start_yaw = _quat_yaw(rot) if ("w" in rot or "W" in rot) else float(rot.get("yaw", rot.get("Yaw", 0.0)))
        kind = g("sceneKind", "SceneKind", "scene_kind", default="")
        return cls(
            seed=int(g("seed", "Seed", default=0) or 0),
            preset=str(g("preset", "Preset", default="") or ""),
            scene_kind=str(kind or ""),
            bounds_min=_vec(bounds.get("min") or bounds.get("Min") or {}),
            bounds_max=_vec(bounds.get("max") or bounds.get("Max") or {}),
            features=[Feature.from_json(f) for f in (g("features", "Features", default=[]) or [])],
            routes=[Route.from_json(r) for r in (g("routes", "Routes", default=[]) or [])],
            suggested_start=start_loc, suggested_start_yaw=start_yaw,
            raw=d,
        )

    @classmethod
    def load(cls, path: str | Path) -> "Manifest":
        return cls.from_json(Path(path).read_text(encoding="utf-8"))

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.raw, indent=1), encoding="utf-8")
        return p

    # ── queries ─────────────────────────────────────────────────────

    def features_of(self, kind: str, subtype: Optional[str] = None) -> list[Feature]:
        return [f for f in self.features if f.kind == kind and (subtype is None or f.subtype == subtype)]

    @property
    def issues(self) -> list[Feature]:
        """Every non-compliant feature."""
        return [f for f in self.features if not f.compliant]

    def issues_by_kind(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for f in self.issues:
            out[f.issue] = out.get(f.issue, 0) + 1
        return dict(sorted(out.items()))

    def kinds(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for f in self.features:
            out[f.kind] = out.get(f.kind, 0) + 1
        return dict(sorted(out.items()))

    def route(self, name: str) -> Optional[Route]:
        return next((r for r in self.routes if r.name == name), None)

    def routes_narrower_than(self, width: float) -> list[Route]:
        return [r for r in self.routes if r.min_clear_width < width]

    def features_near(self, p: tuple[float, float, float], radius: float) -> list[Feature]:
        return sorted((f for f in self.features if f.distance_to(p) <= radius), key=lambda f: f.distance_to(p))

    def features_on_route(self, route: Route | str, corridor: float = 200.0) -> list[Feature]:
        """Features tagged with the route's name, plus any within `corridor` cm of its polyline."""
        r = self.route(route) if isinstance(route, str) else route
        if r is None:
            return []
        out = [f for f in self.features if f.route == r.name]
        seen = set(id(f) for f in out)
        for f in self.features:
            if id(f) in seen:
                continue
            if _dist_to_polyline(f.location, r.points) <= corridor:
                out.append(f)
        return out

    def summary(self) -> dict:
        return {
            "preset": self.preset, "seed": self.seed, "scene_kind": self.scene_kind,
            "features": len(self.features), "routes": len(self.routes), "issues": len(self.issues),
            "kinds": self.kinds(), "issues_by_kind": self.issues_by_kind(),
            "route_length_m": round(sum(r.length for r in self.routes) / 100.0, 1),
            "narrowest_route_cm": round(min((r.min_clear_width for r in self.routes), default=0.0), 1),
            "suggested_start": self.suggested_start, "suggested_start_yaw": self.suggested_start_yaw,
        }

    def __iter__(self) -> Iterator[Feature]:
        return iter(self.features)


def _dist_to_polyline(p: tuple[float, float, float], pts: Iterable[tuple[float, float, float]]) -> float:
    pts = list(pts)
    if not pts:
        return float("inf")
    if len(pts) == 1:
        return math.dist(p[:2], pts[0][:2])
    best = float("inf")
    px, py = p[0], p[1]
    for a, b in zip(pts, pts[1:]):
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 1e-9 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best
