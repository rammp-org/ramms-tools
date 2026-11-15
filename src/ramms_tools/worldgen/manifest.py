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


def _name(v: Any) -> str:
    """An FName from the manifest: "None" (Unreal's empty name) becomes ""."""
    s = str(v or "")
    return "" if s == "None" else s


def _quat_yaw(q: dict) -> float:
    """Yaw (degrees) of a UE quaternion {x,y,z,w}."""
    x, y, z, w = (float(q.get(k, q.get(k.upper(), 0.0))) for k in ("x", "y", "z", "w"))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


# Kinds the plugin writes. Unreal FNames compare case-insensitively and a cooked build can hand
# back a different casing than the editor ("door" vs "Door"), so names are canonicalised here.
KNOWN_KINDS = (
    "Aisle", "Building", "CarPark", "CheckoutLane", "Cover", "Crossing", "Crosswalk", "Curb", "CurbRamp", "Decor", "Door",
    "Driveway", "Entrance", "Furniture", "Grade", "Hazard", "House", "Intersection", "Path", "Pond", "Prop", "Ramp",
    "Road", "Room", "Shrub", "SpeedHump", "Stall", "Step", "Surface", "Terrain", "Threshold", "Tree", "Vehicle", "Water",
)
_CANON = {k.lower(): k for k in KNOWN_KINDS}


def canonical_name(name: str) -> str:
    """Canonical casing for a kind / subtype name: a known kind as spelled above, else first letter upper."""
    name = str(name or "")
    if not name:
        return name
    return _CANON.get(name.lower()) or (name[0].upper() + name[1:])


# The features the roughness parameters lay (kind, subtype), on graded / rough ground:
# SurfaceRoughness -> Surface/JointLip, CrackDensity -> Surface/Crack, PotholeDensity -> Surface/Pothole,
# PavingPatchDensity -> Surface/Brick and Surface/Setts, HeaveDensity -> Hazard/Heave, RootHeaveChance ->
# Hazard/RootHeave, CoverDensity -> Cover/Raised and Cover/Sunken, SpeedHumpChance -> SpeedHump/Hump and Table.
ROUGHNESS_FEATURES = (
    ("Surface", "JointLip"), ("Surface", "Crack"), ("Surface", "Pothole"), ("Surface", "Brick"), ("Surface", "Setts"),
    ("Hazard", "Heave"), ("Hazard", "RootHeave"), ("Cover", "Raised"), ("Cover", "Sunken"),
    ("SpeedHump", "Hump"), ("SpeedHump", "Table"),
)
_ROUGHNESS = {(k.lower(), st.lower()) for k, st in ROUGHNESS_FEATURES}


def _range(values: list[float], digits: int) -> tuple[float, float]:
    return (round(min(values), digits), round(max(values), digits)) if values else (0.0, 0.0)


def _counts(items: Iterable[str]) -> dict[str, int]:
    out: dict[str, int] = {}
    for it in items:
        out[it] = out.get(it, 0) + 1
    return dict(sorted(out.items()))


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
    asset: str = ""                 # mesh asset placed for it (palette meshes), e.g. /Game/UrbanVerse/Bench/SM_UV_...
    source: str = ""                # where that asset came from, when known: "urbanverse:<uid>|<category>"
    raw: dict = field(default_factory=dict, repr=False)

    @property
    def source_dataset(self) -> str:
        """The dataset part of `source` ("urbanverse"), or ""."""
        return self.source.split(":", 1)[0] if ":" in self.source else ""

    @property
    def source_uid(self) -> str:
        """The asset id within that dataset (an UrbanVerse / Sketchfab uid), or ""."""
        return self.source.split(":", 1)[1].split("|", 1)[0] if ":" in self.source else ""

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
            kind=canonical_name(g("kind", "Kind", default="")),
            subtype=canonical_name(g("subtype", "Subtype", default="") or ""),
            location=loc, yaw=yaw,
            size=_vec(g("size", "Size", "extent", "Extent", default={}) or {}),
            compliant=bool(g("bCompliant", "compliant", "Compliant", default=True)),
            issue=str(g("issue", "Issue", default="None")),
            measure=float(g("measure", "Measure", default=0.0) or 0.0),
            threshold=float(g("threshold", "Threshold", default=0.0) or 0.0),
            route=_name(g("route", "Route", default="")),
            asset=str(g("asset", "Asset", default="") or ""),
            source=str(g("source", "Source", default="") or ""),
            raw=d,
        )

    def crossing_ends(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """For a Crosswalk: the two ends of its centre line at the curbs (size.x is curb to curb, yaw
        points along the crossing)."""
        h = self.size[0] * 0.5
        dx, dy = math.cos(math.radians(self.yaw)) * h, math.sin(math.radians(self.yaw)) * h
        x, y, z = self.location
        return (x - dx, y - dy, z), (x + dx, y + dy, z)

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
    palette: str = ""               # object path of the palette it was generated with ("" in older manifests)
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
            palette=str(g("palette", "Palette", default="") or ""),
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
        """Features of a kind (and optionally subtype); matching ignores case, as Unreal names do."""
        k, st = kind.lower(), None if subtype is None else subtype.lower()
        return [f for f in self.features if f.kind.lower() == k and (st is None or f.subtype.lower() == st)]

    def crosswalks(self, step_free: Optional[bool] = None) -> list[Feature]:
        """Street crossings that join two sidewalks. step_free=True: only those with a curb ramp at both
        ends (subtype Ramped or SteepRamp - SteepRamp is too steep for some chairs); False: the rest
        (OneRamp, NoRamps)."""
        out = self.features_of("Crosswalk")
        if step_free is None:
            return out
        return [f for f in out if (f.subtype.lower() in ("ramped", "steepramp")) == step_free]

    @property
    def issues(self) -> list[Feature]:
        """Every non-compliant feature."""
        return [f for f in self.features if not f.compliant]

    def issues_by_kind(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for f in self.issues:
            out[f.issue] = out.get(f.issue, 0) + 1
        return dict(sorted(out.items()))

    def assets(self, kind: Optional[str] = None) -> dict[str, int]:
        """How often each mesh asset was placed (optionally for one kind); features without an asset are left out."""
        out: dict[str, int] = {}
        for f in self.features:
            if f.asset and (kind is None or f.kind.lower() == kind.lower()):
                out[f.asset] = out.get(f.asset, 0) + 1
        return dict(sorted(out.items()))

    def sources(self) -> dict[str, int]:
        """Placed-asset counts per source dataset ("urbanverse", ...; "" for the palette's own meshes)."""
        out: dict[str, int] = {}
        for f in self.features:
            if f.asset:
                out[f.source_dataset] = out.get(f.source_dataset, 0) + 1
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

    # ── graded streets and rough ground ─────────────────────────────

    def grades(self, subtype: Optional[str] = None) -> list[Feature]:
        """Grade records (graded ground): per sidewalk side, subtype Running (its steepest running grade along the
        route) and Cross (its cross slope at mid-side); measure is rise / run, route the side."""
        return self.features_of("Grade", subtype)

    def grades_by_side(self) -> dict[str, dict[str, float]]:
        """{side: {"running": grade, "cross": slope}} from the Grade records (rise / run)."""
        out: dict[str, dict[str, float]] = {}
        for f in self.grades():
            side = f.route or f"{f.location[0]:.0f},{f.location[1]:.0f}"
            out.setdefault(side, {})[f.subtype.lower()] = f.measure
        return out

    def curbs(self, subtype: Optional[str] = None) -> list[Feature]:
        """Curb records (graded ground): per side, subtype Low (under 10 cm) / Standard / High (over 17 cm);
        measure is the mean curb height, size.z the highest (cm)."""
        return self.features_of("Curb", subtype)

    def curb_heights(self) -> list[tuple[float, float]]:
        """(mean, highest) curb height per side, cm."""
        return [(f.measure, f.size[2] or f.measure) for f in self.curbs()]

    def curb_ramps(self, subtype: Optional[str] = None) -> list[Feature]:
        """Curb ramps (Standard, TooSteep, NoDetectableWarning, CrossSlope, SteepFlare)."""
        return self.features_of("CurbRamp", subtype)

    def curb_ramps_by_subtype(self) -> dict[str, int]:
        return _counts(f.subtype or "-" for f in self.curb_ramps())

    def roughness(self, subtype: Optional[str] = None) -> list[Feature]:
        """The rough-ground features (see ROUGHNESS_FEATURES): joint lips, cracks, potholes, pavers, heaves, root
        heaves, utility covers and speed humps; optionally one subtype."""
        st = None if subtype is None else subtype.lower()
        return [f for f in self.features if (f.kind.lower(), f.subtype.lower()) in _ROUGHNESS
                and (st is None or f.subtype.lower() == st)]

    def roughness_counts(self) -> dict[str, int]:
        """How many of each rough-ground feature, keyed "Kind/Subtype"."""
        return _counts(f"{f.kind}/{f.subtype}" for f in self.roughness())

    def speed_humps(self) -> list[Feature]:
        return self.features_of("SpeedHump")

    def terrain_summary(self) -> dict:
        """
        The ground at a glance: steepest running grade, cross-slope range, curb-height range (lowest mean to
        highest point, cm), curb ramps by subtype, rough-ground features by kind / subtype, and the
        non-compliant grades. A flat world (no Grade records) reports graded False and zeros.
        """
        running = [abs(f.measure) for f in self.grades("Running")]
        cross = [abs(f.measure) for f in self.grades("Cross")]
        curbs = self.curbs()
        lows = [f.measure for f in curbs]
        highs = [max(f.size[2], f.measure) for f in curbs]
        return {
            "graded": bool(running or cross),
            "grades": len(running) + len(cross),
            "max_running_grade": round(max(running, default=0.0), 4),
            "cross_slope_range": _range(cross, 4),
            "curbs": len(curbs),
            "curb_height_range": (round(min(lows), 1), round(max(highs), 1)) if curbs else (0.0, 0.0),
            "curbs_by_subtype": _counts(f.subtype or "-" for f in curbs),
            "curb_ramps": self.curb_ramps_by_subtype(),
            "roughness": self.roughness_counts(),
            "steep_running_grades": sum(1 for f in self.grades("Running") if not f.compliant),
            "steep_cross_slopes": sum(1 for f in self.grades("Cross") if not f.compliant),
        }

    # ── traffic ─────────────────────────────────────────────────────

    def roads(self, subtype: Optional[str] = None) -> list[Feature]:
        """Road records (AlongX / AlongY): a live street segment; location on its centreline, size.x its length
        between the intersection squares, size.y = measure its width (cm)."""
        return self.features_of("Road", subtype)

    def intersections(self, subtype: Optional[str] = None) -> list[Feature]:
        """Intersection records (Cross, Tee, Through, Bend, End); measure = how many segments meet."""
        return self.features_of("Intersection", subtype)

    def traffic_signals(self) -> list[Feature]:
        """Traffic signal posts (Prop / TrafficSignal records): each makes its nearest intersection signalized."""
        return self.features_of("Prop", "TrafficSignal")

    def traffic_summary(self) -> dict:
        """Roads (count, total length in m, width range in cm), intersections by subtype, signals, crosswalks,
        speed humps and potholes: what the traffic is built from."""
        roads = self.roads()
        return {
            "roads": len(roads),
            "road_length_m": round(sum(f.size[0] for f in roads) / 100.0, 1),
            "road_width_range_cm": _range([f.measure or f.size[1] for f in roads], 1),
            "intersections": _counts(f.subtype or "-" for f in self.intersections()),
            "signals": len(self.traffic_signals()),
            "crosswalks": len(self.features_of("Crosswalk")),
            "speed_humps": len(self.speed_humps()),
            "potholes": len(self.features_of("Surface", "Pothole")),
        }

    def summary(self) -> dict:
        return {
            "preset": self.preset, "seed": self.seed, "scene_kind": self.scene_kind, "palette": self.palette,
            "features": len(self.features), "routes": len(self.routes), "issues": len(self.issues),
            "kinds": self.kinds(), "issues_by_kind": self.issues_by_kind(), "asset_sources": self.sources(),
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
