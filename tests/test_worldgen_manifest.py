"""Offline tests for ramms_tools.worldgen.manifest (run: python -m pytest tests)."""
import json
import math

from ramms_tools.worldgen.manifest import Feature, Manifest, Route

SAMPLE = {
    "seed": 7, "preset": "PRE_Downtown", "sceneKind": "Downtown",
    "bounds": {"min": {"x": -100, "y": -100, "z": -5}, "max": {"x": 100, "y": 100, "z": 5}, "isValid": True},
    "features": [
        {"kind": "Entrance", "subtype": "Level", "location": {"x": 10, "y": 0, "z": 0},
         "rotation": {"pitch": 0, "yaw": -90, "roll": 0}, "size": {"x": 60, "y": 150, "z": 10},
         "bCompliant": False, "issue": "DoorTooNarrow", "measure": 78.7, "threshold": 81.3},
        {"kind": "Hazard", "subtype": "AFrame", "location": {"x": 50, "y": 5, "z": 0},
         "rotation": {"pitch": 0, "yaw": 0, "roll": 0}, "size": {"x": 60, "y": 60, "z": 100},
         "bCompliant": True, "issue": "None", "measure": 120.0, "threshold": 91.4, "route": "Block0_Side0"},
        {"kind": "Tree", "subtype": "", "location": {"x": 0, "y": 900, "z": 0},
         "rotation": {"pitch": 0, "yaw": 0, "roll": 0}, "size": {"x": 400, "y": 400, "z": 800},
         "bCompliant": True, "issue": "None", "measure": 0, "threshold": 0},
    ],
    "routes": [
        {"name": "Block0_Side0", "points": [{"x": 0, "y": 0, "z": 0}, {"x": 100, "y": 0, "z": 0}, {"x": 100, "y": 50, "z": 0}],
         "nominalWidth": 150, "minClearWidth": 120, "bCompliant": True},
        {"name": "Block0_Side1", "points": [{"x": 0, "y": 0, "z": 0}, {"x": 0, "y": 80, "z": 0}],
         "nominalWidth": 150, "minClearWidth": 60, "bCompliant": False},
    ],
    "suggestedStart": {"rotation": {"x": 0, "y": 0, "z": 1.0, "w": 0.0}, "translation": {"x": 5, "y": 6, "z": 8},
                       "scale3D": {"x": 1, "y": 1, "z": 1}},
}


def _m() -> Manifest:
    return Manifest.from_json(json.dumps(SAMPLE))


def test_header_and_start():
    m = _m()
    assert (m.seed, m.preset, m.scene_kind) == (7, "PRE_Downtown", "Downtown")
    assert m.bounds_min == (-100, -100, -5) and m.bounds_max == (100, 100, 5)
    assert m.suggested_start == (5, 6, 8)
    assert abs(abs(m.suggested_start_yaw) - 180.0) < 1e-6


def test_features_and_issues():
    m = _m()
    assert m.kinds() == {"Entrance": 1, "Hazard": 1, "Tree": 1}
    assert [f.kind for f in m.issues] == ["Entrance"]
    assert m.issues_by_kind() == {"DoorTooNarrow": 1}
    e = m.features_of("Entrance")[0]
    assert e.yaw == -90 and e.size == (60, 150, 10) and e.measure == 78.7 and e.threshold == 81.3
    assert [f.kind for f in m.features_near((0, 0, 0), 20)] == ["Entrance"]


def test_routes():
    m = _m()
    r = m.route("Block0_Side0")
    assert r is not None and r.length == 150 and r.start == (0, 0, 0) and r.end == (100, 50, 0)
    assert [x.name for x in m.routes_narrower_than(91.4)] == ["Block0_Side1"]
    wps = r.waypoints(spacing=50)
    assert wps[0] == (0, 0, 0) and wps[-1] == (100, 50, 0)
    assert all(math.dist(a, b) <= 50 + 1e-6 for a, b in zip(wps, wps[1:]))
    on = m.features_on_route("Block0_Side0", corridor=10)
    assert {f.kind for f in on} == {"Hazard", "Entrance"}  # tagged + within the corridor; the tree is far away


def test_summary_and_roundtrip(tmp_path):
    m = _m()
    s = m.summary()
    assert s["features"] == 3 and s["routes"] == 2 and s["issues"] == 1 and s["narrowest_route_cm"] == 60
    p = m.save(tmp_path / "m.json")
    assert Manifest.load(p).summary() == s
