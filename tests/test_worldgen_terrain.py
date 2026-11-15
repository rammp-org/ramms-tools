"""Offline tests for the world-gen terrain / traffic helpers (run: python -m pytest tests)."""
import json

import pytest

from fake_remote import FakeRemote
from ramms_tools.worldgen import BRIDGE, ROUGHNESS_PARAMS, TERRAIN_PARAMS, Generator, Traffic, TrafficError, WorldGen, terrain_values
from ramms_tools.worldgen.manifest import Manifest


def feat(kind, subtype="", x=0.0, y=0.0, size=(0, 0, 0), measure=0.0, compliant=True, issue="None", route="None"):
    return {"kind": kind, "subtype": subtype, "location": {"x": x, "y": y, "z": 0},
            "rotation": {"pitch": 0, "yaw": 0, "roll": 0}, "size": {"x": size[0], "y": size[1], "z": size[2]},
            "bCompliant": compliant, "issue": issue, "measure": measure, "threshold": 0, "route": route}


GRADED = {"seed": 21, "preset": "PRE_GradedTraffic", "sceneKind": "Downtown", "routes": [], "features": [
    feat("Grade", "Running", route="Block0_Side0", measure=0.029),
    feat("Grade", "Cross", route="Block0_Side0", measure=0.018),
    feat("Grade", "Running", route="Block0_Side1", measure=-0.091, compliant=False, issue="SlopeTooSteep"),
    feat("grade", "cross", route="Block0_Side1", measure=0.045, compliant=False, issue="CrossSlopeTooSteep"),
    feat("Curb", "Low", route="Block0_Side0", measure=8.2, size=(4000, 15, 9.5)),
    feat("Curb", "High", route="Block0_Side1", measure=18.4, size=(4000, 15, 19.9)),
    feat("CurbRamp", "Standard"), feat("CurbRamp", "Standard"), feat("CurbRamp", "CrossSlope", compliant=False),
    feat("CurbRamp", "SteepFlare", compliant=False),
    feat("Surface", "JointLip", measure=0.9), feat("Surface", "Crack"), feat("Surface", "Pothole", measure=6.0),
    feat("Surface", "Brick"), feat("Surface", "Cobbled"),           # a park patch, not roughness
    feat("Hazard", "Heave"), feat("Hazard", "RootHeave"), feat("Hazard", "AFrame"),
    feat("Cover", "Raised", measure=1.2, compliant=False, issue="VerticalChangeTooHigh"), feat("Cover", "Sunken"),
    feat("SpeedHump", "Table", measure=8.0),
    feat("Road", "AlongX", size=(4200, 1100, 0), measure=1100), feat("Road", "AlongY", size=(3800, 1000, 0), measure=1000),
    feat("Intersection", "Cross", measure=4), feat("Intersection", "Tee", measure=3), feat("Intersection", "Cross", measure=4),
    feat("Prop", "TrafficSignal"), feat("Prop", "TrafficSignal"), feat("Prop", "Bench"),
    feat("Crosswalk", "Ramped"),
]}


def test_new_kinds_are_canonical():
    m = Manifest.from_json(GRADED)
    k = m.kinds()
    assert k["Grade"] == 4 and k["SpeedHump"] == 1 and k["Intersection"] == 3 and k["Road"] == 2 and k["Cover"] == 2


def test_grades_curbs_ramps_roughness():
    m = Manifest.from_json(GRADED)
    assert len(m.grades()) == 4 and [f.measure for f in m.grades("running")] == [0.029, -0.091]
    assert m.grades_by_side() == {"Block0_Side0": {"running": 0.029, "cross": 0.018},
                                  "Block0_Side1": {"running": -0.091, "cross": 0.045}}
    assert m.curb_heights() == [(8.2, 9.5), (18.4, 19.9)] and [f.subtype for f in m.curbs("high")] == ["High"]
    assert m.curb_ramps_by_subtype() == {"CrossSlope": 1, "SteepFlare": 1, "Standard": 2}
    assert len(m.curb_ramps("Standard")) == 2
    assert m.roughness_counts() == {"Cover/Raised": 1, "Cover/Sunken": 1, "Hazard/Heave": 1, "Hazard/RootHeave": 1,
                                    "SpeedHump/Table": 1, "Surface/Brick": 1, "Surface/Crack": 1, "Surface/JointLip": 1,
                                    "Surface/Pothole": 1}
    assert [f.kind for f in m.roughness("Pothole")] == ["Surface"]


def test_terrain_summary():
    ts = Manifest.from_json(GRADED).terrain_summary()
    assert ts["graded"] and ts["grades"] == 4
    assert ts["max_running_grade"] == 0.091                      # magnitude: a street falls as well as climbs
    assert ts["cross_slope_range"] == (0.018, 0.045)
    assert ts["curbs"] == 2 and ts["curb_height_range"] == (8.2, 19.9) and ts["curbs_by_subtype"] == {"High": 1, "Low": 1}
    assert ts["steep_running_grades"] == 1 and ts["steep_cross_slopes"] == 1
    assert ts["roughness"]["Surface/Pothole"] == 1 and ts["curb_ramps"]["Standard"] == 2
    flat = Manifest.from_json({"features": [feat("CurbRamp", "Standard")], "routes": []}).terrain_summary()
    assert not flat["graded"] and flat["max_running_grade"] == 0.0 and flat["curb_height_range"] == (0.0, 0.0)
    assert json.dumps(ts)                                         # plain data


def test_traffic_summary():
    m = Manifest.from_json(GRADED)
    assert [f.subtype for f in m.traffic_signals()] == ["TrafficSignal", "TrafficSignal"]
    t = m.traffic_summary()
    assert t == {"roads": 2, "road_length_m": 80.0, "road_width_range_cm": (1000.0, 1100.0),
                 "intersections": {"Cross": 2, "Tee": 1}, "signals": 2, "crosswalks": 1, "speed_humps": 1, "potholes": 1}
    assert Manifest.from_json({"features": [], "routes": []}).traffic_summary()["roads"] == 0


def test_terrain_values_map_friendly_names_exactly():
    # The README's names (Parameters table; "Adjusting slope, cross slope and curb heights").
    assert {f: p for f, (p, _, _) in TERRAIN_PARAMS.items()} == {
        "max_grade": "MaxStreetGrade", "crown": "RoadCrown", "cross_slope": "SidewalkCrossSlope",
        "noncompliant_cross_slope": "NonCompliantCrossSlopeChance", "curb_height": "CurbHeight",
        "curb_min": "CurbHeightMin", "curb_max": "CurbHeightMax"}
    assert {f: p for f, (p, _, _) in ROUGHNESS_PARAMS.items()} == {
        "roughness": "SurfaceRoughness", "panel_length": "PanelLength", "heaves": "HeaveDensity",
        "cracks": "CrackDensity", "covers": "CoverDensity", "paving": "PavingPatchDensity",
        "potholes": "PotholeDensity", "humps": "SpeedHumpChance", "root_heaves": "RootHeaveChance"}
    assert terrain_values(max_grade=0.08, curb_min=10, curb_max=20, potholes=2, humps=None) == {
        "MaxStreetGrade": 0.08, "CurbHeightMin": 10.0, "CurbHeightMax": 20.0, "PotholeDensity": 2.0}
    assert terrain_values(MaxStreetGrade=0.1) == {"MaxStreetGrade": 0.1}      # parameter names work too
    flat = terrain_values(flat=True, crown=0.02)
    assert flat["MaxStreetGrade"] == 0.0 and flat["RoadCrown"] == 0.02 and "CurbHeight" not in flat
    with pytest.raises(ValueError, match="0..0.15"):
        terrain_values(max_grade=0.2)
    with pytest.raises(ValueError, match="below curb_min"):
        terrain_values(curb_min=15, curb_max=10)
    with pytest.raises(ValueError, match="PanelLength"):
        terrain_values(panel_length=30)                     # the plugin clamps panels to 60 cm and up
    assert terrain_values(panel_length=60) == {"PanelLength": 60.0}
    with pytest.raises(KeyError, match="grade_max"):
        terrain_values(grade_max=0.1)


class FakeWorldGenBridge:
    def __init__(self, known):
        self.params = dict(known)
        self.sets = []

    def __call__(self, func, p):
        if func == "GetStatusJson":
            return {"ReturnValue": json.dumps({"ok": True, "seed": 1})}
        if func == "SetParamsJson":
            values = json.loads(p["Json"])
            self.sets.append(values)
            applied = [k for k in values if k in self.params]
            self.params.update({k: values[k] for k in applied})
            return {"ReturnValue": json.dumps({"ok": True, "applied": applied, "unknown": [k for k in values if k not in self.params]})}
        if func == "GetParamsJson":
            return {"ReturnValue": json.dumps(self.params)}
        raise AssertionError(func)


def test_generator_set_terrain_and_read_back():
    ue = FakeRemote()
    bridge = FakeWorldGenBridge({"MaxStreetGrade": 0.0, "RoadCrown": 0.0, "CurbHeight": 15.0, "SurfaceRoughness": 0.0,
                                 "HazardDensity": 2.0})
    ue.objects[BRIDGE] = bridge
    gen = Generator(WorldGen(ue), "/Game/M.M:PersistentLevel.Gen")
    res = gen.set_terrain(max_grade=0.05, roughness=0.5, potholes=1.0)
    assert bridge.sets[-1] == {"MaxStreetGrade": 0.05, "SurfaceRoughness": 0.5, "PotholeDensity": 1.0}
    assert res["unknown"] == ["PotholeDensity"]                 # an older plugin: warned, not fatal
    with pytest.raises(KeyError):
        gen.set_terrain(potholes=1.0, strict=True)
    assert gen.terrain() == {"max_grade": 0.05, "crown": 0.0, "curb_height": 15.0, "roughness": 0.5}
    assert gen.set_terrain() == {"ok": True, "applied": [], "unknown": []}


def traffic_world():
    ue = FakeRemote()
    state = {"count": 0, "spawned": 0}

    def spawner(func, p):
        if func == "SpawnTraffic":
            state["count"], state["spawned"] = 24, state["spawned"] + 1
            return {"ReturnValue": True}
        if func == "DespawnTraffic":
            state["count"] = 0
            return {}
        if func == "GetSpawnedCount":
            return {"ReturnValue": state["count"]}
        raise AssertionError(func)

    def clock(func, p):
        if func == "GetSignalState":
            return {"ReturnValue": {"LightX": "Green", "LightY": "Red", "WalkAlongX": "DontWalk", "WalkAlongY": "Walk",
                                    "SecondsToChangeX": 12.5, "SecondsToChangeY": 17.5}}
        if func == "GetControlledCrossingCount":
            return {"ReturnValue": 8}
        raise AssertionError(func)

    ue.add_actor("RammsTrafficSpawner_0", cls="RammsTrafficSpawner", handler=spawner)
    ue.add_actor("RammsTrafficSignalClock_0", cls="RammsTrafficSignalClock", handler=clock)
    for i in range(3):
        def car(func, p, i=i):
            if func == "K2_GetActorLocation":
                return {"ReturnValue": {"X": 100.0 * i, "Y": 5.0, "Z": -20.0}}
            if func == "K2_GetActorRotation":
                return {"ReturnValue": {"Pitch": 1.0, "Yaw": 90.0 * i, "Roll": 0.0}}
            raise AssertionError(func)
        ue.add_actor(f"RammsTrafficVehicleActor_{i}", cls="RammsTrafficVehicleActor", handler=car)
    return ue, state


def test_traffic_spawn_count_signals_cars():
    ue, state = traffic_world()
    t = Traffic(ue)
    assert len(t.spawners()) == 1 and len(t.clocks()) == 1 and t.count() == 0
    assert list(t.spawn().values()) == [True] and t.count() == 24
    s = t.signal_state()
    assert (s.light_x, s.light_y, s.walk_along_x, s.walk_along_y) == ("Green", "Red", "DontWalk", "Walk")
    assert s.seconds_to_change_y == 17.5 and "X Green" in str(s)
    assert t.controlled_crossings() == 8
    cars = t.cars()
    assert [c.name for c in cars] == ["RammsTrafficVehicleActor_0", "RammsTrafficVehicleActor_1", "RammsTrafficVehicleActor_2"]
    assert cars[1].location == (100.0, 5.0, -20.0) and cars[2].yaw == 180.0
    assert len(t.cars(limit=2)) == 2
    st = t.status()
    assert st["cars"] == 24 and st["signals"]["light_x"] == "Green" and st["controlled_crossings"] == 8
    assert t.despawn() == 1 and t.count() == 0
    with pytest.raises(TrafficError, match="no traffic spawner matching"):
        t.spawn("Nope")


def test_traffic_on_a_level_without_any():
    t = Traffic(FakeRemote())
    assert t.spawners() == [] and t.signal_state() is None and t.status() == {"spawners": 0, "clocks": 0, "cars": 0}
    with pytest.raises(TrafficError, match="no traffic spawner in the world"):
        t.spawn()
