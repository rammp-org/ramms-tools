"""Offline tests for ramms_tools.robot: drive modes, Gen1 vs Gen2 curb, seat, out parameters (run: python -m pytest tests)."""
import time

import pytest

from fake_remote import FakeComponent, robot_world
from ramms_tools.robot import (Gen1Curb, Gen2Curb, ModeError, Robot, RobotNotFound, StatusText)


def test_find_by_name_and_first():
    ue, path, _ = robot_world("gen1")
    r = Robot.find(ue=ue)
    assert r.actor_path == path and r.name == "BP_RammpGen1PoweredFront_Ramms_C_0"
    assert Robot.find("rammpgen1poweredfront", ue=ue).actor_path == path
    with pytest.raises(RobotNotFound, match="robots: BP_RammpGen1PoweredFront_Ramms_C_0"):
        Robot.find("gen2", ue=ue)


def test_gen1_vs_gen2_curb_by_component_class():
    ue1, _, _ = robot_world("gen1")
    ue2, _, _ = robot_world("gen2")
    r1, r2 = Robot.find(ue=ue1), Robot.find(ue=ue2)
    assert isinstance(r1.curb, Gen1Curb) and r1.generation == "gen1" and r1.seat is None
    assert isinstance(r2.curb, Gen2Curb) and r2.generation == "gen2" and r2.seat is not None
    assert r1.curb.component_path.endswith(".CurbController")
    assert r2.curb.component_path.endswith(".CurbNegotiation")


def test_robot_without_curb_controller():
    ue, path, comps = robot_world("gen1")
    del ue.actors[path]["CurbController"]
    r = Robot.find(ue=ue)
    assert r.curb is None and r.generation == "" and r.status()["generation"] == ""


def test_modes_listing_and_labels():
    ue, _, _ = robot_world("gen2")
    r = Robot.find(ue=ue)
    modes = r.modes()
    assert [m.id for m in modes] == ["curb", "differential", "holonomic", "low_level"]
    assert [m.label for m in modes] == ["Curb", "Differential", "Holonomic", "Low level"]
    assert [m.id for m in modes if m.active] == ["differential"] and r.mode == "differential"
    dm = r.drive_modes
    assert dm.has_selector
    assert dm.resolve("Low level") == "low_level" and dm.resolve("holo") == "holonomic" and dm.resolve(0) == "curb"
    with pytest.raises(ModeError, match="modes: curb, differential"):
        dm.resolve("flying")
    motion = r.measured_motion()
    assert motion.forward_cm_s == pytest.approx(-0.003) and motion.speed_cm_s < 0.01


def test_gen1_mode_labels_pair_with_ids():
    ue, _, _ = robot_world("gen1")
    r = Robot.find(ue=ue)
    assert r.drive_modes.labels() == {"curb": "Curb crossing", "differential": "Differential"}
    assert r.drive_modes.resolve("curb crossing") == "curb"


def test_set_mode_waits_for_the_hand_over():
    ue, _, comps = robot_world("gen2", switch_polls=3)
    r = Robot.find(ue=ue)
    seen = []
    status = r.set_mode("Holonomic", on_status=seen.append)
    assert r.mode == "holonomic" and status == "holonomic after the stance"
    assert seen[0].startswith("waiting") and len(ue.calls_to("IsSwitchingMode")) >= 3
    assert ue.calls_to("SetActiveMode")[-1] == {"ModeId": "holonomic"}


def test_set_mode_refused_and_unknown():
    ue, _, comps = robot_world("gen2", refuse={"curb"})
    r = Robot.find(ue=ue)
    with pytest.raises(ModeError, match="refused"):
        r.set_mode("curb")
    assert r.mode == "differential"
    with pytest.raises(ModeError):
        r.set_mode("teleport")


def test_cycle_and_fallback_without_selector():
    ue, path, comps = robot_world("gen1")
    r = Robot.find(ue=ue)
    assert r.cycle_mode() == "curb"
    assert r.cycle_mode(wait=True) == "differential"
    ue3, _, _ = robot_world("gen2", refuse={"holonomic"})
    with pytest.raises(ModeError, match="cycling from 'differential' refused"):
        Robot.find(ue=ue3).drive_modes.cycle(wait=True, poll=0.01)
    # No selector: the drive.mode Enum control stands in.
    ue2, path2, comps2 = robot_world("gen1")
    del ue2.actors[path2]["DriveModeSelector"]
    r2 = Robot.find(ue=ue2)
    assert not r2.drive_modes.has_selector and r2.mode == "differential"
    r2.set_mode("curb crossing")
    assert comps2["ControlSurface"].values["drive.mode"] == 0.0 and r2.mode == "curb_crossing"
    assert r2.measured_motion() is None


def test_gen1_curb_out_parameters():
    ue, _, comps = robot_world("gen1")
    c = Robot.find(ue=ue).curb
    assert c.phase() == "Idle" and c.phase_label("FrontUp") == "Front omni wheels up"
    assert c.status_line() == StatusText("Ready", "Info")
    text, severity = c.status_line()
    assert (text, severity) == ("Ready", "Info")
    x = c.crossing()
    assert (x.step_cm, x.crossing_height_cm) == (0.0, 0.0)
    assert c.stats()["MinMarginPhase"] == "Idle" and "CrossingSeconds" in c.stats()
    comps["CurbController"].handlers["StartCrossing"] = lambda p: {"OutReason": "No curb ahead", "ReturnValue": False}
    assert c.start() == (False, "No curb ahead")
    c.abort("test")
    assert ue.calls_to("Abort")[-1] == {"Reason": "test"}
    det = c.detect()
    assert det["available"] is False and det["step_cm"] == 0.0


def test_gen1_assist_and_style():
    ue, _, comps = robot_world("gen1")
    c = Robot.find(ue=ue).curb
    fake = comps["ControlSurface"]
    assert c.assist is False
    c.assist = True
    assert fake.values["curb.assist"] == 1.0 and c.assist is True
    assert c.style == "Stop and step"
    c.set_style("blended")
    assert c.style == "Rolling"
    c.style = "Stop and step"
    assert fake.values["curb.style"] == 0.0


def test_gen2_detection_and_measurements():
    ue, _, comps = robot_world("gen2")
    c = Robot.find(ue=ue).curb
    d = c.detect_step_ahead()
    assert d.found and d.reason.startswith("+12.0 cm step")
    p = d.profile
    assert p.step_height_cm == 12.0 and p.edge_distance_cm == 80.0 and p.edge_skew_cm == 10.0 and p.direction == "up"
    assert p.terrain.valid and p.terrain.near.samples == 400 and p.terrain.far.samples == 250
    assert p.terrain.edge_normal == pytest.approx((0.965926, 0.258819)) and p.terrain.far.origin[2] == -88.0
    assert p.terrain.step_height_range_cm == (11.8, 12.2) and not p.terrain.far_depth_observed
    t = c.detect_terrain_ahead()
    assert t.found and t.profile is None and t.terrain.valid and t.raw["ReturnValue"] is True
    assert not c.active_terrain().valid and c.active_profile().edge_distance_cm == 50
    assert c.measured_tilt() == (0, 0) and c.base_height() == (0, 0)
    cmd, meas = c.drive_speed()
    assert cmd == 0 and meas == pytest.approx(0.002)
    assert c.wheel_progress() == [] and c.leg_heights() == [] and c.stability_margin() == 0
    assert c.plan() == [] and c.schedule() == [] and c.step_log() == []
    assert c.run_stats()["Replans"] == 0 and c.stats() == c.run_stats()
    assert c.phase_axle() == "Front" and c.status() == "Idle"
    snap = c.snapshot()
    assert snap["phase"] == "Idle" and snap["axle"] == "Front" and snap["severity"] == "Info"
    assert c.detect()["step_found"] and "80.0 cm ahead" in c.detection_text()


def test_gen2_lists_and_start():
    ue, _, comps = robot_world("gen2")
    c = Robot.find(ue=ue).curb
    neg = comps["CurbNegotiation"]
    neg.responses["GetLegHeights"] = {"ReturnValue": [{"X": 1.0, "Y": 1.5}, {"X": -2.0, "Y": -2.25}]}
    neg.responses["GetWheelStatus"] = {"ReturnValue": [{"Leg": "FrontLeft", "State": "Lifting", "Plane": 0, "PastEdgeCm": -12.5},
                                                       "not a struct"]}
    neg.responses["GetWheelProgress"] = {"ReturnValue": [-12.5, -12.5, 30.25, "x"]}
    neg.responses["GetWheelClearance"] = {"ReturnValue": [4.0, 0.0]}
    neg.responses["GetPlanDescription"] = {"ReturnValue": ["t= 0.00s front axle up", "t= 1.20s front axle down"]}
    assert c.leg_heights() == [(1.0, 1.5), (-2.0, -2.25)]
    assert c.wheel_status() == [{"Leg": "FrontLeft", "State": "Lifting", "Plane": 0, "PastEdgeCm": -12.5}]
    assert c.wheel_progress() == [-12.5, -12.5, 30.25, 0.0] and c.wheel_clearance() == [4.0, 0.0]
    assert c.plan()[1].endswith("front axle down")
    neg.handlers["NegotiateStepAhead"] = lambda p: {"OutReason": "", "ReturnValue": True}
    assert c.start() == (True, "")
    c.abort()
    assert ue.calls_to("Abort")[-1] == {}


def scripted_phases(comp, script):
    """Make GetPhase / GetStatusLine walk through `script` [(phase, text, severity), ...], one step per poll."""
    state = {"i": 0}

    def phase(p):
        i = min(state["i"], len(script) - 1)
        state["i"] += 1
        return {"ReturnValue": script[i][0]}

    def status(p):
        i = min(max(state["i"] - 1, 0), len(script) - 1)
        return {"OutSeverity": script[i][2], "ReturnValue": script[i][1]}
    comp.handlers.update(GetPhase=phase, GetStatusLine=status)


def test_wait_until_done_logs_phases():
    ue, _, comps = robot_world("gen1")
    c = Robot.find(ue=ue).curb
    scripted_phases(comps["CurbController"], [
        ("Idle", "Ready", "Info"), ("Level", "Crossing up", "Info"), ("FrontUp", "Crossing up", "Info"),
        ("FrontUp", "Waiting: the front omni wheels", "Attention"), ("Restore", "Back", "Info"), ("Done", "Crossed 15 cm", "Info")])
    events = []
    run = c.wait_until_done(timeout=5.0, poll=0.0, on_event=events.append)
    assert run.started and run.succeeded and run.phase == "Done" and run.status.text == "Crossed 15 cm"
    assert [e.phase for e in events] == ["Idle", "Level", "FrontUp", "FrontUp", "Restore", "Done"]
    assert events[2].label == "Front omni wheels up" and events[3].severity == "Attention"
    assert "CrossingSeconds" in run.stats


def test_wait_until_done_never_started():
    ue, _, comps = robot_world("gen1")
    c = Robot.find(ue=ue).curb
    run = c.wait_until_done(timeout=5.0, poll=0.01, start_grace=0.05)
    assert not run.started and not run.succeeded and run.phase == "Idle" and not run.timed_out


def test_gen2_waits_out_a_retried_fault():
    ue, _, comps = robot_world("gen2")
    c = Robot.find(ue=ue).curb
    c.RETRY_GRACE_SECONDS = 0.2
    script = [("Approach", "Crossing", "Info"), ("Faulted", "Faulted: the far side not confirmed", "Fault")] + \
             [("Faulted", "Faulted: retrying", "Fault")] * 3 + [("Lift", "Crossing", "Info"), ("Done", "Crossed", "Info")]
    scripted_phases(comps["CurbNegotiation"], script)
    run = c.wait_until_done(timeout=5.0, poll=0.01)
    assert run.succeeded and "Lift" in [e.phase for e in run.events]


def test_gen2_fault_that_stays():
    ue, _, comps = robot_world("gen2")
    c = Robot.find(ue=ue).curb
    c.RETRY_GRACE_SECONDS = 0.05
    scripted_phases(comps["CurbNegotiation"], [("Approach", "Crossing", "Info"), ("Faulted", "Faulted: tipping", "Fault")])
    run = c.wait_until_done(timeout=5.0, poll=0.01)
    assert run.started and not run.succeeded and run.phase == "Faulted" and run.status.severity == "Fault"


def test_edited_profiles_and_terrains_are_sent_as_edited():
    ue, _, comps = robot_world("gen2")
    c = Robot.find(ue=ue).curb
    neg = comps["CurbNegotiation"]
    for f in ("BeginNegotiation", "CheckFeasibility", "BeginNegotiationOnTerrain"):
        neg.handlers[f] = lambda p: {"OutReason": "", "ReturnValue": True}
    prof = c.detect_step_ahead().profile
    prof.step_height_cm = 10.0
    prof.terrain.near.samples = 12
    c.begin(prof)
    sent = ue.calls_to("BeginNegotiation")[-1]["Profile"]
    assert sent["StepHeightCm"] == 10.0 and sent["Terrain"]["Near"]["Samples"] == 12
    assert sent["EdgeDistanceCm"] == prof.edge_distance_cm and sent["Terrain"]["bValid"] is True
    from ramms_tools.robot import CurbProfile, CurbTerrain
    c.check_feasibility(CurbProfile(step_height_cm=12.0, edge_distance_cm=60.0))      # built from scratch
    sent = ue.calls_to("CheckFeasibility")[-1]["Profile"]
    assert sent["StepHeightCm"] == 12.0 and sent["EdgeDistanceCm"] == 60.0 and sent["Terrain"]["bValid"] is False
    t = CurbTerrain(valid=True, step_height_cm=-8.0, edge_point=(1.0, 2.0, 3.0))
    c.begin_on_terrain(t)
    sent = ue.calls_to("BeginNegotiationOnTerrain")[-1]["Terrain"]
    assert sent["StepHeightCm"] == -8.0 and sent["EdgePoint"] == {"X": 1.0, "Y": 2.0, "Z": 3.0}
    assert CurbProfile.from_json(prof.to_json()).step_height_cm == 10.0          # round trip


def timed_phases(comp, script, t0):
    """GetPhase / GetStatusLine follow `script` [(seconds from t0["t"], phase, text, severity), ...] in time."""
    def now():
        if t0["t"] is None:
            return script[0]
        el = time.monotonic() - t0["t"]
        return [row for row in script if row[0] <= el][-1]
    comp.handlers.update(GetPhase=lambda p: {"ReturnValue": now()[1]},
                         GetStatusLine=lambda p: {"OutSeverity": now()[3], "ReturnValue": now()[2]})
    return now


def test_cross_holds_the_stick_only_while_the_crossing_runs():
    """Gen2: a fault that hands the robot back (refused at rest), then the retry. The stick is not held while it is
    Faulted -- the differential mode would drive on it -- and is released on a latching sim by the end."""
    ue, path, comps = robot_world("gen2", deadman=False)
    c = Robot.find(ue=ue).curb
    c.RETRY_GRACE_SECONDS = 1.0
    t0 = {"t": None}
    script = [(0.0, "Idle", "Ready", "Info"), (0.05, "Approach", "Crossing", "Info"),
              (0.5, "Faulted", "Faulted: refused at rest", "Fault"), (1.1, "Cross", "Crossing", "Info"),
              (1.5, "Done", "Crossed", "Info")]
    now = timed_phases(comps["CurbNegotiation"], script, t0)

    def start(p):
        t0["t"] = time.monotonic()
        return {"OutReason": "", "ReturnValue": True}
    comps["CurbNegotiation"].handlers["NegotiateStepAhead"] = start
    fake = comps["ControlSurface"]
    seen = []
    fake.on_call = lambda f, p: seen.append((time.monotonic(), f, now()[1], now()[0])) if p.get("Id") == "drive.forward" else None
    events = []
    run = c.cross(forward=0.5, on_event=events.append)
    assert run.succeeded and [e.phase for e in events if e.phase != "Idle"] == ["Approach", "Faulted", "Cross", "Done"]
    sets = [(t, ph, since) for t, f, ph, since in seen if f == "SetControl"]
    assert {ph for t, ph, _ in sets} >= {"Approach", "Cross"}
    # Never held well into a phase that is not under way (a poll and a send of slack at most).
    late = [(ph, round(t - t0["t"] - since, 3)) for t, ph, since in sets if ph in ("Faulted", "Done", "Idle")
            and t - t0["t"] - since > 0.25]
    assert late == []
    assert all(v == 0.5 for _, cid, v in fake.set_times if cid == "drive.forward")
    releases = [ph for t, f, ph, _ in seen if f == "ReleaseControl"]
    assert "Faulted" in releases                               # let go when the fault was seen
    assert fake.latched() == {} and run.stick_refused == 0


def test_cross_refused_and_the_stick_never_sent():
    ue, path, comps = robot_world("gen1", deadman=False)
    c = Robot.find(ue=ue).curb
    comps["CurbController"].handlers["StartCrossing"] = lambda p: {"OutReason": "No curb ahead", "ReturnValue": False}
    run = c.cross()
    assert not run.started and run.reason == "No curb ahead" and not run.succeeded
    assert comps["ControlSurface"].sets == []


def test_wait_without_a_start_never_holds_the_stick():
    ue, path, comps = robot_world("gen1", deadman=False)
    c = Robot.find(ue=ue).curb
    run = c.wait_until_done(poll=0.01, start_grace=0.15, hold_forward=0.5)
    assert not run.started and comps["ControlSurface"].sets == [] and comps["ControlSurface"].latched() == {}


def test_wait_releases_the_stick_when_interrupted():
    ue, path, comps = robot_world("gen1", deadman=False)
    c = Robot.find(ue=ue).curb
    calls = {"n": 0}

    def phase(p):
        calls["n"] += 1
        if calls["n"] > 5:
            raise KeyboardInterrupt
        return {"ReturnValue": "Level"}
    comps["CurbController"].handlers["GetPhase"] = phase
    with pytest.raises(KeyboardInterrupt):
        c.wait_until_done(poll=0.05, hold_forward=0.5)
    assert ("drive.forward", 0.5, "Remote") in comps["ControlSurface"].sets and comps["ControlSurface"].latched() == {}


def test_seat_out_parameters():
    ue, _, comps = robot_world("gen2")
    s = Robot.find(ue=ue).seat
    x, z = s.pivot()
    assert (x, z) == (-10.0, 40.0)
    assert s.tilt() == 2.0
    poly = s.outline()
    assert len(poly) == 12 and poly[0] == (10.0, 25.0) and s.contains(0.0, 40.0) and not s.contains(20.0, 40.0)
    seat = comps["Seat"]
    seat.handlers.update(SetPivotTarget=lambda p: {"ReturnValue": True}, SetTiltDeg=lambda p: {"ReturnValue": True},
                         ResetSeat=lambda p: {"ReturnValue": True})
    assert s.set_pivot(0.0, 45.0) and ue.calls_to("SetPivotTarget")[-1] == {"TargetXZ": {"X": 0.0, "Y": 45.0}}
    assert s.set_tilt(-5) and ue.calls_to("SetTiltDeg")[-1] == {"Deg": -5.0}
    assert s.reset()
    seat.responses["GetPivot"] = {"bValid": False, "ReturnValue": {"X": 0, "Y": 0}}
    seat.responses["GetReachableOutline"] = {"bValid": False, "ReturnValue": []}
    assert s.pivot() is None and s.outline() == [] and not s.contains(0, 40)


def test_seat_contains():
    from ramms_tools.robot.seat import point_in_polygon
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert point_in_polygon(5, 5, square) and not point_in_polygon(15, 5, square)


def test_controller_on_a_carried_actor_is_found_in_the_world():
    ue, path, comps = robot_world("gen2")
    seat = ue.actors[path].pop("Seat")
    ue.add_actor("SeatActor_C_0", {"Seat": seat})
    r = Robot.find(ue=ue)
    assert r.seat is not None and r.seat.component_path.endswith("SeatActor_C_0.Seat")
