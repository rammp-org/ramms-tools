"""Offline tests for ramms_tools.control against recorded control surfaces (run: python -m pytest tests)."""
import gc
import time

import pytest

from fake_remote import FakeRemote, FakeSurface, load, robot_world
from ramms_tools.control import (ControlAxis, ControlError, ControlSurface, KeepAlive, StatusText, release_all,
                                 severity_name)
from ramms_tools.unreal_remote import UnrealRemote


def surface(gen="gen1", deadman=True):
    ue, path, comps = robot_world(gen, deadman=deadman)
    cs = ControlSurface.find(ue)[0]
    return cs, comps["ControlSurface"], ue


def test_call_full_keeps_out_parameters():
    ue, path, _ = robot_world("gen1")
    curb = path + ".CurbController"
    assert ue._call_function(curb, "GetStatusLine") == "Ready"              # unchanged behaviour
    assert ue.call_full(curb, "GetStatusLine") == {"OutSeverity": "Info", "ReturnValue": "Ready"}
    assert ue.call_full(curb, "GetCrossing") == {"OutStepCm": 0, "OutCrossingHeightCm": 0}   # void + outs
    assert ue.actor(curb).call_full("GetPhase") == {"ReturnValue": "Idle"}


def test_call_full_wraps_a_bare_response():
    class Bare(UnrealRemote):
        def _request(self, method, endpoint, body=None):
            return ["a", "b"]
    assert Bare().call_full("/x", "F") == {"ReturnValue": ["a", "b"]}


def test_find_and_describe_gen1():
    cs, _, ue = surface("gen1")
    assert cs.actor_path.endswith("BP_RammpGen1PoweredFront_Ramms_C_0")
    assert cs.robot_name == "BP_RammpGen1PoweredFront_Ramms_C_0"
    assert cs.groups()[:3] == ["Drive", "Lift", "Casters"]
    fwd = cs.axis("drive.forward")
    assert (fwd.kind, fwd.units, fwd.range, fwd.paired_axis, fwd.is_continuous) == ("Continuous", "Normalized", (-1, 1), "drive.turn", True)
    mode = cs.axis("drive.mode")
    assert mode.is_enum and mode.enum_labels == ["Curb crossing", "Differential"] and mode.default == 1
    phase = cs.axis("curb.phase")
    assert phase.read_only and len(phase.enum_labels) == 19 and phase.paired_axis == ""
    status = cs.axis("curb.status")
    assert status.is_text and not status.commandable
    assert [a.id for a in cs.axes("Curb")][:2] == ["curb.negotiate", "curb.abort"]   # sorted by order
    assert cs.axis("DRIVE.FORWARD").id == "drive.forward"                             # FNames ignore case


def test_gen2_paired_outline_and_find_by_robot():
    ue, path, _ = robot_world("gen2")
    cs = ControlSurface.find_by_robot(ue, "gen2robot")
    assert cs is not None and cs.robot_name == "Gen2Robot_C_0"
    h = cs.axis("seat.height")
    assert h.paired_axis == "seat.fore_aft" and len(h.region_outline) == 8 and h.units == "Centimeters"
    assert cs.axis("drive.mode").enum_labels == ["Curb", "Differential", "Holonomic", "Low level"]
    assert ControlSurface.find_by_robot(ue, "nothing-like-it") is None


def test_find_by_robot_exact_first_and_ambiguous_substrings():
    ue, path, comps = robot_world("gen1")
    ue.add_actor("BP_RammpGen1_Ramms_C_0", {"ControlSurface": ("RammsRobotControlSurfaceComponent",
                                                               FakeSurface(dict(load("gen1", "surface.json"), robotName="Chair")))})
    assert ControlSurface.find_by_robot(ue, "BP_RammpGen1_Ramms_C_0").actor_path.endswith("BP_RammpGen1_Ramms_C_0")
    assert ControlSurface.find_by_robot(ue, "chair").actor_path.endswith("BP_RammpGen1_Ramms_C_0")    # robotName
    assert ControlSurface.find_by_robot(ue, "poweredfront").actor_path == path
    with pytest.raises(ControlError, match="matches 2 robots"):
        ControlSurface.find_by_robot(ue, "RammpGen1")


def test_find_says_when_the_bridge_cannot_be_asked():
    ue, path, _ = robot_world("gen1")
    ue.bridge_down = True
    with pytest.raises(ControlError, match="could not list the robots.*not allowed"):
        ControlSurface.find(ue)


def test_unknown_control_rereads_then_raises():
    cs, fake, ue = surface()
    cs.describe()
    n = len(ue.calls_to("GetControlSurfaceJson"))
    with pytest.raises(ControlError, match="no control 'drive.warp'"):
        cs.set("drive.warp", 1.0)
    assert len(ue.calls_to("GetControlSurfaceJson")) == n + 1        # read once more before giving up
    assert fake.sets == []


def test_set_get_target_release():
    cs, fake, _ = surface()
    assert cs.set("lift.dw_main_plate_l", 12.5)
    assert fake.sets[-1] == ("lift.dw_main_plate_l", 12.5, "Remote")
    assert cs.get("lift.dw_main_plate_l") == 12.5
    assert cs.target("lift.dw_main_plate_l") == 12.5
    cs.release("lift.dw_main_plate_l")
    assert cs.target("lift.dw_main_plate_l") is None
    cs.set("drive.forward", 0.5, source="Autonomy")
    assert fake.sets[-1][2] == "Autonomy"


def test_read_only_and_actions_are_refused():
    cs, fake, _ = surface()
    with pytest.raises(ControlError, match="read-only"):
        cs.set("gripper.closed", 1.0)
    with pytest.raises(ControlError, match="read-only"):
        cs.set_enum("curb.phase", "Done")
    with pytest.raises(ControlError, match="Action"):
        cs.set("camera.next", 1.0)
    with pytest.raises(ControlError, match="read-only"):
        cs.trigger("curb.status")
    assert fake.sets == []
    assert cs.trigger("camera.next") and fake.triggered == ["camera.next"]


def test_enums_by_label_index_and_prefix():
    cs, fake, _ = surface()
    assert cs.get_enum("curb.assist") == "Off"
    cs.set_enum("curb.assist", "on")
    assert fake.values["curb.assist"] == 1.0 and cs.get_enum("curb.assist") == "On"
    cs.set_enum("curb.style", 0)
    assert cs.get_enum("curb.style") == "Stop and step"
    cs.set_enum("curb.style", "roll")                        # unique prefix
    assert cs.get_enum("curb.style") == "Rolling"
    cs.set("drive.mode", "curb crossing")                    # set() takes labels for an Enum too
    assert fake.values["drive.mode"] == 0.0
    with pytest.raises(ControlError, match="choices: Off, On"):
        cs.set_enum("curb.assist", "maybe")
    with pytest.raises(ControlError, match="not an Enum"):
        cs.set_enum("drive.forward", "On")
    a = ControlAxis(id="x", kind="Enum", enum_labels=["Off", "Live", "Live + trail"])
    assert a.enum_index("live + TRAIL") == 2 and a.enum_index("2") == 2 and a.enum_label(1.0) == "Live"
    assert a.enum_index("live") == 1                         # an exact label wins over a prefix
    with pytest.raises(ControlError):
        a.enum_index("liv")                                  # ambiguous prefix: Live / Live + trail
    with pytest.raises(ControlError):
        a.enum_index(5)


def test_text_status_and_severity():
    cs, fake, _ = surface()
    fake.texts["curb.status"] = "Waiting: the front omni wheels to clear the curb"
    fake.values["curb.status"] = 1.0
    st = cs.status("curb.status")
    assert st == StatusText("Waiting: the front omni wheels to clear the curb", "Attention")
    text, sev = st
    assert sev == "Attention" and str(st).startswith("[Attention]")
    assert cs.read("curb.status") == st and cs.read("curb.assist") == "Off" and cs.read("drive.forward") == 0.0
    assert severity_name(2) == "Fault" and severity_name("ERammsControlTextSeverity::Attention") == "Attention"
    assert severity_name(7) == "Fault" and severity_name("Info") == "Info"
    rb = cs.readback("Curb")
    assert "curb.negotiate" not in rb and rb["curb.phase"] == "Idle" and rb["curb.status"].severity == "Attention"


def test_unreadable_values_do_not_crash_reads():
    """GetControlValue answering with something that is not a number (an empty body is {}) reads as NaN."""
    assert severity_name(float("nan")) == "Info" and severity_name(None) == "Info" and severity_name(float("inf")) == "Info"
    assert ControlAxis(id="x", kind="Enum", enum_labels=["a"]).enum_label(float("nan")) == "nan"
    cs, fake, ue = surface()
    real = ue.actors[cs.actor_path]["ControlSurface"][1]
    ue.actors[cs.actor_path]["ControlSurface"] = ("RammsRobotControlSurfaceComponent",
                                                  lambda f, p: {} if f == "GetControlValue" else real(f, p))
    assert cs.status("curb.status") == StatusText("Ready", "Info") and cs.get_enum("curb.assist") == "nan"


def test_hold_resends_then_releases():
    cs, fake, _ = surface()
    ticks = []
    n = cs.hold({"drive.forward": 0.4, "drive.turn": -0.2}, seconds=0.35, rate=20.0, on_tick=ticks.append)
    sent = [s for s in fake.sets if s[0] == "drive.forward"]
    assert n >= 5 and len(sent) == n and len(ticks) == n
    assert all(v == 0.4 for _, v, _ in sent)
    assert set(fake.released[-2:]) == {"drive.forward", "drive.turn"}
    assert cs.last_hold_refused == {}


def test_hold_stops_early_and_refuses_read_only():
    cs, fake, _ = surface()
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] > 2
    n = cs.hold({"drive.forward": 0.1}, seconds=5.0, rate=50.0, should_stop=stop)
    assert n == 2 and fake.released[-1] == "drive.forward"
    with pytest.raises(ControlError):
        cs.hold({"curb.phase": 1}, seconds=1.0)               # read-only: refused before anything is sent


def test_hold_counts_refusals():
    cs, fake, _ = surface()
    fake.refuse.add("drive.turn")
    n = cs.hold({"drive.forward": 0.1, "drive.turn": 0.2}, seconds=0.12, rate=50.0)
    assert cs.last_hold_refused == {"drive.turn": n}
    assert cs.drive(0.1, 0.2, seconds=0.05, rate=50.0)       # forward was taken
    fake.refuse.add("drive.forward")
    assert not cs.drive(0.1, 0.2, seconds=0.05, rate=50.0)   # nothing was


@pytest.mark.parametrize("deadman", [True, False])
def test_hold_releases_when_a_send_raises_or_is_interrupted(deadman):
    """On a latching sim (deadman False) an unreleased drive command would keep the robot moving."""
    cs, fake, ue = surface(deadman=deadman)
    sends = {"n": 0}

    def flaky(func, p):
        if func == "SetControl" and p["Id"] == "drive.turn":
            sends["n"] += 1
            if sends["n"] == 3:
                raise KeyboardInterrupt
    fake.on_call = flaky
    with pytest.raises(KeyboardInterrupt):
        cs.hold({"drive.forward": 0.4, "drive.turn": 0.2}, seconds=5.0, rate=50.0)
    assert fake.latched() == {}
    assert {"drive.forward", "drive.turn"} <= set(fake.released)


def test_drive_needs_seconds_or_once():
    cs, fake, _ = surface("gen1", deadman=False)
    with pytest.raises(ValueError, match="seconds="):
        cs.drive(0.2, 0.1)
    assert fake.sets == []
    assert cs.drive(0.2, 0.1, once=True)                      # a single command, explicitly
    assert [s[0] for s in fake.sets[-2:]] == ["drive.forward", "drive.turn"] and fake.released == []
    assert fake.latched() == {"drive.forward": 0.2, "drive.turn": 0.1}      # an older sim keeps it ...
    cs.stop()                                                  # ... until it is released
    assert fake.latched() == {} and set(fake.released) == {"drive.forward", "drive.turn", "drive.strafe"}


def test_drive_strafe_needs_a_holonomic_surface():
    cs, fake, _ = surface("gen1")
    cs.drive(0.2, 0.0, strafe=0.0, once=True)                 # 0 strafe is fine without the control
    with pytest.raises(ControlError, match="no drive.strafe"):
        cs.drive(0.2, 0.0, strafe=0.5, seconds=0.1)
    # A holonomic mode adds drive.strafe to the surface.
    fake.surface["axes"].append(dict(fake.surface["axes"][1], id="drive.strafe", displayName="Strafe", order=2))
    cs.describe(refresh=True)
    cs.drive(0.1, 0.0, strafe=0.3, seconds=0.15, rate=20.0)
    assert ("drive.strafe", 0.3, "Remote") in fake.sets and "drive.strafe" in fake.released


def test_stop_releases_strafe_even_with_a_stale_description():
    """Another client switched to a holonomic mode after this surface cached its description."""
    cs, fake, ue = surface("gen1", deadman=False)
    cs.describe()
    fake.surface["axes"].append(dict(fake.surface["axes"][1], id="drive.strafe", displayName="Strafe", order=2))
    fake("SetControl", {"Id": "drive.strafe", "Value": 0.4, "Source": "Remote"})     # the other client
    assert fake.latched() == {"drive.strafe": 0.4}
    cs.stop()
    assert fake.latched() == {}


def test_a_single_set_is_released_at_exit_and_when_the_surface_goes():
    cs, fake, ue = surface(deadman=False)
    cs.set("drive.forward", 0.3)
    assert fake.latched() == {"drive.forward": 0.3}
    release_all()                                             # what atexit runs
    assert fake.latched() == {}
    cs.set("drive.turn", -0.2)
    assert fake.latched() == {"drive.turn": -0.2}
    del cs
    gc.collect()                                              # its finalizer releases what it set once
    assert fake.latched() == {}


def test_deadman_fake_springs_back_and_latching_fake_does_not():
    cs, fake, _ = surface(deadman=True)
    cs.set("drive.forward", 0.3)
    assert fake.latched() == {"drive.forward": 0.3}
    time.sleep(0.4)
    assert fake.latched() == {} and fake.lapsed == ["drive.forward"]
    cs2, fake2, _ = surface(deadman=False)
    cs2.set("drive.forward", 0.3)
    time.sleep(0.4)
    assert fake2.latched() == {"drive.forward": 0.3}
    cs2.release_once()
    assert fake2.latched() == {}


def test_keep_alive_updates_and_releases():
    cs, fake, _ = surface()
    with cs.keep_alive({"drive.forward": 0.3}, rate=50.0) as ka:
        time.sleep(0.1)
        ka.update({"drive.turn": 0.5})
        time.sleep(0.1)
        assert ka.values == {"drive.forward": 0.3, "drive.turn": 0.5}
    assert ka.sends >= 4
    assert ("drive.turn", 0.5, "Remote") in fake.sets
    assert {"drive.forward", "drive.turn"} <= set(fake.released)


def _slow(fake, ids, seconds):
    def delay(func, p):
        if func == "SetControl" and p["Id"] in ids:
            time.sleep(seconds)
    fake.on_call = delay


def test_keep_alive_replace_never_resends_a_dropped_control():
    cs, fake, _ = surface(deadman=False)
    _slow(fake, {"drive.forward", "drive.turn"}, 0.02)
    for trial in range(8):
        fake.events.clear()
        ka = cs.keep_alive({"drive.forward": 0.3, "drive.turn": 0.4}, rate=50).start()
        time.sleep(0.03 + trial * 0.007)
        ka.update({"drive.forward": 0.3}, replace=True)
        time.sleep(0.1)
        released_at = fake.events.index(("release", "drive.turn"))
        late = [e for e in fake.events[released_at:] if e == ("set", "drive.turn")]
        assert late == [], f"trial {trial}: drive.turn sent after its release"
        assert "drive.turn" not in fake.latched()
        ka.stop()
    assert fake.latched() == {}


def test_keep_alive_stop_waits_for_a_slow_send_and_releases_last():
    """An engine hitch longer than the join: the release still comes after the last send."""
    cs, fake, _ = surface(deadman=False)
    ka = cs.keep_alive({"drive.forward": 0.3, "drive.turn": 0.2}, rate=20).start()
    time.sleep(0.12)
    _slow(fake, {"drive.forward"}, 0.6)
    time.sleep(0.06)                                          # a send of drive.forward is in flight
    ka.join_timeout = 0.1                                     # shorter than the hitch
    ka.stop()
    time.sleep(1.0)                                           # the sender comes back and releases
    last_set = max(i for i, (kind, cid) in enumerate(fake.events) if kind == "set")
    first_final_release = min(i for i, (kind, cid) in enumerate(fake.events) if kind == "release")
    assert first_final_release > last_set and fake.latched() == {} and not ka.running


def test_keep_alive_pause_resume_and_refusals():
    cs, fake, _ = surface(deadman=False)
    fake.refuse.add("drive.turn")
    ka = cs.keep_alive({"drive.forward": 0.2, "drive.turn": 0.1}, rate=50).start()
    time.sleep(0.08)
    ka.pause()
    n = len(fake.set_times)
    assert fake.latched() == {} and ka.paused
    time.sleep(0.08)
    assert len(fake.set_times) == n                           # nothing sent while paused
    ka.resume()
    time.sleep(0.08)
    assert fake.latched() == {"drive.forward": 0.2} and ka.refused["drive.turn"] >= 2
    ka.stop()
    assert fake.latched() == {}


def test_keep_alive_left_running_is_stopped_at_exit():
    cs, fake, _ = surface(deadman=False)
    ka = cs.keep_alive({"drive.forward": 0.25}, rate=50).start()
    time.sleep(0.05)
    assert fake.latched() == {"drive.forward": 0.25}
    release_all()                                             # the atexit hook
    assert not ka.running and fake.latched() == {}


def test_bad_source_is_refused():
    ue = FakeRemote()
    with pytest.raises(ValueError):
        ControlSurface(ue, "/a.b", source="Telepathy")
    with pytest.raises(ValueError):
        KeepAlive(ControlSurface(ue, "/a.b"), {}, rate=0)
