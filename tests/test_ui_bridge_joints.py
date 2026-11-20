"""Offline tests for the joint checks in ramms_tools.scripts.ui_bridge_test.

FakeEngine stands in for a RammpUI editor and models what was MEASURED on one (2026-11-19):

- a preset entry names the editor's level, and a preset call made during play writes the
  EDITOR's copy of the actor -- the running robot does not move;
- the play-in-editor copy lives at the same path with the package renamed UEDPIE_0_<map>;
- Remote Control's preset description lists no arguments for SetAllJointTargets;
- an unrecognised parameter is accepted and arrives empty;
- joints with enforced limits clamp; the monitor classifies each write (arm Stowed/Active,
  wheels moving until WheelIdleTimeout passes with no wheel-target change);
- the HUD's status bar sits under GameInstance_<n>, n unknown, readable in one /remote/batch.
"""
from __future__ import annotations

import copy
import urllib.parse

import pytest

from ramms_tools.scripts import ui_bridge_test as ubt
from ramms_tools.scripts.ui_bridge_test import (
    BridgeClient,
    BridgeError,
    JointRig,
    JointsUnavailable,
    active_offset,
    joint_index,
    pie_path,
    plain_text,
    run_joint_checks,
    with_joints,
)

EDITOR = "/Game/Maps/Map_UI.Map_UI:PersistentLevel."
PIE = "/Game/Maps/UEDPIE_0_Map_UI.Map_UI:PersistentLevel."
ACTOR = "BP_RAMMP_RemoteDisplay_LeftArm_C_1"
POSE = f"{ACTOR}.RammsSkeletalPose"
MONITOR = f"{ACTOR}.JointMotionMonitor"
RECEIVER = EDITOR + "RammsBridgeReceiver_1"
STOW = [22.9183, 0.0, 1.14592, 130.061, -89.9544, -48.1285, 112.873]
ARM = [f"link_{i}" for i in range(7)]
WHEELS = ["drive_wheel_l", "drive_wheel_r"]
GRIPPER = ["link_0_l", "link_3_l", "link_0_r", "link_3_r", "link_2_l", "link_2_r",
           "UserConstraint", "UserConstraint_0"]
BRIDGE_ARGS = {"UpdatePanel": ["PanelJSON"], "UpdateDetectedObjects": ["DetectedObjectsJSON"],
               "UpdateWorldState": ["WorldStateJSON"], "PollUIEvents": ["SinceSequence"],
               "OpenCameraFeed": [], "CloseCameraFeed": []}
LIMITS = {"link_1": 128.9, "link_3": 145.8, "link_5": 120.3, "link_0_l": 45.0}


def _joint_names():
    names = [f"base_{i}" for i in range(38)]
    names[2], names[9] = WHEELS
    return names + GRIPPER + ARM  # 53, laid out as on the RAMMP display


class Clock:
    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Display:
    """One copy of the robot display: pose component, monitor, and (optionally) its status bar."""

    def __init__(self, clock):
        self.clock = clock
        self.joints = [{"JointName": n, "JointType": "Revolute (Rotation)", "bEnforceLimits": n in LIMITS,
                        "MinValue": -LIMITS.get(n, 180.0), "MaxValue": LIMITS.get(n, 180.0),
                        "TargetValue": 0.0, "CurrentValue": 0.0} for n in _joint_names()]
        self.index = {j["JointName"]: i for i, j in enumerate(self.joints)}
        self.arm_state = "Active"  # zero targets at BeginPlay
        self.last_wheel_change = None
        self.monitor_stuck = False
        self.skeleton_stuck = False
        self.writes = 0

    def set_all(self, values):
        self.writes += 1
        before = [self.joints[self.index[w]]["TargetValue"] for w in WHEELS]
        for joint, value in zip(self.joints, values):  # a short array writes only a prefix
            if joint["bEnforceLimits"]:
                value = min(max(value, joint["MinValue"]), joint["MaxValue"])
            joint["TargetValue"] = value
            if not self.skeleton_stuck:
                joint["CurrentValue"] = value  # interpolation off: snaps
        if self.monitor_stuck:
            return
        arm = [self.joints[self.index[n]]["TargetValue"] for n in ARM]
        self.arm_state = "Stowed" if all(abs(a - s) <= 5.0 for a, s in zip(arm, STOW)) else "Active"
        after = [self.joints[self.index[w]]["TargetValue"] for w in WHEELS]
        if any(abs(a - b) > 0.5 for a, b in zip(after, before)):
            self.last_wheel_change = self.clock.now

    def moving(self):
        return self.last_wheel_change is not None and self.clock.now - self.last_wheel_change < 0.5

    def read(self, component, prop):
        if component == "RammsSkeletalPose" and prop == "Joints":
            return copy.deepcopy(self.joints)
        if component == "JointMotionMonitor":
            return {"ArmJointNames": ARM, "StowedPose": STOW, "StowedTolerance": 5.0,
                    "WheelJointNames": WHEELS, "WheelMotionThreshold": 0.5, "WheelIdleTimeout": 0.5,
                    "ArmPoseState": self.arm_state, "bWheelsMoving": self.moving()}[prop]
        raise KeyError(prop)


class FakeEngine:
    """mode: "pie" (editor, playing), "editor" (nothing playing), "packaged", or "game" -- the
    editor binary run with -game, which has GameEngine_0 like a package but also every UnrealEd
    class default object (the editor binary loads UnrealEd whatever the mode)."""

    def __init__(self, mode="pie", status_bar=True, instance=8, joints_exposed=True, joints_bound=True,
                 pie_instance=0, engines=True):
        self.mode = mode
        self.pie = f"/Game/Maps/UEDPIE_{pie_instance}_Map_UI.Map_UI:PersistentLevel."
        self.engines = engines  # False: neither engine object answers (an unknown host)
        self.clock = Clock()
        self.editor_copy = Display(self.clock) if mode in ("pie", "editor") else None
        self.game = Display(self.clock) if mode in ("pie", "packaged", "game") else None
        self.status_bar = status_bar
        self.status_default = None  # e.g. "Active" to model a bar stuck on its designer text
        engine = "GameEngine_0" if mode in ("packaged", "game") else "UnrealEdEngine_0"
        self.status_root = (f"/Engine/Transient.{engine}:GameInstance_{instance}.W_RAMMS_UI_C_0."
                            "WidgetTree_0.RAMMS_StatusBar.WidgetTree_0.")
        self.joints_exposed = joints_exposed
        self.joints_bound = joints_bound
        self.log = []  # (method, endpoint, payload)
        self.fail_next_batch = False  # one /remote/batch answers nothing at all
        self.drop_writes_after = None  # writes past this many are accepted and land nowhere

    # what lives where
    def _display_at(self, level):
        if self.mode == "pie":
            return {EDITOR: self.editor_copy, self.pie: self.game}.get(level)
        if self.mode == "editor":
            return self.editor_copy if level == EDITOR else None
        return self.game if level == EDITOR else None  # a game: the map is loaded at its own path

    def read(self, path, prop):
        if self.engines and path == "/Engine/Transient.UnrealEdEngine_0" and prop == "PlayWorld" \
                and self.mode in ("pie", "editor"):
            return self.pie.partition(":")[0] if self.mode == "pie" else ""
        if self.engines and path == "/Engine/Transient.GameEngine_0" and prop == "bUseFixedFrameRate" \
                and self.mode in ("packaged", "game"):
            return False
        if path == "/Script/UnrealEd.Default__EditorLoadingSavingSettings" and prop == "bAutoSaveEnable" \
                and self.mode != "packaged":
            return True  # loaded by any editor binary, -game included: not a test for "editor"
        if path.startswith(self.status_root) and prop == "Text" and self.status_bar and self.game:
            widget = path[len(self.status_root):]
            if widget == "TXT_ArmStatus":
                return f"<status-default>{self.status_default or self.game.arm_state}</>"
            if widget == "TXT_ChairStatus":
                return f"<status-emphasis>{'DRIVING' if self.game.moving() else 'PARKED'}</>"
        for level in (EDITOR, self.pie):
            if path.startswith(level):
                display = self._display_at(level)
                rest = path[len(level):]
                if display and rest in (POSE, MONITOR):
                    try:
                        return display.read(rest.rpartition(".")[2], prop)
                    except KeyError:
                        break
        raise BridgeError(f"Object: {path} could not be resolved")

    def preset(self):
        functions = [{"DisplayName": label,
                      "UnderlyingFunction": {"Name": name, "Arguments": [{"Name": a} for a in BRIDGE_ARGS[name]]},
                      "OwnerObjects": [{"Path": RECEIVER}]} for name, label in ubt.FALLBACK_LABELS.items()]
        if self.joints_exposed:
            functions.append({"DisplayName": "Set All Joint Targets",
                              "UnderlyingFunction": {"Name": "SetAllJointTargets", "Arguments": []},
                              "OwnerObjects": [{"Path": EDITOR + POSE}] if self.joints_bound else []})
        return {"Preset": {"Name": "RCPS", "Groups": [{"Name": "All", "ExposedProperties": [],
                                                       "ExposedFunctions": functions}]}}

    def request(self, method, endpoint, payload=None):
        self.log.append((method, endpoint, payload))
        if method == "GET" and endpoint == "/remote/presets":
            return {"Presets": [{"Name": "RCPS"}]}
        if method == "GET" and endpoint == "/remote/preset/RCPS":
            return self.preset()
        if endpoint == "/remote/object/property":
            prop = payload["propertyName"]
            return {prop: self.read(payload["objectPath"], prop)}
        if endpoint == "/remote/batch":
            if self.fail_next_batch:
                self.fail_next_batch = False
                raise BridgeError("PUT /remote/batch failed: timed out")
            out = []
            for r in payload["Requests"]:
                try:
                    body = {r["Body"]["propertyName"]: self.read(r["Body"]["objectPath"], r["Body"]["propertyName"])}
                    out.append({"RequestId": r["RequestId"], "ResponseCode": 200, "ResponseBody": body})
                except BridgeError as exc:
                    out.append({"RequestId": r["RequestId"], "ResponseCode": 400,
                                "ResponseBody": {"errorMessage": str(exc)}})
            return {"Responses": out}
        if endpoint == "/remote/object/call":
            path = payload["objectPath"]
            for level in (EDITOR, self.pie):
                display = self._display_at(level) if path == level + POSE else None
                if display:
                    if self.drop_writes_after is not None and display.writes >= self.drop_writes_after:
                        return {"Values": []}  # accepted, and nothing lands
                    # UE matches parameter names case-insensitively and ignores the rest.
                    values = next((v for k, v in payload["parameters"].items() if k.lower() == "values"), [])
                    display.set_all(values)
                    return {"Values": values}
            raise BridgeError(f"Object: {path} could not be resolved")
        if endpoint == "/remote/preset/RCPS/function/" + urllib.parse.quote("Set All Joint Targets"):
            if not self.joints_bound:
                return {"ReturnedValues": [{}]}
            values = next((v for k, v in payload["Parameters"].items() if k.lower() == "values"), [])
            # The binding names the editor level, and resolves THERE even while playing.
            target = self.game if self.mode in ("packaged", "game") else self.editor_copy
            target.set_all(values)
            return {"ReturnedValues": [{"Values": values}]}
        if endpoint.startswith("/remote/preset/RCPS/function/"):
            if "Poll" in endpoint:
                return {"ReturnedValues": [{"LatestSequence": 0, "Events": []}]}
            return {"ReturnedValues": [{}]}
        raise AssertionError(f"unexpected request {method} {endpoint}")


def _client(engine):
    client = BridgeClient()
    client._request = engine.request
    return client


def _checks():
    results = []

    def check(ok, what, detail=""):
        results.append((bool(ok), what, detail))
    return check, results


def _run(engine, rig):
    check, results = _checks()
    run_joint_checks(rig, check, sleep=engine.clock.sleep, clock=engine.clock.time)
    return results


# ── pure helpers ─────────────────────────────────────────────────────

def test_pie_path_renames_only_the_map_package():
    assert pie_path(EDITOR + POSE) == PIE + POSE
    assert pie_path(PIE + POSE) == PIE + POSE  # already a play-session path
    assert pie_path("/Script/RammsCore.Default__RammsCoreBridge") == "/Script/RammsCore.Default__RammsCoreBridge"
    assert pie_path("NoDots") == "NoDots"
    assert pie_path("/Game/UI/RCPS.RCPS") == "/Game/UI/RCPS.RCPS"  # an asset, not a level object


def test_plain_text_strips_rich_text_tags():
    assert plain_text("<status-default>Stowed</>") == "Stowed"
    assert plain_text("<status-emphasis>PARKED</>") == "PARKED"
    assert plain_text(None) == ""


def test_with_joints_matches_names_exactly_and_holds_the_rest():
    joints = Display(Clock()).joints
    index = joint_index(joints)
    base = [float(i) for i in range(len(joints))]
    out = with_joints(base, index, {"link_0": 99.0})
    assert out[index["link_0"]] == 99.0
    # The gripper's link_0_l is a different joint and must not move with the arm's link_0.
    assert out[index["link_0_l"]] == base[index["link_0_l"]]
    assert sum(a != b for a, b in zip(out, base)) == 1


def test_with_joints_refuses_a_joint_the_display_does_not_have():
    joints = Display(Clock()).joints
    with pytest.raises(BridgeError, match="link_7"):
        with_joints([0.0] * len(joints), joint_index(joints), {"link_7": 1.0})


def test_active_offset_moves_one_joint_far_past_the_tolerance_within_limits():
    joints = Display(Clock()).joints
    index = joint_index(joints)
    nudge = active_offset(joints, index, ARM, STOW, 5.0)
    (name, value), = nudge.items()
    base = STOW[ARM.index(name)]
    assert abs(value - base) >= 30.0
    assert ubt.within_limits(joints[index[name]], value)


def test_active_offset_skips_a_direction_its_limits_would_clamp():
    joints = Display(Clock()).joints
    index = joint_index(joints)
    # Only link_3 offered, stowed at 130 with a 145.8 limit: +30 would clamp, so it goes down.
    assert active_offset(joints, index, ["link_3"], [130.061], 5.0) == {"link_3": pytest.approx(100.061)}


# ── the argument Remote Control does not report ─────────────────────

def test_values_is_accepted_although_the_preset_declares_no_arguments():
    engine = FakeEngine(mode="packaged")
    client = _client(engine)
    client.load_preset()
    assert client._arguments["SetAllJointTargets"] == []
    client.call("SetAllJointTargets", Values=[0.0] * 53)
    assert engine.game.writes == 1


def test_a_misspelled_values_is_still_refused_before_sending():
    engine = FakeEngine(mode="packaged")
    client = _client(engine)
    client.load_preset()
    with pytest.raises(BridgeError):
        client.call("SetAllJointTargets", Value=[0.0] * 53)
    assert engine.game.writes == 0


# ── finding the live display ─────────────────────────────────────────

def test_during_play_the_play_session_copy_is_used():
    rig = JointRig.locate(_client(FakeEngine(mode="pie")))
    assert rig.world == "play session"
    assert rig.pose_path == PIE + POSE
    assert rig.monitor_path == PIE + MONITOR


def test_in_the_editor_with_nothing_playing_nothing_is_written():
    engine = FakeEngine(mode="editor")
    with pytest.raises(JointsUnavailable, match="not playing"):
        JointRig.locate(_client(engine))
    assert engine.editor_copy.writes == 0


@pytest.mark.parametrize("mode", ["packaged", "game"])
def test_a_game_uses_the_bound_path_as_is(mode):
    """`UnrealEditor -game` included: it has UnrealEd's settings objects, but no editor level."""
    rig = JointRig.locate(_client(FakeEngine(mode=mode)))
    assert rig.world == "game"
    assert rig.pose_path == EDITOR + POSE


def test_the_play_world_the_editor_reports_is_used_not_instance_0():
    engine = FakeEngine(mode="pie", pie_instance=1)
    rig = JointRig.locate(_client(engine))
    assert rig.pose_path == "/Game/Maps/UEDPIE_1_Map_UI.Map_UI:PersistentLevel." + POSE


def test_when_neither_engine_answers_nothing_is_sent():
    """Guessing "game" in an editor would send through the preset into the saved level."""
    engine = FakeEngine(mode="editor", engines=False)
    with pytest.raises(BridgeError, match="cannot tell"):
        JointRig.locate(_client(engine))
    assert engine.editor_copy.writes == 0


def test_play_copy_only_maps_into_the_map_being_played():
    assert ubt.play_copy(EDITOR + POSE, "/Game/Maps/UEDPIE_2_Map_UI.Map_UI") == \
        "/Game/Maps/UEDPIE_2_Map_UI.Map_UI:PersistentLevel." + POSE
    assert ubt.play_copy(EDITOR + POSE, "/Game/Maps/UEDPIE_0_Other.Other") is None
    assert ubt.play_copy(EDITOR + POSE, "") is None


def test_a_preset_without_joints_is_a_skip_not_a_failure():
    with pytest.raises(JointsUnavailable):
        JointRig.locate(_client(FakeEngine(joints_exposed=False)))


def test_an_unbound_joint_entry_names_what_to_rebind_it_to():
    with pytest.raises(BridgeError, match="RammsSkeletalPose component of the robot display"):
        JointRig.locate(_client(FakeEngine(joints_bound=False)))


# ── sending ──────────────────────────────────────────────────────────

def test_in_play_targets_go_to_the_game_not_through_the_preset():
    """The measured bug this avoids: a preset call during play moved the editor's actor."""
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    rig.send(with_joints(rig.targets(), rig.index, {"link_0": 12.5}))
    assert engine.game.joints[engine.game.index["link_0"]]["TargetValue"] == 12.5
    assert engine.editor_copy.writes == 0
    assert not any("/remote/preset/RCPS/function/" in e for _, e, _ in engine.log)


def test_in_a_packaged_game_targets_go_through_the_preset():
    engine = FakeEngine(mode="packaged")
    rig = JointRig.locate(_client(engine))
    rig.send(with_joints(rig.targets(), rig.index, {"link_0": 12.5}))
    assert engine.game.joints[engine.game.index["link_0"]]["TargetValue"] == 12.5
    assert any(e.endswith(urllib.parse.quote("Set All Joint Targets")) for _, e, _ in engine.log)


def test_a_short_vector_is_refused_before_it_is_sent():
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    with pytest.raises(BridgeError, match="short"):
        rig.send([0.0] * 7)
    assert engine.game.writes == 0


def test_targets_that_land_somewhere_else_are_caught_by_the_readback():
    """Model the preset-in-PIE route: the write lands on the editor copy, the game is read."""
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    rig.world = "game"  # force the preset route
    with pytest.raises(BridgeError, match="did not land"):
        rig.send(with_joints(rig.targets(), rig.index, {"link_0": 12.5}))
    assert engine.editor_copy.writes == 1


def test_a_clamped_joint_is_not_reported_as_a_failed_landing():
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    rig.send(with_joints(rig.targets(), rig.index, {"link_1": 500.0}))  # clamps to 128.9


# ── the whole sequence ───────────────────────────────────────────────

def test_the_full_sequence_passes_on_a_working_display():
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    results = _run(engine, rig)
    failed = [(what, detail) for ok, what, detail in results if not ok]
    assert not failed
    names = [what for _, what, _ in results]
    for expected in ("arm stowed: monitor says Stowed", "arm active: status bar reads 'Active'",
                     "wheels turning: status bar reads 'DRIVING'", "wheels stopped: status bar reads 'PARKED'"):
        assert expected in names


def test_the_sequence_restores_the_arm_and_leaves_the_wheels_where_they_stopped():
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    before = rig.targets()
    _run(engine, rig)
    after = rig.targets()
    for name in ARM:
        assert after[rig.index[name]] == before[rig.index[name]]
    for name in WHEELS:
        assert after[rig.index[name]] > before[rig.index[name]]  # not spun back
    engine.clock.sleep(0.1)
    assert not engine.game.moving()  # so the chair is not left reading DRIVING


def test_a_monitor_that_never_reacts_fails_the_indicator_checks():
    engine = FakeEngine(mode="pie")
    engine.game.monitor_stuck = True
    rig = JointRig.locate(_client(engine))
    failed = {what for ok, what, _ in _run(engine, rig) if not ok}
    assert "arm stowed: monitor says Stowed" in failed
    assert "wheels turning: monitor says moving" in failed
    assert "wheels turning: status bar reads 'DRIVING'" in failed


def test_a_skeleton_that_does_not_follow_its_targets_fails():
    engine = FakeEngine(mode="pie")
    engine.game.skeleton_stuck = True
    rig = JointRig.locate(_client(engine))
    failed = {what for ok, what, _ in _run(engine, rig) if not ok}
    assert "arm stowed: arm joints moved to their targets" in failed


def test_a_status_bar_stuck_on_its_designer_text_fails():
    engine = FakeEngine(mode="pie")
    engine.status_default = "Active"
    rig = JointRig.locate(_client(engine))
    failed = {what for ok, what, _ in _run(engine, rig) if not ok}
    assert "arm stowed: status bar reads 'Stowed'" in failed
    assert "arm stowed: monitor says Stowed" not in failed  # the monitor was right


def test_a_status_bar_that_cannot_be_found_is_one_failure_not_a_quiet_pass():
    engine = FakeEngine(mode="pie", status_bar=False)
    rig = JointRig.locate(_client(engine))
    results = _run(engine, rig)
    failed = [what for ok, what, _ in results if not ok]
    assert failed == ["the HUD's status bar is readable"]
    assert not any("status bar reads" in what for _, what, _ in results)  # skipped, not passed


def test_one_dropped_probe_does_not_hide_the_status_bar_for_the_rest_of_the_run():
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    engine.fail_next_batch = True
    assert rig.status_text("TXT_ArmStatus") is None
    assert rig.status_text("TXT_ArmStatus") == "Active"


def test_the_status_bar_is_found_in_one_batch_whatever_the_instance_number():
    engine = FakeEngine(mode="pie", instance=700)
    rig = JointRig.locate(_client(engine))
    assert rig.status_text("TXT_ChairStatus") == "PARKED"
    assert sum(1 for _, e, _ in engine.log if e == "/remote/batch") == 1


def test_a_packaged_game_status_bar_is_found_too():
    engine = FakeEngine(mode="packaged", instance=0)
    rig = JointRig.locate(_client(engine))
    assert rig.status_text("TXT_ArmStatus") == "Active"


# ── selftest and the CLI ─────────────────────────────────────────────

class _Args:
    def __init__(self, **kw):
        defaults = dict(check=False, stow=False, activate=False, drive=False, sweep=False, set=None,
                        seconds=1.0, rate=10.0, step=5.0)
        defaults.update(kw)
        self.__dict__.update(defaults)


def _fake_time(monkeypatch, engine):
    monkeypatch.setattr(ubt, "_sleep", engine.clock.sleep)
    monkeypatch.setattr(ubt, "_clock", engine.clock.time)
    # This fake has a status bar but no panel or POI layer; those checks have their own fake HUD
    # (tests/test_ui_bridge_hud.py).
    monkeypatch.setattr(ubt, "run_style_checks", lambda *a, **k: [])
    monkeypatch.setattr(ubt, "run_poi_checks", lambda *a, **k: ([], {}))


def test_selftest_runs_the_joint_checks(monkeypatch, capsys):
    engine = FakeEngine(mode="pie")
    _fake_time(monkeypatch, engine)
    assert ubt.cmd_selftest(_client(engine), _Args()) == 0
    out = capsys.readouterr().out
    assert "joints" in out and "wheels stopped: status bar reads 'PARKED'" in out


def test_selftest_still_runs_the_bridge_checks_when_only_the_joints_are_unbound(monkeypatch, capsys):
    engine = FakeEngine(mode="pie", joints_bound=False)
    _fake_time(monkeypatch, engine)
    assert ubt.cmd_selftest(_client(engine), _Args()) == 1
    out = capsys.readouterr().out
    assert "pushes accepted" in out  # not skipped
    assert "FAIL  the robot display's joints are reachable" in out


def test_joints_set_moves_only_the_named_joint(monkeypatch, capsys):
    engine = FakeEngine(mode="pie")
    _fake_time(monkeypatch, engine)
    before = [j["TargetValue"] for j in engine.game.joints]
    assert ubt.cmd_joints(_client(engine), _Args(set=["link_1=40"])) == 0
    after = [j["TargetValue"] for j in engine.game.joints]
    changed = [engine.game.joints[i]["JointName"] for i, (a, b) in enumerate(zip(before, after)) if a != b]
    assert changed == ["link_1"]


def test_joints_check_exits_nonzero_when_an_indicator_is_wrong(monkeypatch):
    engine = FakeEngine(mode="pie")
    engine.status_default = "Active"
    _fake_time(monkeypatch, engine)
    assert ubt.cmd_joints(_client(engine), _Args(check=True)) == 1


def test_joints_refuses_in_the_editor_with_nothing_playing(monkeypatch, capsys):
    engine = FakeEngine(mode="editor")
    assert ubt.cmd_joints(_client(engine), _Args(check=True)) == 1
    assert engine.editor_copy.writes == 0
    assert "Start PIE" in capsys.readouterr().err


def test_a_restore_that_does_not_land_is_a_failure():
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    engine.drop_writes_after = 3 + 8  # three arm poses and eight drive frames land; the restore does not
    results = _run(engine, rig)
    failed = [what for ok, what, _ in results if not ok]
    assert failed == ["joint targets put back as found"]


def test_a_monitor_naming_a_joint_the_display_lacks_is_reported_not_a_crash():
    engine = FakeEngine(mode="pie")
    rig = JointRig.locate(_client(engine))
    rig.monitor = dict(rig.monitor, WheelJointNames=["drive_wheel_l", "caster_wheel_x"])
    results = _run(engine, rig)
    assert [(ok, what) for ok, what, _ in results] == [
        (False, "the display's joint-motion monitor watches joints it has")]
    assert "caster_wheel_x" in results[0][2]
    assert engine.game.writes == 0  # refused before anything moved


def test_selftest_skips_the_joints_when_the_editor_is_not_playing_and_says_so(monkeypatch, capsys):
    engine = FakeEngine(mode="editor")
    _fake_time(monkeypatch, engine)
    assert ubt.cmd_selftest(_client(engine), _Args()) == 0
    out = capsys.readouterr().out
    assert "skip  the editor is not playing" in out
    assert "joints NOT checked" in out.splitlines()[-1]
    assert engine.editor_copy.writes == 0
