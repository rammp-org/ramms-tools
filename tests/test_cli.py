"""Offline tests for the ramms-control / ramms-curb / ramms-sensors CLIs and the ramms-worldgen additions."""
import json
import signal
import time

import pytest

from fake_remote import FakeRemote, robot_world
from test_robot import scripted_phases
from test_sensing import FakeSensingBridge
from test_worldgen_terrain import GRADED, FakeWorldGenBridge, traffic_world
from ramms_tools.scripts import _common, control_cli, curb_cli, sensors_cli, worldgen_cli
from ramms_tools.sensing import BRIDGE as SENSING_BRIDGE
from ramms_tools.worldgen import BRIDGE as WORLDGEN_BRIDGE, WorldGen


@pytest.fixture
def world(monkeypatch):
    """Point the CLIs at a fake world: world("gen2") -> (remote, actor path, components)."""
    def make(gen="gen1", **kw):
        ue, path, comps = robot_world(gen, **kw)
        ue.objects[SENSING_BRIDGE] = FakeSensingBridge()
        monkeypatch.setattr(_common, "make_remote", lambda host, port, timeout: ue)
        return ue, path, comps
    return make


@pytest.mark.parametrize("main", [control_cli.main, curb_cli.main, sensors_cli.main])
def test_help(main, capsys):
    with pytest.raises(SystemExit) as e:
        main(["--help"])
    assert e.value.code == 0 and "--robot" in capsys.readouterr().out


@pytest.mark.parametrize("argv", [["terrain", "--help"], ["traffic", "--help"], ["manifest", "--help"]])
def test_worldgen_help(argv, capsys):
    with pytest.raises(SystemExit) as e:
        worldgen_cli.main(argv)
    out = capsys.readouterr().out
    assert e.value.code == 0
    if argv[0] == "terrain":
        assert "MaxStreetGrade" in out and "--root-heaves" in out
    if argv[0] == "manifest":
        assert "--terrain" in out and "--traffic" in out


def test_parsing():
    a = control_cli.build_parser().parse_args(["drive", "-f", "0.4", "--turn", "-0.1", "--seconds", "3", "--robot", "gen2"])
    assert (a.forward, a.turn, a.strafe, a.seconds, a.robot, a.source) == (0.4, -0.1, None, 3.0, "gen2", "Remote")
    a = control_cli.build_parser().parse_args(["--host", "10.0.0.5", "set", "x", "1", "--source", "Autonomy"])
    assert a.host == "10.0.0.5" and a.source == "Autonomy"
    a = curb_cli.build_parser().parse_args(["go", "--wait", "--port", "30011"])
    assert a.wait and a.port == 30011 and a.forward == 0.5 and not a.no_hold and a.on_interrupt == "abort"
    a = sensors_cli.build_parser().parse_args(["add", "cam", "--kind", "camera", "--at", "1,2,3", "--no-motion",
                                               "--json", '{"fov_deg": 70}', "--actor", "X"])
    m = sensors_cli.mount_from_args(a)
    assert (m.kind, m.location, m.motion, m.fov_deg, a.actor) == ("Camera", (1, 2, 3), False, 70.0, "X")


def test_no_engine(monkeypatch, capsys):
    class Down(FakeRemote):
        def ping(self):
            return False
    monkeypatch.setattr(_common, "make_remote", lambda h, p, t: Down())
    assert control_cli.main(["robots"]) == 2 and "no Remote Control API" in capsys.readouterr().err


# ── ramms-control ─────────────────────────────────────────────────────────

def test_control_describe_get_set(world, capsys):
    ue, path, comps = world("gen1")
    assert control_cli.main(["describe", "--group", "Curb"]) == 0
    out = capsys.readouterr().out
    assert "curb.style" in out and "0=Stop and step | 1=Rolling" in out and "lift.dw_main_plate_l" not in out
    assert control_cli.main(["get", "drive.mode", "curb.status"]) == 0
    out = capsys.readouterr().out
    assert "Differential" in out and "[Info] Ready" in out
    assert control_cli.main(["set", "curb.assist", "on"]) == 0
    assert comps["ControlSurface"].values["curb.assist"] == 1.0
    assert control_cli.main(["set", "drive.warp", "1"]) == 1
    assert "no control 'drive.warp'" in capsys.readouterr().err
    assert control_cli.main(["--json", "get", "lift.dw_main_plate_l", "--target"]) == 0
    assert json.loads(capsys.readouterr().out)["lift.dw_main_plate_l"] == {"value": 0.0, "target": None}


def test_control_robots_mode_drive(world, capsys):
    ue, path, comps = world("gen2")
    assert control_cli.main(["robots", "--json"]) == 0
    rows = json.loads(capsys.readouterr().out)
    assert rows[0]["robot"] == "Gen2Robot_C_0" and rows[0]["generation"] == "gen2" and rows[0]["mode"] == "differential"
    assert control_cli.main(["mode"]) == 0
    assert "* 1  differential" in capsys.readouterr().out
    assert control_cli.main(["mode", "Holonomic"]) == 0
    assert comps["DriveModeSelector"].active == "holonomic"
    assert control_cli.main(["mode", "--cycle"]) == 0
    assert comps["DriveModeSelector"].active == "low_level"
    assert control_cli.main(["drive", "--forward", "0.2", "--seconds", "0.15", "--rate", "20"]) == 0
    sent = comps["ControlSurface"].sets
    assert ("drive.forward", 0.2, "Remote") in sent and "drive.forward" in comps["ControlSurface"].released
    assert control_cli.main(["drive", "--strafe", "0.5", "--seconds", "0.1"]) == 1
    assert "no drive.strafe" in capsys.readouterr().err


def test_control_set_continuous_needs_hold(world, capsys):
    ue, path, comps = world("gen1", deadman=False)
    fake = comps["ControlSurface"]
    assert control_cli.main(["set", "drive.forward", "0.3"]) == 1
    assert "Continuous control" in capsys.readouterr().err and fake.sets == []
    assert control_cli.main(["set", "drive.forward", "0.3", "--hold", "0.1", "--rate", "50"]) == 0
    assert "released" in capsys.readouterr().out and fake.latched() == {}
    assert control_cli.main(["set", "lift.dw_main_plate_l", "5"]) == 0
    assert "holds that target until released" in capsys.readouterr().out


@pytest.mark.parametrize("cmd", [["drive", "--forward", "0.5", "--seconds", "5", "--rate", "50"],
                                 ["set", "drive.turn", "0.4", "--hold", "5", "--rate", "50"]])
def test_control_signal_releases_and_exits_cleanly(world, capsys, cmd):
    """SIGTERM (or Ctrl+C) mid-hold: the held controls are released and the CLI says so, no traceback."""
    ue, path, comps = world("gen1", deadman=False)
    fake = comps["ControlSurface"]
    n = {"sets": 0}

    def term(func, p):
        if func == "SetControl":
            n["sets"] += 1
            if n["sets"] == 5:
                raise _common.Interrupted(signal.SIGTERM)
    fake.on_call = term
    assert control_cli.main(cmd) == 130
    assert "interrupted (SIGTERM): held controls released" in capsys.readouterr().err
    assert fake.latched() == {}


def test_exit_signals_turn_sigterm_into_interrupted():
    previous = signal.getsignal(signal.SIGTERM)
    with pytest.raises(_common.Interrupted) as e:
        with _common.exit_signals():
            signal.raise_signal(signal.SIGTERM)
            time.sleep(0.5)
    assert e.value.signame == "SIGTERM" and signal.getsignal(signal.SIGTERM) is previous


def test_control_watch(world, capsys):
    world("gen1")
    assert control_cli.main(["watch", "curb.phase", "curb.status", "--rate", "50", "--duration", "0.05", "--json"]) == 0
    lines = [json.loads(ln) for ln in capsys.readouterr().out.splitlines()]
    assert lines and lines[0]["curb.phase"] == "Idle" and lines[0]["curb.status"]["text"] == "Ready"


# ── ramms-curb ────────────────────────────────────────────────────────────

def test_curb_status_both_generations(world, capsys):
    world("gen1")
    assert curb_cli.main(["status"]) == 0
    out = capsys.readouterr().out
    assert "RAMMP Gen1" in out and "status   Ready" in out and "style Stop and step" in out
    world("gen2")
    assert curb_cli.main(["status", "--json"]) == 0
    s = json.loads(capsys.readouterr().out)
    assert s["generation"] == "gen2" and s["phase"] == "Idle" and s["detection"].startswith("+12.0 cm step")


def test_curb_detect_assist_style_plan_seat(world, capsys):
    ue, path, comps = world("gen1")
    assert curb_cli.main(["detect"]) == 0 and "no look-ahead step detection" in capsys.readouterr().out
    assert curb_cli.main(["assist", "on"]) == 0 and comps["ControlSurface"].values["curb.assist"] == 1.0
    assert curb_cli.main(["style", "rolling"]) == 0 and "Rolling" in capsys.readouterr().out
    assert curb_cli.main(["plan"]) == 1 and "RAMMP Gen2" in capsys.readouterr().err
    assert curb_cli.main(["seat"]) == 1
    ue, path, comps = world("gen2")
    assert curb_cli.main(["detect"]) == 0
    out = capsys.readouterr().out
    assert "step found" in out and "height +12.0 cm, edge 80.0 cm ahead" in out
    assert curb_cli.main(["style"]) == 1
    assert curb_cli.main(["plan"]) == 0 and "plan: (none yet)" in capsys.readouterr().out
    comps["Seat"].handlers["SetTiltDeg"] = lambda p: {"ReturnValue": True}
    assert curb_cli.main(["seat", "--tilt", "-5", "--outline"]) == 0
    out = capsys.readouterr().out
    assert "tilt -> -5 deg" in out and "seat pivot (-10.0, 40.0) cm" in out and "reachable outline" in out


def test_curb_go_starts_and_waits_holding_forward(world, capsys):
    ue, path, comps = world("gen1", deadman=False)
    comps["CurbController"].handlers["StartCrossing"] = lambda p: {"OutReason": "", "ReturnValue": True}
    scripted_phases(comps["CurbController"], [("Idle", "Ready", "Info")] + [("Level", "Crossing", "Info")] * 5
                    + [("Done", "Crossed 15 cm", "Info")])
    assert curb_cli.main(["go", "--wait", "--wait-timeout", "5"]) == 0
    out = capsys.readouterr().out
    # The crossing is started directly (the controller selects the curb mode); go never selects it by hand.
    assert "drive mode differential -> curb" not in out and "crossing started" in out and "done after" in out
    assert "holding drive.forward +0.50 while the crossing is under way" in out and "CrossingSeconds" in out
    assert comps["DriveModeSelector"].active == "differential"
    assert ("drive.forward", 0.5, "Remote") in comps["ControlSurface"].sets
    assert comps["ControlSurface"].latched() == {} and "drive.forward" in comps["ControlSurface"].released


@pytest.mark.parametrize("on_interrupt", ["abort", "pause"])
def test_curb_go_wait_interrupted(world, capsys, on_interrupt):
    """Ctrl+C (or SIGTERM) mid-crossing: the stick is released first, then the crossing aborted (or left paused)."""
    ue, path, comps = world("gen1", deadman=False)
    comps["CurbController"].handlers["StartCrossing"] = lambda p: {"OutReason": "", "ReturnValue": True}
    n = {"polls": 0}

    def phase(p):
        n["polls"] += 1
        if n["polls"] == 8:
            raise _common.Interrupted(signal.SIGINT)
        return {"ReturnValue": "Idle" if n["polls"] == 1 else "DriveUp"}
    comps["CurbController"].handlers["GetPhase"] = phase
    assert curb_cli.main(["go", "--wait", "--on-interrupt", on_interrupt]) == 130
    err = capsys.readouterr().err
    assert comps["ControlSurface"].latched() == {} and ("drive.forward", 0.5, "Remote") in comps["ControlSurface"].sets
    if on_interrupt == "abort":
        assert ue.calls_to("Abort") == [{"Reason": "Stopped from ramms-curb (SIGINT)"}] and "crossing aborted" in err
    else:
        assert ue.calls_to("Abort") == [] and "the crossing is paused" in err


def test_curb_go_without_wait_says_the_stick_is_needed(world, capsys):
    ue, path, comps = world("gen1")
    comps["CurbController"].handlers["StartCrossing"] = lambda p: {"OutReason": "", "ReturnValue": True}
    assert curb_cli.main(["go"]) == 0
    assert "moves only while the stick" in capsys.readouterr().out and comps["ControlSurface"].sets == []


def test_curb_drive_interrupted_releases(world, capsys):
    ue, path, comps = world("gen1", deadman=False)
    fake = comps["ControlSurface"]
    n = {"sets": 0}

    def term(func, p):
        if func == "SetControl":
            n["sets"] += 1
            if n["sets"] == 6:
                raise _common.Interrupted(signal.SIGTERM)
    fake.on_call = term
    assert curb_cli.main(["drive", "--speed", "0.3", "--seconds", "5", "--rate", "50"]) == 130
    assert "interrupted (SIGTERM)" in capsys.readouterr().err and fake.latched() == {}


def test_curb_go_when_the_mode_switch_started_it(world, capsys):
    ue, path, comps = world("gen1")
    comps["CurbController"].handlers["StartCrossing"] = lambda p: {"OutReason": "Already crossing", "ReturnValue": False}
    phases = iter(["Idle", "Level"])              # idle before the start, crossing a tick later
    comps["CurbController"].handlers["GetPhase"] = lambda p: {"ReturnValue": next(phases, "Level")}
    assert curb_cli.main(["go"]) == 0
    assert "crossing under way (Chassis to crossing height)" in capsys.readouterr().out


def test_curb_go_refused(world, capsys):
    ue, path, comps = world("gen1")
    comps["CurbController"].handlers["StartCrossing"] = lambda p: {"OutReason": "No curb within 2 m", "ReturnValue": False}
    assert curb_cli.main(["go"]) == 1
    assert "not started: No curb within 2 m" in capsys.readouterr().err
    assert comps["DriveModeSelector"].active == "differential"


# ── ramms-sensors ─────────────────────────────────────────────────────────

def test_sensors_rig_offline(capsys):
    assert sensors_cli.main(["rig", "list"]) == 0
    assert "rammp_gen1" in capsys.readouterr().out
    assert sensors_cli.main(["rig", "show", "chair"]) == 0
    assert json.loads(capsys.readouterr().out)[0]["parent"] == ""


def test_sensors_add_list_read_watch(world, capsys):
    ue, path, comps = world("gen1")
    assert sensors_cli.main(["add", "front_tof", "--kind", "tof", "--parent", "chassis", "--at", "45,0,15", "--rot=-20,0,0"]) == 0
    sent = json.loads(ue.calls_to("AddSensorJson")[-1]["MountJson"])
    assert sent["location"] == [45.0, 0.0, 15.0] and sent["rotation"] == [-20.0, 0.0, 0.0] and sent["parent"] == "chassis"
    assert sensors_cli.main(["add", "cam", "--json", '{"kind": "camera", "channel": 3}', "--no-motion"]) == 0
    out = capsys.readouterr().out
    assert "base channel 3" in out and "ch  303  mask/cam" in out and "motion" not in out.split("cam (Camera)")[1]
    assert sensors_cli.main(["add", "x", "--kind", "imu", "--parent", "no_such_body"]) == 1
    assert 'has no component "no_such_body" to mount on' in capsys.readouterr().err
    assert sensors_cli.main(["list"]) == 0
    out = capsys.readouterr().out
    assert "front_tof" in out and "mask   ch  303" in out
    assert sensors_cli.main(["read", "front_tof", "--measure", "--grid"]) == 0
    out = capsys.readouterr().out
    assert "nearest 41.2 cm" in out and "  41.2" in out
    assert sensors_cli.main(["read", "cam"]) == 1 and "stream over RMSS" in capsys.readouterr().err
    assert sensors_cli.main(["watch", "--rate", "50", "--duration", "0.03", "--format", "csv"]) == 0
    lines = capsys.readouterr().out.splitlines()
    assert lines[0].startswith("t,name,kind,timestamp,ok") and "front_tof" in lines[1]
    assert sensors_cli.main(["rig", "apply", "rammp_gen1", "--replace"]) == 0
    assert "applied rammp_gen1 to" in capsys.readouterr().out
    assert sensors_cli.main(["remove", "front_sonar"]) == 0


def test_sensors_add_in_the_editor_world(world, capsys):
    ue, path, comps = world("gen1")
    bridge = ue.objects[SENSING_BRIDGE]
    bridge.play = False
    assert sensors_cli.main(["add", "tof", "--kind", "tof"]) == 1
    assert "needs a play world" in capsys.readouterr().err
    bridge.legacy = True                                     # an older sim: kept, built at BeginPlay
    assert sensors_cli.main(["add", "tof", "--kind", "tof"]) == 0
    assert "has not begun play" in capsys.readouterr().err


def test_sensors_streams_and_classes(world, capsys):
    ue, path, comps = world("gen1")
    sensors_cli.main(["add", "cam", "--kind", "camera"])
    capsys.readouterr()
    assert sensors_cli.main(["streams", "start", "--every", "2", "--no-motion"]) == 0
    out = capsys.readouterr().out
    assert "capture on (every 2 frames" in out and "camera/cam/color" in out
    assert sensors_cli.main(["streams", "stop"]) == 0 and "capture off" in capsys.readouterr().out
    assert sensors_cli.main(["classes"]) == 0 and "Car" in capsys.readouterr().out
    ue.objects[SENSING_BRIDGE].server_running = False
    assert sensors_cli.main(["streams", "status"]) == 0 and "RMSS server not running" in capsys.readouterr().out
    assert sensors_cli.main(["camera", "cam"]) == 1 and "RMSS server is not running" in capsys.readouterr().err


# ── ramms-worldgen additions ──────────────────────────────────────────────

def test_worldgen_inspect_terrain_traffic(tmp_path, capsys):
    f = tmp_path / "graded.json"
    f.write_text(json.dumps(GRADED))
    assert worldgen_cli.main(["inspect", str(f), "--terrain", "--traffic"]) == 0
    out = capsys.readouterr().out
    assert "steepest running grade 9.1 %" in out and "8.2..19.9 cm" in out and "Surface/Pothole 1" in out
    assert "2 segments, 80.0 m" in out and "Cross 2, Tee 1" in out and "signals 2" in out
    assert worldgen_cli.main(["inspect", str(f), "--terrain", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["terrain"]["max_running_grade"] == 0.091


def test_worldgen_terrain_sets_parameters(monkeypatch, capsys):
    ue = FakeRemote()
    bridge = FakeWorldGenBridge({"MaxStreetGrade": 0.0, "SidewalkCrossSlope": 0.0, "CurbHeightMin": 0.0, "CurbHeightMax": 0.0})
    calls = {"n": 0}

    def wg_bridge(func, p):
        if func == "FindGenerators":
            return {"ReturnValue": ["/Game/M.M:PersistentLevel.Gen"]}
        return bridge(func, p)
    ue.objects[WORLDGEN_BRIDGE] = wg_bridge
    monkeypatch.setattr(worldgen_cli, "WorldGen", lambda host, port, timeout: WorldGen(ue))
    assert worldgen_cli.main(["terrain", "--max-grade", "0.08", "--cross-slope", "0.02", "--curb-min", "10", "--curb-max", "20",
                              "--potholes", "2"]) == 0
    assert bridge.sets[-1] == {"MaxStreetGrade": 0.08, "SidewalkCrossSlope": 0.02, "CurbHeightMin": 10.0, "CurbHeightMax": 20.0,
                               "PotholeDensity": 2.0}
    cap = capsys.readouterr()
    assert "no PotholeDensity" in cap.err and "max_grade" in cap.out and "MaxStreetGrade" in cap.out
    with pytest.raises(SystemExit):
        worldgen_cli.main(["terrain", "--max-grade", "0.5"])          # out of the README's range: refused up front
    assert "0..0.15" in capsys.readouterr().err


def test_worldgen_traffic(monkeypatch, capsys):
    ue, state = traffic_world()
    monkeypatch.setattr(worldgen_cli, "WorldGen", lambda host, port, timeout: WorldGen(ue))
    assert worldgen_cli.main(["traffic", "spawn"]) == 0 and "cars: 24" in capsys.readouterr().out
    assert worldgen_cli.main(["traffic", "status"]) == 0
    out = capsys.readouterr().out
    assert "spawners 1  cars 24" in out and "X Green (12.5 s)" in out
    assert worldgen_cli.main(["traffic", "cars", "--limit", "2"]) == 0 and "2 cars" in capsys.readouterr().out
    assert worldgen_cli.main(["traffic", "signals", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["walk_along_y"] == "Walk"
    assert worldgen_cli.main(["traffic", "despawn"]) == 0 and state["count"] == 0


def test_sensors_camera_saves_a_frame_set(world, tmp_path, capsys):
    """`camera` against a local test RMSS server on a free port (not the engine's)."""
    import socket
    import threading

    import numpy as np

    from test_sensing import sample_arrays, send_camera
    from ramms_tools.streaming.sender import StreamSender

    world("gen1")
    server = socket.create_server(("127.0.0.1", 0))
    port = server.getsockname()[1]
    rgb, depth, motion, mask = sample_arrays()

    def serve():
        conn, _ = server.accept()
        with conn:
            conn.recv(65536)                       # the SUBSCRIBE
            s = StreamSender(queue_size=0)
            s._sock = conn
            send_camera(s, "front_cam", 4, rgb, depth, motion, mask, {3: "Car"})
            conn.recv(65536)                       # until the client hangs up
    t = threading.Thread(target=serve, daemon=True)
    t.start()
    out_dir = tmp_path / "shots"
    assert sensors_cli.main(["camera", "front_cam", "--channel", "4", "--stream-host", "127.0.0.1", "--stream-port", str(port),
                             "--save", str(out_dir), "--frames", "1", "--wait", "5"]) == 0
    server.close()
    out = capsys.readouterr().out
    assert "wrote 1 frame set(s)" in out and "missing" not in out
    assert np.array_equal(np.load(out_dir / "depth_000000.npy"), depth)
    assert np.array_equal(np.load(out_dir / "motion_000000.npy"), motion)
    assert json.loads((out_dir / "classes.json").read_text()) == {"3": "Car"}


def test_sensors_camera_is_the_robots_own(world, tmp_path, capsys):
    """Two robots each stream a "front_cam" (the same stream ids). `camera --robot` takes the one on that robot: its
    channel, its roles, and only frames naming its actor."""
    import socket
    import threading

    import numpy as np

    from fake_remote import FakeSurface, load
    from test_sensing import sample_arrays, send_camera
    from ramms_tools.streaming.sender import StreamSender

    ue, path, comps = world("gen1")
    other = ue.add_actor("Chair_C_0", {"ControlSurface": ("RammsRobotControlSurfaceComponent",
                                                          FakeSurface(dict(load("gen1", "surface.json"), robotName="Chair")))})
    bridge = ue.objects[SENSING_BRIDGE]
    bridge.AddSensorJson({"ActorPath": other, "MountJson": json.dumps({"name": "front_cam", "kind": "camera", "channel": 6})})
    bridge.AddSensorJson({"ActorPath": path, "MountJson": json.dumps({"name": "front_cam", "kind": "camera", "channel": 2,
                                                                      "motion": False, "segmentation": False})})
    server = socket.create_server(("127.0.0.1", 0))
    port = server.getsockname()[1]
    rgb, depth, motion, mask = sample_arrays()
    subscribed = {}

    def serve():
        conn, _ = server.accept()
        with conn:
            subscribed["msg"] = conn.recv(65536)
            s = StreamSender(queue_size=0)
            s._sock = conn
            # The other robot's camera on our channels too (a misconfigured source): dropped by actor.
            send_camera(s, "front_cam", 2, rgb[::-1].copy(), depth * 3, motion, mask, {3: "Car"}, actor=other, frame=1,
                        roles=("color", "depth"))
            send_camera(s, "front_cam", 2, rgb, depth, motion, mask, {3: "Car"}, actor=path, frame=2,
                        roles=("color", "depth"))
            conn.recv(65536)
    t = threading.Thread(target=serve, daemon=True)
    t.start()
    assert sensors_cli.main(["camera", "front_cam", "--robot", "PoweredFront", "--stream-host", "127.0.0.1",
                             "--stream-port", str(port), "--save", str(tmp_path), "--wait", "5"]) == 0
    server.close()
    out = capsys.readouterr().out
    assert "channels {'color': 2, 'depth': 102}" in out and "missing" not in out
    assert np.array_equal(np.load(tmp_path / "depth_000000.npy"), depth)
    assert "1 frames of another actor" in out or "2 frames of another actor" in out
    assert b'"channels": [2, 102]' in subscribed["msg"]
    capsys.readouterr()
    assert sensors_cli.main(["camera", "rear_cam", "--robot", "PoweredFront"]) == 1
    assert sensors_cli.main(["camera", "front_cam", "--actor", "/Game/Maps/Map_Test.Map_Test:PersistentLevel.Nobody_C_0"]) == 1
    assert "no camera 'front_cam' on Nobody_C_0 (there is one on" in capsys.readouterr().err
