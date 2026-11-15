"""Offline tests for ramms_tools.sensing and RMSS camera-frame decoding (run: python -m pytest tests)."""
import json
import logging
import socket
import threading
import time

import numpy as np
import pytest

from fake_remote import FakeRemote, robot_world
from ramms_tools.robot import Robot
from ramms_tools.sensing import (BRIDGE, CameraStreams, FrameSet, ImuStream, RIG_PRESETS, Sensing, SensingError,
                                 SensorInfo, SensorMount, StreamStatus, apply_rig, imu_stream_id, rig_mounts, rig_names,
                                 save_frame)
from ramms_tools.streaming.client import StreamClient
from ramms_tools.streaming.frames import (IMU_FIELDS, camera_channels, camera_stream_ids, decode_message, decode_rgbd,
                                          mask_classes, message_role)
from ramms_tools.streaming.protocol import Compression, MessageType, StreamMessage
from ramms_tools.streaming.sender import StreamSender


# ── a sensing bridge that follows the spec ───────────────────────────────

class FakeSensingBridge:
    """
    URammsSensingBridge as the C++ answers (RammsSensingBridge.cpp) and the fix-round spec adds:

    - play=False is the editor world. Sims from the fix round (legacy=False) refuse sensing changes there
      ({"ok": false, "error": "... needs a play world (PIE or a game)"}); older sims (legacy=True) keep the mount
      and answer "built": false with a note (built at BeginPlay).
    - A parent component the actor does not have is an error ({"ok": false, ...}), never "built": false.
    - Hand-placed sensors are listed with "rig": false; a hand-placed camera no stream source forwards reports
      channel -1 and no streams. Parent / socket are "" for none (NameOrEmpty).
    - The stream status reports port 0 and "running": false without a server.
    """

    COMPONENTS = ("", "chassis", "seat")           # the components the fake actors have to mount on
    MUTATIONS = ("AddSensorJson", "RemoveSensor", "StartCameraStreamsJson", "StopCameraStreams", "StartImuStreamJson")

    def __init__(self, play: bool = True, legacy: bool = False):
        self.sensors: dict[str, dict[str, dict]] = {}      # actor -> name -> entry
        self.capturing = False
        self.server_running = True
        self.options = {}
        self.next_channel = 0
        self.play = play
        self.legacy = legacy

    def __call__(self, func, p):
        f = getattr(self, func, None)
        if f is None:
            raise AssertionError(func)
        if func in self.MUTATIONS and not self.play and not self.legacy:
            out = False if func in ("RemoveSensor", "StopCameraStreams") else                 {"ok": False, "error": f"{func} needs a play world (PIE or a game)"}
        else:
            out = f(p)
        return {"ReturnValue": out if isinstance(out, bool) else json.dumps(out)}

    def _streams(self, actor, name, ch, s):
        out = []
        for role, on, off, msg, fmt in (("color", s["color"], 0, "FrameRGB", "bgra8"), ("depth", s["depth"], 100, "FrameDepth", "float32"),
                                        ("motion", s["motion"], 200, "FrameMotion", "rg32f"), ("mask", s["segmentation"], 300, "FrameData", "mono8")):
            if on:
                sid = {"color": f"camera/{name}/color", "depth": f"camera/{name}/depth"}.get(role, f"{role}/{name}")
                out.append({"role": role, "stream_id": sid, "channel": ch + off, "message": msg, "fmt": fmt})
        return out

    def ListSensorsJson(self, p):
        actors = [self._key(p["ActorPath"])] if self._key(p["ActorPath"]) else list(self.sensors)
        return {"ok": True, "sensors": [{k: v for k, v in e.items() if not k.startswith("_")}
                                        for a in actors for e in self.sensors.get(a, {}).values()]}

    def AddSensorJson(self, p):
        m = json.loads(p["MountJson"])
        kind = {"tof": "ToF", "sonar": "Sonar", "imu": "IMU", "camera": "Camera"}[m["kind"].lower()]
        actor, name = self._key(p["ActorPath"]), m["name"]
        parent = m.get("parent", "") or ""
        if parent not in self.COMPONENTS:
            return {"ok": False, "error": f"{actor.rsplit('.', 1)[-1]} has no component \"{parent}\" to mount on"}
        settings = {}
        if kind == "ToF":
            settings = {k: m.get(k, d) for k, d in (("rows", 8), ("columns", 8), ("h_fov_deg", 45), ("v_fov_deg", 45), ("max_range_cm", 400))}
        elif kind == "Sonar":
            settings = {"max_range_cm": m.get("sonar_max_range_cm", 500), "cone_deg": m.get("sonar_cone_deg", 15)}
        elif kind == "Camera":
            ch = m.get("channel", -1)
            if ch < 0:
                ch, self.next_channel = self.next_channel, self.next_channel + 1
            settings = {k: m.get(k, d) for k, d in (("width", 640), ("height", 480), ("fov_deg", 87), ("color", True), ("depth", True),
                                                     ("motion", True), ("segmentation", True))}
            settings["channel"] = ch
        loc = m.get("location", [0, 0, 0])
        rot = m.get("rotation", [0, 0, 0])
        entry = {"actor": actor, "actor_name": actor.rsplit(".", 1)[-1], "name": name, "kind": kind,
                 "_rig": f"{actor}.SensorRig_0",
                 "component_path": f"{actor}.{name}", "parent": parent, "socket": "", "rig": True,
                 "location": {"x": 100 + loc[0], "y": loc[1], "z": loc[2]}, "rotation": {"pitch": rot[0], "yaw": rot[1], "roll": rot[2]},
                 "settings": settings, "streams": self._streams(actor, name, settings["channel"], settings) if kind == "Camera" else []}
        self.sensors.setdefault(actor, {})[name] = entry
        out = {"ok": True, "name": name, "kind": kind, "component_path": entry["component_path"], "rig_path": entry["_rig"],
               "built": self.play}
        if not self.play:
            out["component_path"] = ""
            out["note"] = "the actor has not begun play; the mount is built at BeginPlay"
        elif kind == "Camera":
            out.update(channel=settings["channel"], streams=entry["streams"])
        return out

    def _key(self, actor):
        """An actor named by path or by name (the bridge takes either; it answers with the path)."""
        if not actor or actor in self.sensors:
            return actor
        return next((k for k in self.sensors if k.rsplit(".", 1)[-1] == actor), actor)

    def place_camera(self, actor, name, channel=-1):
        """A capture camera placed by hand: no rig; channel -1 and no streams when no stream source forwards it."""
        self.sensors.setdefault(actor, {})[name] = {
            "actor": actor, "actor_name": actor.rsplit(".", 1)[-1], "name": name, "kind": "Camera",
            "component_path": f"{actor}.{name}", "parent": "", "socket": "", "rig": False,
            "location": {"x": 0, "y": 0, "z": 0}, "rotation": {"pitch": 0, "yaw": 0, "roll": 0},
            "settings": {"width": 640, "height": 480, "fov_deg": 90, "color": channel >= 0, "depth": channel >= 0,
                         "motion": False, "segmentation": False, "channel": channel},
            "streams": [] if channel < 0 else self._streams(actor, name, channel, {"color": True, "depth": True,
                                                                                "motion": False, "segmentation": False})}

    def RemoveSensor(self, p):
        e = self.sensors.get(self._key(p["ActorPath"]), {}).get(p["Name"])
        if e is None or not e["rig"]:
            return False                        # only rig sensors can be removed
        return self.sensors[self._key(p["ActorPath"])].pop(p["Name"]) is not None

    def _reading(self, e):
        common = {"ok": True, "name": e["name"], "kind": e["kind"], "actor": e["actor"], "timestamp": 12.3, "has_sample": True}
        if e["kind"] == "IMU":
            common["source"] = "mujoco"
        if e["kind"] == "ToF":
            d = [-1.0] * 64
            d[27] = 41.2
            return {**common, "rows": 8, "columns": 8, "distances_cm": d, "min_distance_cm": 41.2, "any_hit": True}
        if e["kind"] == "Sonar":
            return {**common, "distance_cm": 88.0, "hit": True, "hit_location": {"x": 1, "y": 2, "z": 3},
                    "hit_normal": {"x": -1, "y": 0, "z": 0}, "hit_actor": "Curb_3"}
        if e["kind"] == "IMU":
            # The component's own units, unconverted: cm/s^2 and deg/s.
            return {**common, "linear_acceleration": {"x": 10.0, "y": 0.0, "z": 981.0}, "angular_velocity": {"x": 0, "y": 0, "z": 1.5},
                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "euler": {"roll": 0, "pitch": 1.0, "yaw": 90.0},
                    "units": {"linear_acceleration": "cm/s^2", "angular_velocity": "deg/s"}, "includes_gravity": True}
        return {"ok": False, "name": e["name"], "kind": e["kind"], "error": "cameras stream over RMSS: see its streams"}

    def ReadSensorJson(self, p):
        e = self.sensors.get(self._key(p["ActorPath"]), {}).get(p["Name"])
        return self._reading(e) if e else {"ok": False, "error": f"no sensor {p['Name']}"}

    def ReadSensorsJson(self, p):
        return {"ok": True, "actor": self._key(p["ActorPath"]),
                "readings": [self._reading(e) for e in self.sensors.get(self._key(p["ActorPath"]), {}).values() if e["kind"] != "Camera"]}

    def GetStreamStatusJson(self, p):
        cams = [e for a in self.sensors.values() for e in a.values() if e["kind"] == "Camera"]
        server = {"running": True, "port": 30030, "connections": 1} if self.server_running else             {"running": False, "port": 0, "connections": 0}
        return {"ok": True, "server": server,
                "capture": {"capturing": self.capturing, "every_n_frames": self.options.get("every_n_frames", 1), "color": True,
                            "depth": True, "motion": True, "segmentation": True, "save_to_disk": False, "cameras": len(cams)},
                "streams": [dict(s, actor=e["actor"], camera=e["name"]) for e in cams for s in e["streams"]]
                + list(getattr(self, "imu_streams", {}).values())}

    def StartCameraStreamsJson(self, p):
        self.options = json.loads(p["OptionsJson"])
        self.capturing = True
        self.server_running = True
        return self.GetStreamStatusJson(p)

    def StopCameraStreams(self, p):
        self.capturing = False
        return True

    # Body IMU (ReadBodyImuJson): the first read of a point starts tracking it and has no sample yet.
    def ReadBodyImuJson(self, p):
        opts = json.loads(p["OptionsJson"])
        key = (p["ActorPath"], p["Body"], tuple(opts.get("offset", [0, 0, 0])))
        self.body_reads = getattr(self, "body_reads", {})
        n = self.body_reads[key] = self.body_reads.get(key, 0) + 1
        body = p["Body"] or "chassis"
        if body == "nope":
            return {"ok": False, "error": "no body 'nope' on the actor"}
        out = {"ok": True, "actor": p["ActorPath"], "body": body, "bone": "", "component_path": f"{p['ActorPath']}.{body}",
               "source": "kinematic" if opts.get("source") == "kinematic" else "mujoco", "timestamp": 3.0 + n,
               "sim_time": 2.9 + n, "has_sample": n > 1, "has_acceleration": n > 1, "includes_gravity": True,
               "units": {"linear_acceleration": "cm/s^2", "angular_velocity": "deg/s", "angular_acceleration": "deg/s^2"},
               "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "euler": {"roll": 0, "pitch": -1.4, "yaw": 0},
               "linear_velocity": {"x": 50.0, "y": 0, "z": 0}, "location": {"x": 1, "y": 2, "z": 3}}
        if n > 1:
            out.update(linear_acceleration={"x": -23.0, "y": 0.0, "z": 981.0}, angular_velocity={"x": 0, "y": 0, "z": 0.5},
                       angular_acceleration={"x": 0, "y": 0, "z": 2.0})
        else:
            out.update(linear_acceleration={"x": 0, "y": 0, "z": 0}, angular_velocity={"x": 0, "y": 0, "z": 0})
        return out

    # Buffered physics-step samples (ReadImuSamplesJson) and the RMSS IMU stream, at a 500 Hz physics rate.
    IMU_RATE = 500.0
    IMU_BUFFER = 1024

    def _imu_seq(self, key):
        self.imu_t0 = getattr(self, "imu_t0", {})
        t0 = self.imu_t0.setdefault(key, time.monotonic())
        return int((time.monotonic() - t0) * self.IMU_RATE)

    @staticmethod
    def imu_sample(seq):
        return {"seq": seq, "sim_time": seq / 500.0, "linear_acceleration": {"x": 0.5, "y": 0.0, "z": 981.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": seq * 0.01}, "angular_acceleration": {"x": 0, "y": 0, "z": 5.0},
                "linear_velocity": {"x": 10.0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}

    def ReadImuSamplesJson(self, p):
        opts = json.loads(p["OptionsJson"] or "{}")
        target = p["Target"]
        if target == "nope":
            return {"ok": False, "error": "no IMU sensor or body \"nope\""}
        now = self._imu_seq((p["ActorPath"], target))
        since = int(opts.get("since", -1))
        out = {"ok": True, "actor": p["ActorPath"], "target": target, "body": target or "chassis", "source": "mujoco",
               "units": {"linear_acceleration": "cm/s^2", "angular_velocity": "deg/s", "angular_acceleration": "deg/s^2",
                         "linear_velocity": "cm/s", "time": "s"}, "rate_hz": self.IMU_RATE}
        if since < 0:
            return {**out, "samples": [], "next": now, "dropped": 0}
        oldest = max(0, now - self.IMU_BUFFER)
        dropped = max(0, oldest - since)
        first = max(since, oldest)
        last = min(now, first + int(opts.get("max", 1024)))
        return {**out, "samples": [self.imu_sample(q) for q in range(first, last)], "next": last, "dropped": dropped}

    def StartImuStreamJson(self, p):
        """As URammsSensingBridge::StartImuStreamJson answers: the stream itself, and the server under "server"."""
        opts = json.loads(p["OptionsJson"] or "{}")
        self.imu_streams = getattr(self, "imu_streams", {})
        actor, target = p["ActorPath"], p["Target"].strip()
        ch = opts.get("channel", -1)
        if ch < 0:                                  # the lowest free channel from 1000 (a restart keeps its own)
            old = self.imu_streams.get((actor, target))
            taken = {e["channel"] for k, e in self.imu_streams.items() if k != (actor, target)}
            ch = old["channel"] if old else min(c for c in range(1000, 2000) if c not in taken)
        body = target or "chassis"
        entry = {"actor": actor, "camera": "", "target": target, "role": "imu",
                 "stream_id": f"imu/{actor.rsplit('.', 1)[-1]}/{target or body}", "channel": ch, "message": "FrameData",
                 "fmt": "imu_f64x18"}
        self.imu_streams[(actor, target)] = entry       # listed in GetStreamStatusJson's streams (StreamEntryJson)
        self.server_running = True
        return {"ok": True, "actor": actor, "target": target, "body": body, "source": "mujoco",
                "stream_id": entry["stream_id"], "channel": ch, "message": "FrameData", "fmt": "imu_f64x18",
                "server": {"running": True, "port": 30030, "connections": 1}}

    def StopImuStream(self, p):
        return getattr(self, "imu_streams", {}).pop((p["ActorPath"], p["Target"].strip()), None) is not None

    def ListImuBodiesJson(self, p):
        return {"ok": True, "actor": p["ActorPath"], "bodies": [
            {"name": "chassis", "engine": "mujoco", "component": "chassis", "component_path": f"{p['ActorPath']}.chassis",
             "bone": "", "default": True},
            {"name": "seat", "engine": "mujoco", "component": "seat", "component_path": f"{p['ActorPath']}.seat", "bone": "",
             "default": False}]}

    def GetSegmentationClassesJson(self, p):
        return {"ok": True, "classes": [{"value": 3, "name": "Car", "coarse": "Vehicle", "coarse_id": 2},
                                        {"value": 7, "name": "Curb", "coarse": "Ground", "coarse_id": 1}],
                "coarse": [{"id": 1, "name": "Ground"}, {"id": 2, "name": "Vehicle"}], "reserved": [0], "num_segment_ids": 8}


def sensing_world(gen="gen1"):
    ue, path, comps = robot_world(gen)
    bridge = FakeSensingBridge()
    ue.objects[BRIDGE] = bridge
    return ue, path, bridge


# ── mounts and rigs ──────────────────────────────────────────────────────

def test_mount_json_follows_the_spec():
    m = SensorMount("front_tof", "tof", parent="chassis", location=(45, 0, 15), rotation=(-20, 0, 0))
    d = m.to_dict()
    assert d == {"name": "front_tof", "kind": "ToF", "parent": "chassis", "socket": "", "location": [45.0, 0.0, 15.0],
                 "rotation": [-20.0, 0.0, 0.0], "rows": 8, "columns": 8, "h_fov_deg": 45.0, "v_fov_deg": 45.0, "max_range_cm": 400.0}
    cam = SensorMount.from_dict({"name": "cam", "kind": "CAMERA", "location": {"x": 1, "y": 2, "z": 3},
                                 "rotation": {"pitch": -10, "yaw": 5, "roll": 0}, "motion": False, "channel": 4})
    assert cam.kind == "Camera" and cam.location == (1, 2, 3) and cam.rotation == (-10, 5, 0)
    assert set(cam.to_dict()) == {"name", "kind", "parent", "socket", "location", "rotation", "width", "height", "fov_deg",
                                  "color", "depth", "motion", "segmentation", "channel"}
    assert cam.stream_ids() == {"color": "camera/cam/color", "depth": "camera/cam/depth", "mask": "mask/cam"}
    assert set(SensorMount("i", "imu").to_dict()) == {"name", "kind", "parent", "socket", "location", "rotation"}
    spec_keys = {"name", "kind", "parent", "socket", "location", "rotation", "rows", "columns", "h_fov_deg", "v_fov_deg",
                 "max_range_cm", "sonar_max_range_cm", "sonar_cone_deg", "width", "height", "fov_deg", "color", "depth",
                 "motion", "segmentation", "channel"}
    assert set(SensorMount("i", "imu").to_dict(all_keys=True)) == spec_keys
    with pytest.raises(SensingError, match="unknown mount keys: fov"):
        SensorMount.from_dict({"name": "x", "kind": "Camera", "fov": 90})
    with pytest.raises(SensingError, match="kind must be one of"):
        SensorMount("x", "lidar")
    with pytest.raises(SensingError, match="needs a name"):
        SensorMount("", "ToF")


def test_sonar_mount_takes_the_keys_list_sensors_reports():
    """ListSensorsJson reports a sonar's range / cone as max_range_cm / cone_deg, and AddSensorJson takes them too."""
    m = SensorMount.from_dict({"name": "s", "kind": "sonar", "max_range_cm": 300, "cone_deg": 10})
    assert (m.sonar_max_range_cm, m.sonar_cone_deg) == (300.0, 10.0)
    assert m.to_dict()["sonar_max_range_cm"] == 300.0 and m.to_dict()["sonar_cone_deg"] == 10.0
    assert SensorMount("s", "Sonar", max_range_cm=250).sonar_max_range_cm == 250.0      # the constructor too
    assert SensorMount("s", "Sonar").sonar_max_range_cm == 500.0 and SensorMount("t", "ToF").max_range_cm == 400.0
    assert SensorMount("t", "ToF", max_range_cm=120).max_range_cm == 120.0
    with pytest.raises(SensingError, match="disagree"):
        SensorMount.from_dict({"name": "s", "kind": "sonar", "max_range_cm": 300, "sonar_max_range_cm": 200})
    with pytest.raises(SensingError, match="unknown mount keys: cone_deg"):
        SensorMount.from_dict({"name": "t", "kind": "tof", "cone_deg": 10})        # only a sonar has a cone


def test_a_mount_without_a_name_is_a_sensing_error(tmp_path):
    with pytest.raises(SensingError, match="needs a name"):
        SensorMount.from_dict({"kind": "tof"})
    f = tmp_path / "rig.json"
    f.write_text(json.dumps([{"kind": "tof", "location": [1, 2, 3]}]))
    with pytest.raises(SensingError, match="needs a name"):
        rig_mounts(str(f))


def test_imu_stream_id_for_the_default_body():
    a = "/Game/M.M:PersistentLevel.R_C_0"
    assert imu_stream_id(a, "imu") == "imu/R_C_0/imu"
    assert imu_stream_id(a, "", body="chassis") == "imu/R_C_0/chassis"        # target "": the body's name
    with pytest.raises(ValueError, match="body="):
        imu_stream_id(a, "")


def test_rig_presets_are_valid_mounts(tmp_path):
    assert {"rammp_gen1", "chair"} <= set(rig_names())
    for name in RIG_PRESETS:
        ms = rig_mounts(name)
        assert {m.kind for m in ms} == {"ToF", "Sonar", "IMU", "Camera"}
        assert len({m.name for m in ms}) == len(ms)
    assert {m.parent for m in rig_mounts("rammp_gen1")} == {"chassis"}
    assert {m.parent for m in rig_mounts("chair")} == {""}
    f = tmp_path / "rig.json"
    f.write_text(json.dumps({"mounts": [{"name": "s", "kind": "sonar"}]}))
    assert [m.kind for m in rig_mounts(str(f))] == ["Sonar"]
    with pytest.raises(SensingError, match="no rig preset"):
        rig_mounts("nope")


# ── the bridge ───────────────────────────────────────────────────────────

def test_add_list_read_remove():
    ue, path, bridge = sensing_world()
    s = Robot.find(ue=ue).sensors
    res = s.add(SensorMount("front_tof", "ToF", parent="chassis", location=(45, 0, 15)))
    assert res["ok"] and res["name"] == "front_tof" and res["component_path"] == path + ".front_tof" and res["built"]
    assert res["kind"] == "ToF" and res["rig_path"].endswith("SensorRig_0")
    sent = json.loads(ue.calls_to("AddSensorJson")[-1]["MountJson"])
    assert sent["kind"] == "ToF" and sent["location"] == [45.0, 0.0, 15.0] and ue.calls_to("AddSensorJson")[-1]["ActorPath"] == path
    s.add({"name": "front_sonar", "kind": "sonar"})
    s.add({"name": "imu", "kind": "IMU"})
    info = {i.name: i for i in s.list()}
    assert info["front_tof"].location == (145, 0, 15) and info["front_tof"].parent == "chassis" and info["front_tof"].socket == ""
    assert info["front_tof"].summary().startswith("8x8") and info["front_sonar"].settings["cone_deg"] == 15
    tof = s.read("front_tof", measure=True)
    assert ue.calls_to("ReadSensorJson")[-1]["bMeasureNow"] is True
    assert tof.any_hit and tof.min_distance_cm == 41.2 and tof.value == 41.2
    g = tof.grid()
    assert g.shape == (8, 8) and g[3, 3] == pytest.approx(41.2) and np.isnan(g[0, 0])
    readings = {r.name: r for r in s.read_all()}
    assert readings["front_sonar"].hit_actor == "Curb_3" and readings["front_sonar"].hit_normal == (-1, 0, 0)
    imu = readings["imu"]
    assert imu.linear_acceleration == (10.0, 0.0, 981.0) and imu.units["linear_acceleration"] == "cm/s^2"   # raw, unconverted
    assert imu.linear_acceleration_ms2() == pytest.approx((0.1, 0.0, 9.81)) and imu.angular_velocity_degs() == (0, 0, 1.5)
    assert imu.euler == (0, 1.0, 90.0) and imu.orientation == (0, 0, 0, 1) and imu.includes_gravity is True
    assert imu.has_sample and imu.actor == path and "9.81) m/s^2" in imu.summary() and "deg/s" in imu.summary()
    assert imu.flat()["az"] == 981.0 and imu.flat()["accel_units"] == "cm/s^2"
    assert readings["front_sonar"].flat()["distance_cm"] == 88.0
    from ramms_tools.sensing import SensorReading
    assert SensorReading.from_json({"name": "i", "kind": "IMU", "has_sample": False,
                                    "linear_acceleration": {"x": 0, "y": 0, "z": 0}}).summary() == "no sample yet"
    with pytest.raises(SensingError, match="no sensor"):
        s.read("ghost")
    assert s.remove("front_sonar") and not s.remove("front_sonar")
    bridge.sensors[path]["imu"]["rig"] = False               # placed by hand: the bridge will not remove it
    assert not s.remove("imu") and "imu" in bridge.sensors[path]


def test_camera_read_is_refused_and_streams_listed():
    ue, path, bridge = sensing_world()
    s = Robot.find(ue=ue).sensors
    s.add(SensorMount("front_cam", "Camera", motion=False))
    with pytest.raises(SensingError, match="stream over RMSS"):
        s.read("front_cam")
    cam = s.list()[0]
    assert [st.role for st in cam.streams] == ["color", "depth", "mask"]
    assert cam.streams[2].stream_id == "mask/front_cam" and cam.streams[2].channel == 300 and cam.streams[0].message == "FrameRGB"
    assert cam.channel == 0                                  # the channel in use, auto-assigned
    res = s.add(SensorMount("rear_cam", "Camera", channel=-1))
    assert res["channel"] == 1 and [st["channel"] for st in res["streams"]] == [1, 101, 201, 301]
    cs = CameraStreams.from_sensor(cam)
    assert cs.base_channel == 0 and cs.roles == ("color", "depth", "mask")
    with pytest.raises(ValueError):
        CameraStreams.from_sensor(SensorInfo(name="t", kind="ToF"))


def test_apply_rig_and_replace():
    ue, path, bridge = sensing_world()
    robot = Robot.find(ue=ue)
    robot.sensors.add({"name": "old_sonar", "kind": "Sonar"})
    res = apply_rig(robot, "rammp_gen1")
    assert [r["name"] for r in res] == ["imu", "front_tof", "front_sonar", "front_camera"]
    assert "old_sonar" in bridge.sensors[path]
    robot.sensors.apply_rig("rammp_gen1", replace=True)
    assert set(bridge.sensors[path]) == {"front_tof", "front_sonar", "imu", "front_camera"}


def test_stream_control_status_and_classes():
    ue, path, bridge = sensing_world()
    sensing = Sensing(ue)
    sensing.add(path, {"name": "front_cam", "kind": "Camera"})
    st = sensing.start_streams(every_n_frames=2, motion=True, save_to_disk=None)
    assert bridge.options == {"every_n_frames": 2, "motion": True}            # None is not sent
    assert st.running and st.capturing and st.port == 30030 and st.cameras() == ["front_cam"]
    assert {s.role: s.channel for s in st.camera_streams("front_cam")} == {"color": 0, "depth": 100, "motion": 200, "mask": 300}
    assert sensing.stop_streams() and not sensing.status().capturing
    with pytest.raises(SensingError, match="unknown stream options: fps"):
        sensing.start_streams(fps=30)
    table = sensing.segmentation_classes()
    assert table.names() == {3: "Car", 7: "Curb"} and table.classes[0].coarse == "Vehicle" and len(table.coarse) == 2
    assert table.num_segment_ids == 8 and table.reserved == [0]


def test_contract_details():
    from ramms_tools.sensing import SensorReading
    from ramms_tools.sensing.bridge import check_sensor_name
    placed = SensorInfo.from_json({"name": "cam", "kind": "Camera", "settings": {"channel": -1}, "streams": []})
    assert placed.channel is None                           # a hand-placed camera no source forwards
    assert SensorInfo.from_json({"name": "cam", "kind": "Camera", "streams": [
        {"role": "depth", "channel": 104, "stream_id": "camera/cam/depth"}]}).channel == 4
    assert SensorInfo.from_json({"name": "s", "kind": "Sonar", "settings": {"sonar_max_range_cm": 300,
                                                                          "sonar_cone_deg": 10}}).summary() == \
        "300 cm, cone half-angle 10 deg"
    for st in ({"sim_time": None}, {}, {"sim_time": -1.0}):
        r = SensorReading.from_json({"name": "i", "kind": "IMU", "linear_acceleration": {"x": 0, "y": 0, "z": 981}, **st})
        assert r.sim_time is None
    assert SensorReading.from_json({"name": "i", "kind": "IMU", "sim_time": 2.5,
                                    "linear_acceleration": {"x": 0, "y": 0, "z": 981}}).sim_time == 2.5
    sonar = SensorReading.from_json({"name": "s", "kind": "Sonar", "distance_cm": 50.0, "hit": True, "has_sample": False,
                                     "max_range_cm": 500})
    assert sonar.has_sample is False and sonar.summary() == "no sample yet" and sonar.max_range_cm == 500.0
    assert check_sensor_name("front_cam.v2") == "front_cam.v2"
    for bad in ("", "a b", "x" * 65, "cam_stream", "tof_depth", "a/b"):
        with pytest.raises(SensingError):
            check_sensor_name(bad)
    with pytest.raises(SensingError, match="reserved|suffix"):
        SensorMount("cam_seg", "Camera")


def test_editor_world_add_is_refused_or_kept_for_begin_play():
    ue, path, bridge = sensing_world()
    s = Sensing(ue)
    bridge.play = False
    with pytest.raises(SensingError, match="needs a play world"):
        s.add(path, {"name": "tof", "kind": "ToF"})
    bridge.legacy = True                                   # an older sim keeps it and builds it at BeginPlay
    res = s.add(path, {"name": "tof", "kind": "ToF"})
    assert res["built"] is False and "BeginPlay" in res["note"]
    bridge.play, bridge.legacy = True, False
    with pytest.raises(SensingError, match="has no component \"wing\" to mount on"):
        s.add(path, {"name": "tof", "kind": "ToF", "parent": "wing"})


def test_apply_rig_replace_by_actor_name():
    ue, path, bridge = sensing_world()
    name = path.rsplit(".", 1)[-1]
    bridge.sensors[path] = {}                               # the bridge resolves a name to the actor; lists the path
    s = Sensing(ue)
    s.add(name, {"name": "old_sonar", "kind": "Sonar"})
    assert [x.actor for x in s.list(name)] == [path]
    s.apply_rig(name, rig_mounts("chair"), replace=True)
    assert "old_sonar" not in bridge.sensors[path] and "front_camera" in bridge.sensors[path]


def test_bridge_error_is_raised():
    ue = FakeRemote()
    ue.objects[BRIDGE] = lambda func, p: {"ReturnValue": json.dumps({"ok": False, "error": "no such actor"})}
    with pytest.raises(SensingError, match="ListSensorsJson: no such actor"):
        Sensing(ue).list("/Game/X")


# ── RMSS frames: produced by the sender, received by the client, decoded ──

W, H = 8, 6


def pair():
    """A StreamClient receiving what a synchronous StreamSender sends, over a socket pair."""
    ours, theirs = socket.socketpair()
    client = StreamClient()
    client._sock = ours
    client._connected.set()
    sender = StreamSender(queue_size=0)
    sender._sock = theirs
    drain = threading.Thread(target=lambda: _drain(theirs), daemon=True)   # swallow the client's SUBSCRIBE
    drain.start()
    return client, sender, (ours, theirs)


def _drain(sock):
    try:
        while sock.recv(65536):
            pass
    except OSError:
        pass


def color_message(name, channel, rgb, extra=None):
    """A FrameRGB as UE sends it: BGRA8 with the camera's metadata."""
    bgra = np.concatenate([rgb[:, :, ::-1], np.full(rgb.shape[:2] + (1,), 255, np.uint8)], axis=2)
    m = StreamMessage()
    m.header.message_type = MessageType.FRAME_RGB
    m.header.channel_id = channel
    m.metadata = json.dumps({"w": W, "h": H, "fmt": "bgra8", "stream_id": f"camera/{name}/color", "group": name,
                             "role": "color", "name": f"{name} color", **(extra or {})}).encode()
    m.payload = bgra.tobytes()
    return m


def send_camera(sender, name, base, rgb, depth, motion, mask, classes, actor=None, frame=None, roles=("color", "depth", "motion", "mask")):
    """One capture of a camera as the sim streams it; actor / frame add the metadata the sim adds (its actor path,
    the capture's frame number)."""
    extra = {k: v for k, v in (("actor", actor), ("frame", frame)) if v is not None}
    if "color" in roles:
        sender._send_msg(color_message(name, base, rgb, extra))
    if "depth" in roles:
        sender.send_numpy_depth(base + 100, depth, metadata={"depth_w": W, "depth_h": H, **extra},
                                stream_id=f"camera/{name}/depth", group=name, name=f"{name} depth")
    if "motion" in roles:
        sender.send_numpy_motion(base + 200, motion, metadata={"motion_w": W, "motion_h": H, **extra},
                                 stream_id=f"motion/{name}", group=name)
    if "mask" in roles:
        sender.send_numpy_mask(base + 300, mask, metadata={"classes": {str(k): v for k, v in classes.items()},
                                                           "material_params": {"NumSegmentIDs": max(classes) + 1}, **extra},
                               stream_id=f"mask/{name}", group=name)


def sample_arrays():
    rng = np.random.default_rng(1)
    rgb = rng.integers(0, 255, (H, W, 3), dtype=np.uint8)
    depth = rng.uniform(10, 900, (H, W)).astype(np.float32)
    motion = rng.normal(0, 2, (H, W, 2)).astype(np.float32)
    mask = rng.integers(0, 8, (H, W), dtype=np.uint8)
    return rgb, depth, motion, mask


def test_camera_streams_decode_all_four_roles(tmp_path):
    client, sender, socks = pair()
    rgb, depth, motion, mask = sample_arrays()
    classes = {3: "Car", 7: "Curb"}
    with CameraStreams("front_cam", base_channel=0, client=client) as cam:
        assert cam.channels == {"color": 0, "depth": 100, "motion": 200, "mask": 300}
        send_camera(sender, "rear_cam", 1, rgb, depth, motion, mask, classes)      # another camera: ignored
        send_camera(sender, "front_cam", 0, rgb, depth, motion, mask, classes)
        frames = cam.grab(timeout=5.0)
    client.disconnect()
    for s in socks:
        s.close()
    assert set(frames) == {"color", "depth", "motion", "mask"}
    c, d, mo, ma = frames["color"], frames["depth"], frames["motion"], frames["mask"]
    assert c.array.shape == (H, W, 3) and c.array.dtype == np.uint8 and np.array_equal(c.array, rgb)
    assert c.camera == "front_cam" and c.stream_id == "camera/front_cam/color" and c.channel == 0
    assert d.array.dtype == np.float32 and np.array_equal(d.array, depth) and d.channel == 100
    assert mo.array.shape == (H, W, 2) and np.array_equal(mo.array, motion)
    assert ma.array.dtype == np.uint8 and np.array_equal(ma.array, mask)
    assert ma.classes == classes and ma.num_segment_ids == 8 and ma.fmt == "mono8"
    files = save_frame(ma, tmp_path, 0) + save_frame(mo, tmp_path, 0) + save_frame(c, tmp_path, 0)
    names = sorted(p.name for p in files)
    assert "motion_000000.npy" in names and "mask_000000.json" in names
    assert np.array_equal(np.load(tmp_path / "motion_000000.npy"), motion)
    if (tmp_path / "mask_000000.png").exists():
        from PIL import Image
        assert np.array_equal(np.asarray(Image.open(tmp_path / "mask_000000.png")), mask)
        assert np.array_equal(np.asarray(Image.open(tmp_path / "color_000000.png")), rgb)
    meta = json.loads((tmp_path / "mask_000000.json").read_text())
    assert meta["classes"] == {"3": "Car", "7": "Curb"} and meta["_shape"] == [H, W]


def test_camera_streams_by_stream_id_without_channels():
    client, sender, socks = pair()
    rgb, depth, motion, mask = sample_arrays()
    status = StreamStatus.from_json({"server": {"port": 30030}, "streams": []})
    cam = CameraStreams.from_status(status, "front_cam", client=client, roles=("depth", "mask"))
    assert cam.base_channel is None and cam.channels == {}
    with cam:
        send_camera(sender, "front_cam", 7, rgb, depth, motion, mask, {1: "Road"})
        frames = cam.grab(timeout=5.0)
    client.disconnect()
    for s in socks:
        s.close()
    assert set(frames) == {"depth", "mask"} and frames["mask"].channel == 307


def test_from_status_finds_the_base_channel():
    st = StreamStatus.from_json({"server": {"running": True, "port": 31000}, "streams": [
        {"camera": "front_cam", "role": "depth", "stream_id": "camera/front_cam/depth", "channel": 102}]})
    cam = CameraStreams.from_status(st, "front_cam")
    assert cam.base_channel == 2 and cam.channels["mask"] == 302 and cam.client.port == 31000


def test_status_without_a_server_and_cameras_on_two_robots():
    stopped = StreamStatus.from_json({"ok": True, "server": {"running": False, "port": 0, "connections": 0}, "streams": []})
    assert stopped.port is None and not stopped.running
    with pytest.raises(SensingError, match="RMSS server is not running"):
        CameraStreams.from_status(stopped, "front_camera")
    a1, a2 = "/Game/M.M:PersistentLevel.Gen1_C_0", "/Game/M.M:PersistentLevel.Chair_C_0"
    st = StreamStatus.from_json({"server": {"running": True, "port": 30030}, "streams": [
        {"actor": a1, "camera": "front_camera", "role": "color", "stream_id": "camera/front_camera/color", "channel": 0},
        {"actor": a2, "camera": "front_camera", "role": "color", "stream_id": "camera/front_camera/color", "channel": 1},
        {"actor": a2, "camera": "", "target": "imu", "role": "imu", "stream_id": "imu/Chair_C_0/imu", "channel": 1000,
         "message": "FrameData", "fmt": "imu_f64x18"}]})
    assert st.camera_actors("front_camera") == [a2, a1] and st.cameras(a1) == ["front_camera"]
    with pytest.raises(SensingError, match="stream on 2 actors"):
        CameraStreams.from_status(st, "front_camera")
    cam = CameraStreams.from_status(st, "front_camera", actor="Chair_C_0")          # a name will do
    assert cam.base_channel == 1 and cam.actor == "Chair_C_0"
    assert [s.target for s in st.imu_streams(a2)] == ["imu"] and st.imu_streams(a1) == []
    from ramms_tools.sensing.bridge import same_actor
    assert same_actor(a1, "Gen1_C_0") and not same_actor(a1, a2) and not same_actor("", a1)
    assert not same_actor(a1, "/Game/Other.Other:PersistentLevel.Gen1_C_0")       # two worlds' actors


def test_a_camera_of_another_robot_is_never_mixed_in():
    """Two robots stream a "front_cam" (the same stream ids) on their own channels; without a known base channel the
    client subscribes to everything and keeps only its actor's frames."""
    client, sender, socks = pair()
    rgb, depth, motion, mask = sample_arrays()
    other = rgb[::-1].copy()
    a1, a2 = "/Game/M.M:PersistentLevel.Gen1_C_0", "/Game/M.M:PersistentLevel.Gen2_C_0"
    with CameraStreams("front_cam", actor=a1, client=client, roles=("color",)) as cam:
        send_camera(sender, "front_cam", 5, other, depth, motion, mask, {1: "Road"}, actor=a2, roles=("color",))
        send_camera(sender, "front_cam", 0, rgb, depth, motion, mask, {1: "Road"}, actor=a1, roles=("color",))
        got = cam.grab(timeout=5.0)
    client.disconnect()
    for s in socks:
        s.close()
    assert np.array_equal(got["color"].array, rgb) and got["color"].actor == a1 and cam.other_actor == 1


def test_grab_returns_the_newest_complete_capture():
    client, sender, socks = pair()
    rgb, depth, motion, mask = sample_arrays()
    with CameraStreams("front_cam", base_channel=0, client=client, roles=("color", "depth")) as cam:
        send_camera(sender, "front_cam", 0, rgb, depth, motion, mask, {1: "Road"}, frame=7, roles=("color", "depth"))
        send_camera(sender, "front_cam", 0, rgb[::-1].copy(), depth * 2, motion, mask, {1: "Road"}, frame=8,
                    roles=("color", "depth"))
        send_camera(sender, "front_cam", 0, rgb, depth, motion, mask, {1: "Road"}, frame=9, roles=("color",))
        wait_queued(client, 5)                                # all queued: grab reads the queue first
        got = cam.grab(timeout=0.5)
    client.disconnect()
    for s in socks:
        s.close()
    # Frame 9 has no depth yet: the newest complete capture is 8, colour and depth of the same capture.
    assert got["color"].frame_number == got["depth"].frame_number == 8
    assert np.array_equal(got["depth"].array, depth * 2)
    assert isinstance(got, FrameSet) and got.complete and got.frame_number == 8 and got.missing == ()


def wait_queued(client, n, timeout=10.0):
    """Until the client has n messages queued (a loaded machine is slow to deliver over the socket pair)."""
    end = time.monotonic() + timeout
    while len(client._queue) < n:
        assert time.monotonic() < end, f"only {len(client._queue)} of {n} messages arrived"
        time.sleep(0.01)


def test_grab_at_timeout_is_one_capture_flagged_incomplete():
    """No capture complete by the timeout: the newest capture with the most roles, flagged -- never colour of one
    capture and depth of another passed off as a set."""
    client, sender, socks = pair()
    rgb, depth, motion, mask = sample_arrays()
    with CameraStreams("front_cam", base_channel=0, client=client, roles=("color", "depth", "mask")) as cam:
        send_camera(sender, "front_cam", 0, rgb, depth, motion, mask, {1: "Road"}, frame=7, roles=("color", "mask"))
        send_camera(sender, "front_cam", 0, rgb, depth, motion, mask, {1: "Road"}, frame=8, roles=("depth",))
        wait_queued(client, 3)
        got = cam.grab(timeout=0.3)
    client.disconnect()
    for s in socks:
        s.close()
    assert not got.complete and got.same_capture and got.frame_number == 7
    assert set(got) == {"color", "mask"} and got.missing == ("depth",)
    assert all(f.frame_number == 7 for f in got.values())


def test_from_sensor_takes_the_stream_ids_the_sim_reports():
    """A camera placed by hand streams under its stream source's StreamName (bExactCameraFilter), not its component
    name: the ids ListSensorsJson reports are the ones to match."""
    client, sender, socks = pair()
    rgb, depth, motion, mask = sample_arrays()
    info = SensorInfo.from_json({"name": "FR_GeminiCamera", "kind": "Camera", "rig": False, "actor": "/Game/M.M:PersistentLevel.R_C_0",
                                 "settings": {"channel": 4}, "streams": [
                                     {"role": "color", "stream_id": "camera/front/color", "channel": 4},
                                     {"role": "mask", "stream_id": "mask/front", "channel": 304}]})
    with CameraStreams.from_sensor(info, client=client) as cam:
        assert cam.group == "front" and cam.stream_ids == {"color": "camera/front/color", "mask": "mask/front"}
        send_camera(sender, "front", 4, rgb, depth, motion, mask, {1: "Road"}, roles=("color", "mask"))
        got = cam.grab(timeout=5.0)
    client.disconnect()
    for s in socks:
        s.close()
    assert set(got) == {"color", "mask"} and np.array_equal(got["color"].array, rgb)


def test_a_stream_client_that_has_not_subscribed_says_so(caplog):
    """The sim sends a connection no frames until it has subscribed."""
    client, sender, socks = pair()
    try:
        with caplog.at_level(logging.WARNING):
            assert client.poll(0) is None and client.poll(0) is None
        assert caplog.text.count("has not subscribed") == 1 and not client.subscribed
        client.subscribe()
        assert client.subscribed
    finally:
        client.disconnect()
        for s in socks:
            s.close()


def test_camera_streams_take_a_combined_rgbd_frame():
    client, sender, socks = pair()
    rgb = np.arange(W * H * 3, dtype=np.uint8).reshape(H, W, 3)
    depth = np.full((H, W), 123.5, np.float32)
    bgra = np.concatenate([rgb[:, :, ::-1], np.full((H, W, 1), 255, np.uint8)], axis=2)
    m = StreamMessage()
    m.header.message_type = MessageType.FRAME_RGBD
    m.header.channel_id = 3
    m.metadata = json.dumps({"w": W, "h": H, "depth_w": W, "depth_h": H, "rgb_size": W * H * 4, "frame": 4,
                             "actor": "/Game/M.M:PersistentLevel.Chair_C_0"}).encode()
    m.payload = bgra.tobytes() + depth.tobytes()
    st = StreamStatus.from_json({"server": {"running": True, "port": 30030}, "streams": [
        {"actor": "/Game/M.M:PersistentLevel.Chair_C_0", "camera": "cam", "role": "rgbd", "stream_id": "", "channel": 3}]})
    cam = CameraStreams.from_status(st, "cam", client=client, roles=("color", "depth"))
    assert cam.base_channel == 3 and cam.rgbd
    with cam:
        sender._send_msg(m)
        got = cam.grab(timeout=5.0)
    client.disconnect()
    for s in socks:
        s.close()
    assert np.array_equal(got["color"].array, rgb) and np.array_equal(got["depth"].array, depth)
    assert got["color"].camera == "cam" and got["depth"].frame_number == 4


def test_compressed_frames_are_skipped_with_one_warning(caplog):
    from ramms_tools.streaming.protocol import Compression as C
    cam = CameraStreams("front_cam", base_channel=0, roles=("color",))
    rgb = sample_arrays()[0]
    for _ in range(5):
        m = color_message("front_cam", 0, rgb)
        m.header.set_compression(C.JPEG)
        cam.client._queue.append(m)
    cam.client._queue_event.set()
    with caplog.at_level("WARNING"):
        assert cam.poll(0) is None
    assert cam.skipped == 5 and len([r for r in caplog.records if "skipping frames" in r.message]) == 1


def test_frame_helpers():
    assert camera_stream_ids("c") == {"color": "camera/c/color", "depth": "camera/c/depth", "motion": "motion/c", "mask": "mask/c"}
    assert camera_channels(5) == {"color": 5, "depth": 105, "motion": 205, "mask": 305}
    m = StreamMessage()
    m.header.message_type = MessageType.FRAME_DATA
    m.metadata = json.dumps({"w": 2, "h": 1, "fmt": "mono8", "stream_id": "mask/c"}).encode()
    m.payload = bytes([1, 2])
    assert message_role(m) == "mask"
    f = decode_message(m)
    assert f.array.tolist() == [[1, 2]] and f.classes == {}
    m.payload = bytes([1, 2, 3])
    with pytest.raises(ValueError, match="3 bytes, expected 2"):
        decode_message(m)
    m.header.set_compression(Compression.LZ4)
    with pytest.raises(ValueError, match="compressed"):
        decode_message(m)
    assert mask_classes({"classes": [{"value": 4, "name": "Tree"}]}) == {4: "Tree"}
    m2 = StreamMessage()
    m2.header.message_type = MessageType.FRAME_DATA
    m2.metadata = json.dumps({"w": 1, "h": 1, "fmt": "mono8", "role": "mask", "MaterialScalarParameters": {"NumSegmentIDs": 9},
                              "material_params": {"NumSegmentIDs": 5}}).encode()
    m2.payload = bytes([3])
    assert decode_message(m2).num_segment_ids == 9          # the key StreamSender and rammp-ui use wins
    m2.metadata = json.dumps({"w": 1, "h": 1, "fmt": "mono8", "role": "mask", "material_params": {"NumSegmentIDs": 5}}).encode()
    assert decode_message(m2).num_segment_ids == 5
    # 16-bit millimetre depth comes out in cm
    d = StreamMessage()
    d.header.message_type = MessageType.FRAME_DEPTH
    d.metadata = json.dumps({"w": 2, "h": 1, "fmt": "16uc1"}).encode()
    d.payload = np.array([[1000, 25]], dtype="<u2").tobytes()
    assert decode_message(d).array.tolist() == [[100.0, 2.5]]
    # a combined RGBD splits into two frames
    r = StreamMessage()
    r.header.message_type = MessageType.FRAME_RGBD
    r.metadata = json.dumps({"w": 1, "h": 1, "rgb_size": 4}).encode()
    r.payload = bytes([30, 20, 10, 255]) + np.array([42.0], dtype="<f4").tobytes()
    color, depth = decode_rgbd(r)
    assert color.array.tolist() == [[[10, 20, 30]]] and depth.array.tolist() == [[42.0]]


def test_stream_cli_saves_motion_and_masks(tmp_path):
    from ramms_tools.scripts import stream_video
    rgb, depth, motion, mask = sample_arrays()
    msgs = []
    cap = StreamSender(queue_size=0)
    cap._send_msg = lambda m: msgs.append(m)
    cap._sock = object()
    send_camera(cap, "front_cam", 0, rgb, depth, motion, mask, {3: "Car"})
    for m in msgs:
        stream_video._save_frame(tmp_path, m.header.channel_id, m, 1)
    assert np.array_equal(np.load(tmp_path / "channel_200" / "frame_0000001.npy"), motion)
    mask_png = tmp_path / "channel_300" / "frame_0000001.png"
    if mask_png.exists():
        from PIL import Image
        assert np.array_equal(np.asarray(Image.open(mask_png)), mask)
    else:
        assert np.array_equal(np.load(tmp_path / "channel_300" / "frame_0000001.npy"), mask)
    assert (tmp_path / "channel_00" / "frame_0000001.raw").exists()               # colour stays raw
    assert (tmp_path / "channel_100" / "frame_0000001.depth").exists()
    assert stream_video._stream_label(msgs[2], msgs[2].get_metadata_json()) == ("motion/front_cam", "motion")


# ── IMU from the physics engine (no component) ───────────────────────────

def test_body_imu_waits_for_a_sample_and_parses_the_engine_fields():
    ue, path, bridge = sensing_world()
    s = Sensing(ue)
    first = s.body_imu(path, "chassis", offset=(50, 0, 15))
    assert first.has_sample is False and first.has_acceleration is False and "no acceleration yet" in first.summary() \
        or first.has_sample is False
    r = s.body_imu(path, "chassis", offset=(80, 0, 10), rotation=(-20, 0, 0), wait=1.0)   # a new point: waits for one
    assert r.has_acceleration is True and r.source == "mujoco" and r.body == "chassis" and r.kind == "IMU"
    assert r.linear_acceleration == (-23.0, 0.0, 981.0) and r.angular_acceleration == (0.0, 0.0, 2.0)
    assert r.linear_velocity == (50.0, 0.0, 0.0) and r.sim_time == pytest.approx(4.9)
    assert r.linear_acceleration_ms2() == pytest.approx((-0.23, 0.0, 9.81))
    assert "[mujoco]" in r.summary() and r.flat()["source"] == "mujoco"
    sent = json.loads(ue.calls_to("ReadBodyImuJson")[-1]["OptionsJson"])
    assert sent == {"source": "auto", "offset": [80.0, 0.0, 10.0], "rotation": [-20.0, 0.0, 0.0]}
    with pytest.raises(SensingError, match="no body 'nope'"):
        s.body_imu(path, "nope")
    robot = Robot.find(ue=ue)
    assert [b["name"] for b in robot.sensors.imu_bodies()] == ["chassis", "seat"]
    assert robot.sensors.body_imu(source="kinematic", wait=1.0).source == "kinematic"


def test_cli_imu_and_bodies(capsys):
    ue, path, bridge = sensing_world()
    from ramms_tools.scripts import sensors_cli
    import ramms_tools.scripts._common as common
    orig = common.connect
    try:
        sensors_cli.connect = lambda a: ue
        assert sensors_cli.main(["bodies"]) == 0
        out = capsys.readouterr().out
        assert "* chassis" in out and "mujoco" in out and "seat" in out
        assert sensors_cli.main(["imu", "--body", "chassis", "--offset", "50,0,15"]) == 0
        out = capsys.readouterr().out
        assert "chassis" in out and "[mujoco]" in out and "v (50.0, 0.0, 0.0) cm/s" in out
    finally:
        sensors_cli.connect = orig


# ── IMU samples at the physics rate: buffered over Remote Control, streamed over RMSS ──

def test_read_imu_samples_cursor_and_drops():
    ue, path, bridge = sensing_world()
    s = Sensing(ue)
    first = s.read_imu_samples(path, "chassis")
    assert first.samples == [] and first.next >= 0 and first.source == "mujoco"
    sent = json.loads(ue.calls_to("ReadImuSamplesJson")[-1]["OptionsJson"])
    assert sent == {"since": -1, "max": 1024}                       # source auto, no pose: not sent
    time.sleep(0.05)
    res = s.read_imu_samples(path, "chassis", since=first.next, offset=(10, 0, 0), source="mujoco")
    sent = json.loads(ue.calls_to("ReadImuSamplesJson")[-1]["OptionsJson"])
    assert sent["offset"] == [10.0, 0.0, 0.0] and sent["source"] == "mujoco" and sent["since"] == first.next
    seqs = [x.seq for x in res.samples]
    assert seqs and seqs == list(range(first.next, res.next)) and res.dropped == 0 and res.rate_hz == 500.0
    smp = res.samples[0]
    assert smp.linear_acceleration == (0.5, 0.0, 981.0) and smp.orientation == (0, 0, 0, 1)
    assert smp.linear_acceleration_ms2() == pytest.approx((0.005, 0.0, 9.81)) and smp.flat()["qw"] == 1
    bridge.IMU_BUFFER = 5                                           # asked for more than the sim still keeps
    late = s.read_imu_samples(path, "chassis", since=0)
    assert late.dropped > 0 and late.samples[0].seq == late.dropped and len(late.samples) <= 5
    with pytest.raises(SensingError, match="no IMU sensor or body"):
        s.read_imu_samples(path, "nope")


def test_follow_imu_samples_yields_every_step():
    ue, path, bridge = sensing_world()
    robot = Robot.find(ue=ue)
    follower = robot.sensors.follow_imu_samples("chassis", period=0.02, duration=0.3)
    seqs = [x.seq for x in follower]
    assert len(seqs) > 50 and seqs == list(range(seqs[0], seqs[0] + len(seqs)))   # none missing, none twice
    assert follower.dropped == 0 and follower.count == len(seqs) and follower.last.rate_hz == 500.0
    bridge.IMU_BUFFER = 10                                          # a poll slower than the buffer loses samples
    follower = Sensing(ue).follow_imu_samples(path, "chassis", period=0.1, duration=0.35)
    list(follower)
    assert follower.dropped > 0


def imu_message(channel, actor, target, seqs, dropped=0):
    rows = np.array([[q, q / 500.0, 0.5, 0, 981.0, 0, 0, q * 0.01, 0, 0, 5.0, 10.0, 0, 0, 0, 0, 0, 1] for q in seqs],
                    dtype="<f8")
    m = StreamMessage()
    m.header.message_type = MessageType.FRAME_DATA
    m.header.channel_id = channel
    m.metadata = json.dumps({"w": len(seqs), "h": 1, "fmt": "imu_f64x18", "stream_id": f"imu/{actor.rsplit('.', 1)[-1]}/{target}",
                             "group": actor.rsplit(".", 1)[-1], "role": "imu", "actor": actor, "target": target,
                             "body": target, "source": "mujoco", "first_seq": seqs[0] if seqs else -1, "count": len(seqs),
                             "dropped": dropped, "fields": list(IMU_FIELDS)}).encode()
    m.payload = rows.tobytes()
    return m


def test_imu_stream_frames_decode():
    m = imu_message(1000, "/Game/M.M:PersistentLevel.Gen1_C_0", "chassis", [10, 11, 12])
    assert message_role(m) == "imu"
    f = decode_message(m)
    assert f.role == "imu" and f.array.shape == (3, 18) and f.array.dtype == np.float64 and f.array[2, 0] == 12
    assert list(IMU_FIELDS[:2]) == ["seq", "sim_time"] and len(IMU_FIELDS) == 18
    m.payload = m.payload[:-8]
    with pytest.raises(ValueError, match="imu 3 samples"):
        decode_message(m)


def test_start_imu_stream_and_read_it_over_rmss():
    ue, path, bridge = sensing_world()
    s = Sensing(ue)
    info = s.start_imu_stream(path, "chassis")
    assert info.channel == 1000 and info.stream_id == "imu/BP_RammpGen1PoweredFront_Ramms_C_0/chassis"
    assert info.target == "chassis" and info.actor == path and info.port == 30030
    assert [x.target for x in s.status().imu_streams(path)] == ["chassis"]
    client, sender, socks = pair()
    other = "/Game/Maps/Map_Test.Map_Test:PersistentLevel.Other_C_0"
    with ImuStream.from_info(info, client=client) as stream:
        sender._send_msg(imu_message(1000, path, "chassis", [1, 2, 3]))
        sender._send_msg(imu_message(1000, other, "chassis", [7, 8]))           # another actor on the channel: ignored
        sender._send_msg(imu_message(1000, path, "chassis", [6, 7], dropped=2))  # 4 and 5 lost
        got = list(stream.samples(timeout=2.0, duration=1.0))
    client.disconnect()
    for sock in socks:
        sock.close()
    assert [x.seq for x in got] == [1, 2, 3, 6, 7] and stream.dropped == 2 and stream.count == 5
    assert got[0].angular_velocity == (0.0, 0.0, 0.01) and got[-1].sim_time == pytest.approx(7 / 500)
    assert s.stop_imu_stream(path, "chassis") and not s.stop_imu_stream(path, "chassis")


def test_start_imu_stream_reads_each_answer_shape():
    ue = FakeRemote()
    path = "/Game/M.M:PersistentLevel.R_C_0"
    for answer in ({"ok": True, "stream": {"channel": 1001, "stream_id": "imu/R_C_0/imu", "target": "imu"}, "port": 31000},
                   {"ok": True, "channel": 1002, "stream_id": "imu/R_C_0/imu", "server": {"running": True, "port": 31001}}):
        ue.objects[BRIDGE] = lambda f, p, a=answer: {"ReturnValue": json.dumps(a)}
        info = Sensing(ue).start_imu_stream(path, "imu")
        assert info.channel in (1001, 1002) and info.role == "imu" and info.port in (31000, 31001) and info.actor == path
    ue.objects[BRIDGE] = lambda f, p: {"ReturnValue": json.dumps({"ok": False, "error": "StartImuStreamJson needs a play world (PIE or a game)"})}
    with pytest.raises(SensingError, match="needs a play world"):
        Sensing(ue).start_imu_stream(path, "imu")


def test_cli_imu_samples_csv_and_stream(capsys):
    ue, path, bridge = sensing_world()
    from ramms_tools.scripts import sensors_cli
    import ramms_tools.scripts._common as common
    orig = sensors_cli.connect
    try:
        sensors_cli.connect = lambda a: ue
        assert sensors_cli.main(["imu", "--body", "chassis", "--samples", "--period", "0.02", "--duration", "0.2",
                                 "--format", "csv"]) == 0
        cap = capsys.readouterr()
        lines = cap.out.splitlines()
        assert lines[0] == ",".join(IMU_FIELDS) and len(lines) > 40
        seqs = [int(float(ln.split(",")[0])) for ln in lines[1:]]
        assert seqs == list(range(seqs[0], seqs[0] + len(seqs)))
        assert "samples of chassis over Remote Control, 500 Hz" in cap.err
        # --stream: the sim starts the stream; with no RMSS server reachable the stream is still stopped at the end.
        bridge.server_running = True
        assert sensors_cli.main(["imu", "--body", "chassis", "--stream", "--stream-host", "127.0.0.1",
                                 "--stream-port", "1", "--duration", "0.1"]) == 1
        assert "RMSS" in capsys.readouterr().err and getattr(bridge, "imu_streams", {}) == {}
    finally:
        sensors_cli.connect = orig


def test_cli_imu_stream_prints_every_sample(capsys):
    """`imu --stream` against a local test RMSS server (not the engine): every sample, then the stream is stopped."""
    ue, path, bridge = sensing_world()
    from ramms_tools.scripts import sensors_cli
    server = socket.create_server(("127.0.0.1", 0))
    port = server.getsockname()[1]

    def serve():
        conn, _ = server.accept()
        with conn:
            conn.recv(65536)                                   # the SUBSCRIBE
            snd = StreamSender(queue_size=0)
            snd._sock = conn
            for first in (100, 103, 106):
                snd._send_msg(imu_message(1000, path, "chassis", [first, first + 1, first + 2]))
            try:
                conn.recv(65536)
            except OSError:
                pass
    threading.Thread(target=serve, daemon=True).start()
    orig = sensors_cli.connect
    try:
        sensors_cli.connect = lambda a: ue
        assert sensors_cli.main(["imu", "--body", "chassis", "--stream", "--stream-host", "127.0.0.1", "--stream-port",
                                 str(port), "--duration", "1.0", "--format", "json"]) == 0
    finally:
        sensors_cli.connect = orig
        server.close()
    cap = capsys.readouterr()
    rows = [json.loads(ln) for ln in cap.out.splitlines()]
    assert [int(r["seq"]) for r in rows] == list(range(100, 109)) and rows[0]["target"] == "chassis"
    assert "9 samples of chassis over RMSS (imu/" in cap.err and getattr(bridge, "imu_streams", {}) == {}


def test_stream_cli_saves_imu_frames(tmp_path):
    from ramms_tools.scripts import stream_video
    m = imu_message(1000, "/Game/M.M:PersistentLevel.Gen1_C_0", "chassis", [5, 6])
    stream_video._save_frame(tmp_path, 1000, m, 1)
    arr = np.load(tmp_path / "channel_1000" / "frame_0000001.npy")
    assert arr.shape == (2, 18) and arr[1, 0] == 6
