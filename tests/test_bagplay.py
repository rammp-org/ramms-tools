"""The bag player: what reaches RammpUI from a recorded RAMMP bag, and when.

Every test plays a small but real rosbag2 bag (tests/rosbag_fixtures.write_bag, CDR from
ramms_tools.rosbag.encode) laid out like the Nov 19 RAMMP bags: a scene camera whose optical frame
hangs off base_link through TF, a wrist camera on a moving link, the tracker's and the segmenter's
detections, and /joint_states. Nothing touches the network; the live sink is tested with fakes.
"""

from __future__ import annotations

import itertools
import math
import threading
import time

import numpy as np
import pytest

from ramms_tools.bagplay import BagSession, LiveSinks, PlayOptions, Player, RecordingSinks
from ramms_tools.bagplay import player as bp
from ramms_tools.bagplay.verify import check_markers, expected_marker, transform_point
from ramms_tools.ros_ue import camera_extrinsic
from ramms_tools.rosbag import encode
from rosbag_fixtures import write_bag

W, H = 8, 5
T0 = 1_795_133_217_000_000_000
STEP = 66_666_667          # 15 Hz
OPTICAL = {"x": -0.5, "y": 0.5, "z": -0.5, "w": 0.5}   # body (x fwd, y left, z up) -> optical (x right, y down, z fwd)
K = [4.0, 0.0, 4.0, 0.0, 4.0, 2.5, 0.0, 0.0, 1.0]


def hdr(stamp_ns, frame):
    return {"stamp": {"sec": stamp_ns // 1_000_000_000, "nanosec": stamp_ns % 1_000_000_000},
            "stamp_ns": stamp_ns, "frame_id": frame}


def image(stamp_ns, frame, encoding, bpp, fill):
    data = bytes([fill % 256]) * (W * H * bpp)
    return {"header": hdr(stamp_ns, frame), "height": H, "width": W, "encoding": encoding, "is_bigendian": 0,
            "step": W * bpp, "data": data}


def camera_info(stamp_ns, frame):
    return {"header": hdr(stamp_ns, frame), "height": H, "width": W, "distortion_model": "plumb_bob",
            "d": [0.0] * 5, "k": list(K), "r": [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
            "p": [4.0, 0, 4.0, 0, 0, 4.0, 2.5, 0, 0, 0, 1.0, 0], "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False}}


def tf(stamp_ns, parent, child, xyz, q):
    return {"header": hdr(stamp_ns, parent), "child_frame_id": child,
            "transform": {"translation": dict(zip("xyz", xyz)), "rotation": q}}


def detected(iid, cls, box, pos=(0.1, -0.05, 1.5), state=0, mask_id=0, has_pose=True):
    return {"instance_id": iid, "perception_class": cls, "score": 0.9, "state": state, "box": [float(b) for b in box],
            "mask_id": mask_id, "has_pose": has_pose,
            "pose": {"position": dict(zip("xyz", pos)), "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
            "size": {"x": 0.2, "y": 0.3, "z": 0.4}}


def detections(stamp_ns, objects, label=None):
    label_image = label if label is not None else {"header": hdr(stamp_ns, ""), "height": 0, "width": 0,
                                                   "encoding": "", "is_bigendian": 0, "step": 0, "data": b""}
    return {"header": hdr(stamp_ns, "scene_camera_color_optical_frame"), "objects": objects,
            "label_image": label_image}


def build(tmp_path, frames=6, wrist=True, odom=False, split=False, scene_encoding="bgr8"):
    """A bag of `frames` 15 Hz frames. The scene camera's body sits at (1, 0, 0.5) m in base_link, facing
    forward; the wrist camera rides an arm link that turns 10 degrees a frame about z."""
    types = {
        "/tf_static": "tf2_msgs/msg/TFMessage", "/tf": "tf2_msgs/msg/TFMessage",
        bp.SCENE_COLOR: "sensor_msgs/msg/Image", bp.SCENE_DEPTH: "sensor_msgs/msg/Image",
        bp.SCENE_INFO: "sensor_msgs/msg/CameraInfo", bp.SCENE_DEPTH_INFO: "sensor_msgs/msg/CameraInfo",
        bp.TRACKED: "rammp_perception_interfaces/msg/ObjectDetections",
        bp.RAW: "rammp_perception_interfaces/msg/ObjectDetections",
        bp.JOINTS: "sensor_msgs/msg/JointState",
    }
    if wrist:
        types.update({bp.WRIST_COLOR: "sensor_msgs/msg/Image", bp.WRIST_DEPTH: "sensor_msgs/msg/Image",
                      bp.WRIST_INFO: "sensor_msgs/msg/CameraInfo", bp.WRIST_DEPTH_INFO: "sensor_msgs/msg/CameraInfo"})
    msgs = []

    def add(topic, ts, msg):
        msgs.append((topic, ts, encode(types[topic], msg)))
    statics = [tf(T0 - 10**12, "base_link", "scene_camera", (1.0, 0.0, 0.5), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
               tf(T0 - 10**12, "scene_camera", "scene_camera_color_optical_frame", (0, 0, 0), OPTICAL),
               tf(T0 - 10**12, "arm_link", "wrist_camera_color_optical_frame", (0.2, 0, 0), OPTICAL)]
    add("/tf_static", T0, {"transforms": statics})
    for i in range(frames):
        ts = T0 + i * STEP
        angle = math.radians(10 * i)
        dyn = [tf(ts, "base_link", "arm_link", (0.5, 0.0, 1.0), {"x": 0.0, "y": 0.0, "z": math.sin(angle / 2),
                                                                 "w": math.cos(angle / 2)})]
        if odom:
            dyn.append(tf(ts, "odom", "base_link", (0.1 * i, 0, 0), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}))
        add("/tf", ts, {"transforms": dyn})
        add(bp.SCENE_COLOR, ts + 1000, image(ts, "scene_camera_color_optical_frame", scene_encoding, 3, i))
        add(bp.SCENE_DEPTH, ts + 2000, image(ts, "scene_camera_color_optical_frame", "16UC1", 2, i))
        add(bp.SCENE_INFO, ts + 3000, camera_info(ts, "scene_camera_color_optical_frame"))
        add(bp.SCENE_DEPTH_INFO, ts + 3000, camera_info(ts, "scene_camera_color_optical_frame"))
        if wrist:
            add(bp.WRIST_COLOR, ts + 4000, image(ts, "wrist_camera_color_optical_frame", "bgr8", 3, i))
            add(bp.WRIST_DEPTH, ts + 5000, image(ts, "wrist_camera_color_optical_frame", "16UC1", 2, i))
            add(bp.WRIST_INFO, ts + 6000, camera_info(ts, "wrist_camera_color_optical_frame"))
            add(bp.WRIST_DEPTH_INFO, ts + 6000, camera_info(ts, "wrist_camera_color_optical_frame"))
        if i % 3 == 0:  # the 5 Hz segmenter: a cup and a fridge, masks 1 and 2
            label_data = np.zeros((H, W), dtype="<u2")
            label_data[0:2, 0:3] = 1
            label_data[3:5, 4:8] = 2
            label = {"header": hdr(ts, "scene_camera_color_optical_frame"), "height": H, "width": W,
                     "encoding": "mono16", "is_bigendian": 0, "step": W * 2, "data": label_data.tobytes()}
            add(bp.RAW, ts + 7000, detections(ts, [detected("drink_cup_1", "drink cup", (0, 0, 3, 2), mask_id=1,
                                                             has_pose=False),
                                                    detected("fridge_2", "fridge", (4, 3, 8, 5), mask_id=2,
                                                             has_pose=False)], label))
        add(bp.TRACKED, ts + 8000, detections(ts, [detected("drink_cup_7", "drink cup", (0, 0, 3, 2)),
                                                   detected("fridge_9", "fridge", (0, 0, 0, 0), state=1)]))
        add(bp.JOINTS, ts + 9000, {"header": hdr(ts, ""),
                                   "name": [f"arm_joint_{n}" for n in range(1, 8)] + ["arm_robotiq_85_left_knuckle_joint"],
                                   "position": [0.1 * n for n in range(1, 8)] + [0.3965],
                                   "velocity": [0.0] * 8, "effort": [0.0] * 8})
        add(bp.JOINTS, ts + 30_000_000, {"header": hdr(ts + 30_000_000, ""),
                                         "name": ["arm_joint_1"], "position": [0.5], "velocity": [0.0],
                                         "effort": [0.0]})
    msgs.sort(key=lambda m: m[1])
    return write_bag(str(tmp_path / "bag"), types, msgs, split_at_ns=T0 + 3 * STEP if split else None)


class FakeClock:
    def __init__(self):
        self.now = 100.0
        self.slept = 0.0

    def clock(self):
        return self.now

    def sleep(self, s):
        self.slept += s
        self.now += s


def play(path, **options):
    session = BagSession(path, log=lambda *a: None)
    sinks = RecordingSinks()
    fc = FakeClock()
    stats = Player(session, sinks, PlayOptions(**options), clock=fc.clock, sleep=fc.sleep, log=lambda *a: None).run()
    return session, sinks, stats, fc


# ── cameras ──────────────────────────────────────────────────────────

def test_scene_colour_and_depth_carry_intrinsics_and_the_cameras_pose(tmp_path):
    session, sinks, stats, _ = play(build(tmp_path), overlay="none")
    colour = [f for f in sinks.frames if f["stream_id"] == "camera/scene/color"]
    depth = [f for f in sinks.frames if f["stream_id"] == "camera/scene/depth"]
    assert len(colour) == len(depth) == 6
    assert (colour[0]["channel"], depth[0]["channel"]) == (0, 1)
    assert colour[0]["group"] == depth[0]["group"] == "camera/scene"
    assert (colour[0]["fmt"], depth[0]["fmt"]) == ("bgr8", "16uc1")
    assert colour[0]["bytes"] == W * H * 3 and depth[0]["bytes"] == W * H * 2
    for f in (colour[0], depth[0]):
        assert f["metadata"]["intrinsics"] == {"fx": 4.0, "fy": 4.0, "cx": 4.0, "cy": 2.5}
        # the body frame sits 1 m ahead and 0.5 m up of base_link, facing forward: UE cm, no rotation
        t = f["metadata"]["transform"]
        assert (t["x"], t["y"], t["z"]) == pytest.approx((100.0, 0.0, 50.0), abs=1e-6)
        assert (t["roll"], t["pitch"], t["yaw"]) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
        assert f["metadata"]["transform_space"] == "relative"


def test_the_wrist_camera_takes_the_overlay_slot_with_its_moving_pose(tmp_path):
    _, sinks, _, _ = play(build(tmp_path))  # auto: the bag has a wrist camera
    wrist = [f for f in sinks.frames if f["stream_id"] == "engine/overlay"]
    wrist_depth = [f for f in sinks.frames if f["stream_id"] == "engine/overlay/depth"]
    assert len(wrist) == len(wrist_depth) == 6 and wrist[0]["channel"] == 2 and wrist_depth[0]["channel"] == 4
    assert wrist[0]["group"] == "overlay"
    yaws = [f["metadata"]["transform"]["yaw"] for f in wrist]
    # the arm link turns +10 deg a frame about ROS z (left); UE's yaw is the negated ROS yaw
    assert yaws == pytest.approx([-10.0 * i for i in range(6)], abs=1e-6)
    assert sinks.feed_calls == [True] and sinks.closed


def test_the_segmentation_overlay_is_the_scene_frame_and_one_value_per_class(tmp_path):
    session, sinks, _, _ = play(build(tmp_path), overlay="segmentation")
    overlay = [f for f in sinks.frames if f["stream_id"] == "engine/overlay"]
    masks = [f for f in sinks.frames if f["stream_id"] == "engine/overlay/segmentation"]
    assert len(overlay) == len(masks) == 2                     # the segmenter runs every third frame
    assert overlay[0]["bytes"] == W * H * 3 and "transform" not in overlay[0]["metadata"]  # no second projector
    assert masks[0]["fmt"] == "mono8" and masks[0]["channel"] == 3
    assert session.classes == ["drink cup", "fridge"]
    assert masks[0]["material_params"] == {"NumMaskIDs": 3.0, "MaskScale": 255.0}   # raw ids, not id/255


def test_segmentation_masks_carry_the_class_values(tmp_path):
    session = BagSession(build(tmp_path), log=lambda *a: None)
    captured = []

    class Sinks(RecordingSinks):
        def video(self, **kw):
            if kw["kind"] == "mask":
                captured.append(np.frombuffer(kw["data"], np.uint8).reshape(kw["height"], kw["width"]))
            super().video(**kw)
    fc = FakeClock()
    Player(session, Sinks(), PlayOptions(overlay="segmentation"), clock=fc.clock, sleep=fc.sleep,
           log=lambda *a: None).run()
    mask = captured[0]
    assert mask[0, 0] == session.class_value["drink cup"] == 1
    assert mask[4, 7] == session.class_value["fridge"] == 2
    assert mask[2, 2] == 0


def test_no_overlay_opens_no_feed(tmp_path):
    _, sinks, _, _ = play(build(tmp_path), overlay="none")
    assert sinks.feed_calls == [] and not [f for f in sinks.frames if f["stream_id"].startswith("engine/")]


def test_asking_for_a_wrist_camera_the_bag_lacks_is_refused(tmp_path):
    with pytest.raises(ValueError, match="no wrist camera"):
        play(build(tmp_path, wrist=False), overlay="wrist")


# ── objects ──────────────────────────────────────────────────────────

def test_every_tracked_message_is_one_push_with_2d_and_3d(tmp_path):
    _, sinks, _, _ = play(build(tmp_path), overlay="none")
    assert len(sinks.pushes) == 6
    push = sinks.pushes[0]
    assert push["stream_id"] == "camera/scene/color"
    cup, fridge = push["objects"]
    assert cup["box"] == pytest.approx([0, 0, 3 / W, 2 / H]) and cup["centroid"] == pytest.approx([1.5 / W, 1 / H])
    assert cup["position_m"] == pytest.approx({"X": 0.1, "Y": -0.05, "Z": 1.5}) and cup["frame"] == "camera"
    assert cup["size_m"] == pytest.approx({"X": 0.2, "Y": 0.3, "Z": 0.4})
    assert fridge["box"] == [0, 0, 0, 0] and "centroid" not in fridge     # occluded: a 3D memory, no 2D box


def test_staleness_counts_from_the_segmenters_last_sighting(tmp_path):
    _, sinks, _, _ = play(build(tmp_path), overlay="none")
    since = [p["objects"][0].get("time_since_detection_s") for p in sinks.pushes]
    # the segmenter saw the cup at frames 0 and 3; in between the tracker only carried it forward
    assert since == pytest.approx([0.0, STEP / 1e9, 2 * STEP / 1e9, 0.0, STEP / 1e9, 2 * STEP / 1e9], abs=1e-6)
    # never sighted by the segmenter (occluded in every tracked frame, no box): no claim of an age at all
    assert all("time_since_detection_s" not in p["objects"][1] for p in sinks.pushes)


def test_derived_positions_leave_the_3d_to_the_ui(tmp_path):
    _, sinks, stats, _ = play(build(tmp_path), overlay="none", positions="derived")
    assert all("position_m" not in o for p in sinks.pushes for o in p["objects"])
    assert stats.last_3d_payload["objects"][0]["has_pose"]   # kept for verifying the derived markers


def test_a_bag_without_odometry_warns_that_its_3d_is_frozen(tmp_path):
    _, _, stats, _ = play(build(tmp_path), overlay="none")
    assert any("frozen" in w for w in stats.warnings)
    _, _, stats, _ = play(build(tmp_path / "vo", odom=True), overlay="none")
    assert not any("frozen" in w for w in stats.warnings)


# ── joints ───────────────────────────────────────────────────────────

def test_joints_go_by_name_in_degrees_at_the_chosen_rate(tmp_path):
    _, sinks, _, _ = play(build(tmp_path), overlay="none", joints_hz=0)
    assert len(sinks.joint_sends) == 12                     # unthrottled: every /joint_states message
    _, sinks, _, _ = play(build(tmp_path / "b"), overlay="none", joints_hz=10.0)
    # messages at 0, 30, 67, 97, 133 ... 363 ms; at most one per 100 ms of bag time: 0, 133, 267
    assert len(sinks.joint_sends) == 3
    first = sinks.joint_sends[0]
    assert first["link_0"] == pytest.approx(-math.degrees(0.1)) and first["link_1"] == pytest.approx(math.degrees(0.2))
    assert first["link_0_l"] == pytest.approx(-22.5) and first["link_0_r"] == pytest.approx(22.5)  # half closed


def test_no_joints_sends_none(tmp_path):
    _, sinks, _, _ = play(build(tmp_path), overlay="none", joints=False)
    assert sinks.joint_sends == []


# ── the clock ────────────────────────────────────────────────────────

def test_playback_keeps_the_bags_time_scaled_by_the_rate(tmp_path):
    _, _, _, fc = play(build(tmp_path), overlay="none", rate=1.0)
    span = (5 * STEP + 30_000_000) / 1e9
    assert fc.slept == pytest.approx(span, abs=0.01)
    _, _, _, fc = play(build(tmp_path / "b"), overlay="none", rate=2.0)
    assert fc.slept == pytest.approx(span / 2, abs=0.01)


def test_start_and_end_select_a_window(tmp_path):
    _, sinks, _, _ = play(build(tmp_path), overlay="none", start_s=2 * STEP / 1e9, end_s=4 * STEP / 1e9)
    assert len([f for f in sinks.frames if f["stream_id"] == "camera/scene/color"]) == 2


def test_late_video_is_dropped_but_objects_and_joints_are_not(tmp_path):
    session = BagSession(build(tmp_path), log=lambda *a: None)
    sinks = RecordingSinks()
    ticks = itertools.count()

    def clock():
        return 100.0 + next(ticks) * 0.5   # every look at the clock is half a second later: always behind
    stats = Player(session, sinks, PlayOptions(overlay="none"), clock=clock, sleep=lambda s: None,
                   log=lambda *a: None).run()
    assert stats.late_dropped == 11            # every scene frame was late; the first colour went anyway,
    # since objects wait for the scene camera's first frame and must never be lost to lateness
    assert len(sinks.pushes) == 6               # objects are never dropped
    assert len(sinks.joint_sends) == 3          # nor joints: the 10 Hz throttle's 0, 133, 267 ms


def test_the_two_files_of_a_split_bag_play_in_order(tmp_path):
    _, sinks, _, _ = play(build(tmp_path, split=True), overlay="none")
    stamps = [f["metadata"]["transform"] and f for f in sinks.frames if f["stream_id"] == "camera/scene/color"]
    assert len(stamps) == 6


def test_the_ui_is_put_back_even_when_playback_is_interrupted(tmp_path):
    session = BagSession(build(tmp_path), log=lambda *a: None)

    class Interrupting(RecordingSinks):
        def objects(self, payload):
            raise KeyboardInterrupt
    sinks = Interrupting()
    with pytest.raises(KeyboardInterrupt):
        Player(session, sinks, PlayOptions(), clock=time.monotonic, sleep=lambda s: None, log=lambda *a: None).run()
    assert sinks.closed


# ── the live sink ────────────────────────────────────────────────────

class FakeClient:
    def __init__(self, delay=0.0):
        self.delay = delay
        self.calls = []
        self.lock = threading.Lock()

    def update_detected_objects(self, payload):
        time.sleep(self.delay)
        with self.lock:
            self.calls.append(("objects", payload))

    def open_camera_feed(self):
        self.calls.append(("feed", True))

    def close_camera_feed(self):
        self.calls.append(("feed", False))

    def call_object(self, path, function, **params):
        self.calls.append(("call_object", path, function, params["Values"]))

    def call(self, function, **params):
        self.calls.append(("call", function, params["Values"]))


class FakeSender:
    def __init__(self):
        self.sent = []
        self.disconnected = False

    def send_image(self, *a, **k):
        self.sent.append(("image", a, k))

    def send_depth_uint16(self, *a, **k):
        self.sent.append(("depth", a, k))

    def send_mask(self, *a, **k):
        self.sent.append(("mask", a, k))

    def disconnect(self, **k):
        self.disconnected = True


class FakeRig:
    world = "play session"
    pose_path = "/Game/Maps/UEDPIE_0_Map_UI.Map_UI:PersistentLevel.Display.RammsSkeletalPose"

    def __init__(self):
        self.names = ["drive_wheel_l", "link_0", "link_0_l", "link_0_r"]
        self.index = {n: i for i, n in enumerate(self.names)}
        self.joints = [{"JointName": n} for n in self.names]
        self.restored = None

    def targets(self):
        return [7.0, 1.0, 2.0, 3.0]

    def restore(self, original):
        self.restored = list(original)


def test_the_live_sink_sends_only_the_newest_detections_when_the_ui_is_slow():
    client = FakeClient(delay=0.05)
    sinks = LiveSinks(client, FakeSender(), log=lambda *a: None)
    for i in range(20):
        sinks.objects({"objects": [], "n": i})
    sinks.drain()
    sent = [c[1]["n"] for c in client.calls if c[0] == "objects"]
    assert sent[-1] == 19 and len(sent) < 20
    sinks.close()


def test_the_live_sink_sends_joints_by_name_holding_the_rest_and_restores_them():
    client, rig = FakeClient(), FakeRig()
    sinks = LiveSinks(client, FakeSender(), rig, log=lambda *a: None)
    sinks.joints({"link_0": 45.0, "link_0_l": -10.0, "not_on_this_display": 1.0})
    sinks.drain()
    call = [c for c in client.calls if c[0] == "call_object"][-1]
    assert call[1] == rig.pose_path and call[3] == [7.0, 45.0, -10.0, 3.0]   # the wheel and link_0_r held
    sinks.feed(True)
    sinks.close()
    kinds = [c[0] if c[0] != "feed" else ("feed", c[1]) for c in client.calls]
    assert ("feed", False) in kinds                                       # opened, so closed
    assert client.calls[-1] == ("feed", False) or rig.restored is not None
    assert [c for c in client.calls if c[0] == "objects"][-1][1]["objects"] == []   # detections cleared
    assert rig.restored == [7.0, 1.0, 2.0, 3.0] and sinks.sender.disconnected


def test_the_live_sink_routes_video_by_kind():
    sender = FakeSender()
    sinks = LiveSinks(FakeClient(), sender, log=lambda *a: None)
    sinks.video(channel=1, kind="depth", stream_id="d", group="g", width=2, height=1, data=b"\0" * 4, fmt="16uc1",
                metadata={"intrinsics": {}})
    sinks.video(channel=3, kind="mask", stream_id="m", group="g", width=2, height=1, data=b"\0\1", fmt="mono8",
                metadata={}, material_params={"NumMaskIDs": 3.0})
    sinks.video(channel=0, kind="color", stream_id="c", group="g", width=2, height=1, data=b"\0" * 6, fmt="bgr8",
                metadata={})
    assert [s[0] for s in sender.sent] == ["depth", "mask", "image"]
    assert sender.sent[1][2]["material_params"] == {"NumMaskIDs": 3.0} and sender.sent[2][2]["fmt"] == "bgr8"
    sinks.close()


# ── verification ─────────────────────────────────────────────────────

def test_expected_markers_compose_the_extrinsic_with_the_receiver():
    extrinsic = {"x": 100.0, "y": 0.0, "z": 50.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    obj = {"position_m": {"X": 0.0, "Y": 0.0, "Z": 2.0}}  # 2 m straight ahead of the camera
    at_origin = expected_marker(obj, extrinsic, {"location": [0, 0, 0], "rotation": [0, 0, 0]})
    assert at_origin == pytest.approx((300.0, 0.0, 50.0))
    turned = expected_marker(obj, extrinsic, {"location": [10, 20, 30], "rotation": [0, 90, 0]})
    assert turned == pytest.approx((10.0, 320.0, 80.0), abs=1e-6)   # yaw 90: forward becomes +Y


def test_marker_check_passes_where_the_bag_says_and_fails_elsewhere():
    extrinsic = {"x": 100.0, "y": 0.0, "z": 50.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    obj = {"instance_id": "cup_1", "perception_class": "cup", "position_m": {"X": 0.0, "Y": 0.0, "Z": 2.0}}
    state = {"payload": {"objects": [obj]}, "extrinsic": extrinsic, "objects": []}
    receiver = {"location": [0, 0, 0], "rotation": [0, 0, 0], "scale": [1, 1, 1]}
    results = []

    def check(ok, what, detail=""):
        results.append(ok)
    check_markers(state, {"receiver": receiver, "markers": [{"label": "cup", "location": [300.2, 0, 50]}]}, check)
    check_markers(state, {"receiver": receiver, "markers": [{"label": "cup", "location": [310, 0, 50]}]}, check)
    check_markers(state, {"receiver": receiver, "markers": []}, check)
    assert results == [True, False, False]


def test_transform_point_follows_frotator_axes():
    assert transform_point((0, 0, 0), (0, 90, 0), (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-9)
    assert transform_point((0, 0, 0), (90, 0, 0), (1, 0, 0)) == pytest.approx((0, 0, 1), abs=1e-9)


# ── the CLI ──────────────────────────────────────────────────────────

def test_info_reports_cameras_objects_and_the_frozen_3d(tmp_path, capsys):
    from ramms_tools.scripts.bag_cli import main
    assert main(["info", build(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "scene camera: 8x5 bgr8" in out and "x 100.00, y 0.00, z 50.00" in out
    assert "tracked objects: 2 ids: drink_cup_7, fridge_9" in out
    assert "WARNING: no odom->base_link" in out


def test_dry_run_sends_nothing_and_says_what_it_would(tmp_path, capsys):
    from ramms_tools.scripts.bag_cli import main
    assert main(["play", build(tmp_path), "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "scene_color 6" in out and "objects 6" in out and "overlay_color 6" in out


def test_a_zipped_bag_is_refused_with_the_command_to_extract_it(tmp_path, capsys):
    from ramms_tools.scripts.bag_cli import main
    z = tmp_path / "bag.zip"
    z.write_bytes(b"PK\x03\x04")
    assert main(["info", str(z)]) == 2
    assert "unzip" in capsys.readouterr().out


def test_derived_markers_are_checked_against_the_depth_they_were_derived_from(capsys):
    from ramms_tools.bagplay.verify import expected_derived_marker
    depth = np.full((40, 64), 2000, dtype=np.uint16)       # a wall 2 m ahead
    depth[18:23, 30:35] = 1000                             # and something 1 m ahead in the middle
    intrinsics = {"fx": 32.0, "fy": 32.0, "cx": 32.0, "cy": 20.0}
    extrinsic = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    receiver = {"location": [0, 0, 0], "rotation": [0, 0, 0], "scale": [1, 1, 1]}
    # the centre pixel (32, 20) sees the near thing: 100 cm straight ahead
    assert expected_derived_marker((0.5, 0.5), depth, 2, intrinsics, extrinsic, receiver) == pytest.approx((100.0, 0, 0))
    # a quarter across sees the wall, 1 m left of centre at 2 m ((16 - 32) * 200 / 32 = -100 cm, UE Y)
    assert expected_derived_marker((0.25, 0.5), depth, 2, intrinsics, extrinsic, receiver) == pytest.approx(
        (200.0, -100.0, 0.0))
    depth[:, :] = 0
    assert expected_derived_marker((0.5, 0.5), depth, 2, intrinsics, extrinsic, receiver) is None   # no return
    depth[:, :] = 2000
    obj = {"instance_id": "cup_1", "perception_class": "cup", "centroid": [0.5, 0.5], "box": [0.4, 0.4, 0.6, 0.6]}
    tracker_obj = {"instance_id": "cup_1", "has_pose": True,
                   "pose": {"position": {"x": 0.0, "y": 0.0, "z": 1.9}}}
    state = {"payload": {"objects": [obj]}, "extrinsic": extrinsic, "objects": [tracker_obj]}
    results = []
    # the segmenter's own boxes ride along to the overlay camera: nothing RammsUI places in 3D, so not checked
    state["payload"] = {"stream_id": "camera/scene/color", "objects": [
        {**obj, "source": "camera/scene/color"},
        {"instance_id": "seg:cup_3", "perception_class": "cup", "source": "engine/overlay", "centroid": [0.5, 0.5]}]}
    readback = {"receiver": receiver, "depth_sample_radius": 2, "markers": [{"label": "cup", "location": [200.3, 0, 0]}]}
    check_markers(state, readback, lambda ok, what, detail="": results.append(ok), positions="derived", depth=depth,
                  intrinsics=intrinsics)
    assert results == [True]
    assert "10 cm from the tracker's own 3D centre" in capsys.readouterr().out



def test_loop_plays_joints_on_every_pass(tmp_path):
    session = BagSession(build(tmp_path), log=lambda *a: None)

    class StopAfter(RecordingSinks):
        def joints(self, values):
            super().joints(values)
            if len(self.joint_sends) > 6:
                raise KeyboardInterrupt
    sinks = StopAfter()
    with pytest.raises(KeyboardInterrupt):
        Player(session, sinks, PlayOptions(overlay="none", rate=0, loop=True), log=lambda *a: None).run()
    assert len(sinks.joint_sends) == 7          # 3 a pass: the throttle starts again each pass


def test_a_looping_pass_lasts_its_whole_window_not_just_until_its_last_message(tmp_path):
    # Its messages end at 0.2 s of a 0.25 s window; the 15 us window holds one frame at its very start, and
    # paced only to its messages looped with no sleep at all.
    path = build(tmp_path)
    for end_s, period in ((0.25, 0.25), (0.000015, bp.MIN_LOOP_PASS_S)):
        fc = FakeClock()

        class Bounded(RecordingSinks):
            def video(self, *a, **k):
                super().video(*a, **k)
                if len(self.frames) >= 200:   # bounded however the pacing behaves
                    raise KeyboardInterrupt
        player = Player(BagSession(path, log=lambda *a: None), Bounded(),
                        PlayOptions(overlay="none", rate=1.0, loop=True, end_s=end_s),
                        clock=fc.clock, sleep=fc.sleep, log=lambda *a: None)
        with pytest.raises(KeyboardInterrupt):
            player.run()
        assert player.stats.passes >= 2
        assert fc.now - 100.0 >= player.stats.passes * period - 1e-9, (end_s, player.stats.passes, fc.now)


def test_an_empty_window_is_refused_rather_than_spun_on(tmp_path):
    with pytest.raises(ValueError, match="select no part"):
        play(build(tmp_path), overlay="none", start_s=500.0, loop=True)
    with pytest.raises(ValueError, match="none of the played topics"):
        play(build(tmp_path / "b"), overlay="none", start_s=0.0, end_s=0.0000001, loop=True)


def test_segmentation_mode_first_empties_the_overlays_depth(tmp_path):
    # Left as an earlier (wrist) replay set it, RammsUI derived 3D markers for every segmenter box from it.
    session = BagSession(build(tmp_path), log=lambda *a: None)

    class Ordered(RecordingSinks):
        def __init__(self):
            super().__init__()
            self.events, self.depth_data = [], []

        def video(self, channel, kind, stream_id, group, width, height, data, fmt, metadata, material_params=None):
            super().video(channel, kind, stream_id, group, width, height, data, fmt, metadata, material_params)
            self.events.append(("video", stream_id))
            if stream_id == "engine/overlay/depth":
                self.depth_data.append(bytes(data))

        def objects(self, payload):
            super().objects(payload)
            self.events.append(("objects", any(o["instance_id"].startswith("seg:") for o in payload["objects"])))
    sinks, fc = Ordered(), FakeClock()
    Player(session, sinks, PlayOptions(overlay="segmentation"), clock=fc.clock, sleep=fc.sleep,
           log=lambda *a: None).run()
    depth = [f for f in sinks.frames if f["stream_id"] == "engine/overlay/depth"]
    assert len(depth) == 1 and set(sinks.depth_data[0]) == {0}            # one frame, and no returns in it
    first = depth[0]
    assert (first["channel"], first["kind"], first["group"], first["width"], first["height"]) == (4, "depth", "overlay", 8, 5)
    # a real camera that saw nothing: the scene camera's intrinsics and pose, so RammsUI neither warns about
    # missing geometry nor about depth not registered to the colour
    scene = next(f for f in sinks.frames if f["stream_id"] == "camera/scene/color")
    assert first["metadata"]["intrinsics"] == scene["metadata"]["intrinsics"]
    assert first["metadata"]["transform"] == scene["metadata"]["transform"]
    clear = sinks.events.index(("video", "engine/overlay/depth"))
    assert sinks.events.index(("video", "engine/overlay")) == clear - 1    # right after the colour it belongs to
    # and the segmenter's boxes wait one more scene frame: a push beating the overlay's frames over to the UI
    # was resolved against the previous replay's overlay (phantom 3D markers from the wrist's depth)
    scene_after = [i for i, e in enumerate(sinks.events) if e == ("video", "camera/scene/color") and i > clear]
    assert sinks.events.index(("objects", True)) > scene_after[0]
    _, sinks, _, _ = play(build(tmp_path / "w"), overlay="wrist")         # the wrist's own depth is untouched
    assert all(f["bytes"] and f["metadata"] for f in sinks.frames if f["stream_id"] == "engine/overlay/depth")


def test_nothing_that_needs_a_camera_goes_out_before_its_colour_frame(tmp_path):
    # A window opening between frames began with frame 0's depth and objects but not its colour. RammsUI then
    # warned, once per stream and silent after: "No extrinsic for stream" for the push, and "not registered
    # to colour" for a depth frame meeting the colour (and intrinsics) a previous replay left on the overlay.
    session = BagSession(build(tmp_path), log=lambda *a: None)
    events = []

    class Ordered(RecordingSinks):
        def video(self, *a, **k):
            super().video(*a, **k)
            events.append(self.frames[-1]["stream_id"])

        def objects(self, payload):
            super().objects(payload)
            events.append("objects")
    fc = FakeClock()
    sinks = Ordered()
    stats = Player(session, sinks, PlayOptions(overlay="wrist", start_s=0.0000015), clock=fc.clock, sleep=fc.sleep,
                   log=lambda *a: None).run()
    assert events.index("camera/scene/color") < events.index("camera/scene/depth")
    assert events.index("camera/scene/color") < events.index("objects")
    assert events.index("engine/overlay") < events.index("engine/overlay/depth")
    # frame 0's scene depth and push held back (its colour is before the window); the wrist's own frame 0 is whole
    assert (stats.sent["scene_depth"], stats.sent["objects"], stats.sent["overlay_depth"]) == (5, 5, 6)


def test_a_colour_that_cannot_be_sent_holds_nothing_back(tmp_path):
    # Its depth and the objects wait for a camera's colour frame; one that can never go out must not hold them
    # for the whole run (it did: joints only).
    warnings = []
    session = BagSession(build(tmp_path, scene_encoding="yuv422"), log=lambda *a: None)
    sinks, fc = RecordingSinks(), FakeClock()
    stats = Player(session, sinks, PlayOptions(overlay="none"), clock=fc.clock, sleep=fc.sleep,
                   log=warnings.append).run()
    assert "scene_color" not in stats.sent
    assert (stats.sent["scene_depth"], stats.sent["objects"]) == (6, 6)
    assert any("yuv422 is not sent" in w and "go out without it" in w for w in warnings)


def test_a_topic_listed_without_messages_counts_as_absent(tmp_path, capsys):
    # A recorder lists every topic it subscribed to, published or not: a scene camera that never came up
    # crashed BagSession with a bare StopIteration.
    joint = {"header": hdr(T0, ""), "name": ["arm_joint_1"], "position": [0.1], "velocity": [0.0], "effort": [0.0]}
    path = write_bag(str(tmp_path / "bag"), {bp.SCENE_COLOR: "sensor_msgs/msg/Image",
                                             bp.SCENE_INFO: "sensor_msgs/msg/CameraInfo",
                                             bp.JOINTS: "sensor_msgs/msg/JointState"},
                     [(bp.JOINTS, T0, encode("sensor_msgs/msg/JointState", joint))])
    session = BagSession(path, log=lambda *a: None)
    assert session.scene_size is None and not session.has(bp.SCENE_COLOR) and session.has(bp.JOINTS)
    assert bp.SCENE_INFO not in session.info
    from ramms_tools.scripts.bag_cli import main
    assert main(["info", path]) == 0
    assert "scene camera: not in this bag" in capsys.readouterr().out


def test_segmentation_mode_sends_the_segmenters_boxes_to_the_overlay(tmp_path):
    _, sinks, _, _ = play(build(tmp_path), overlay="segmentation")
    push = sinks.pushes[-1]
    overlay = [o for o in push["objects"] if o["source"] == "engine/overlay"]
    scene = [o for o in push["objects"] if o["source"] == "camera/scene/color"]
    assert [o["instance_id"] for o in overlay] == ["seg:drink_cup_1", "seg:fridge_2"]
    assert all("position_m" not in o for o in overlay) and overlay[0]["box"] == pytest.approx([0, 0, 3 / W, 2 / H])
    assert [o["instance_id"] for o in scene] == ["drink_cup_7", "fridge_9"]   # one push, both cameras


def test_a_partial_joint_state_keeps_the_other_replayed_joints():
    client, rig = FakeClient(), FakeRig()
    sinks = LiveSinks(client, FakeSender(), rig, log=lambda *a: None)
    sinks.joints({"link_0": 45.0, "link_0_l": -10.0})
    sinks.drain()
    sinks.joints({"link_0": 50.0})               # a message naming only one joint
    sinks.drain()
    last = [c for c in client.calls if c[0] == "call_object"][-1][3]
    assert last == [7.0, 50.0, -10.0, 3.0]        # link_0_l keeps its replayed -10, not the original 2
    sinks.close()


def test_the_cli_puts_the_ui_back_when_playback_fails(tmp_path, monkeypatch, capsys):
    from ramms_tools.scripts import bag_cli
    import ramms_tools.scripts.ui_bridge_test as ubt
    import ramms_tools.streaming as streaming
    calls = []

    class Client(FakeClient):
        def __init__(self, *a, **k):
            super().__init__()

        def load_preset(self):
            pass

        def get_setting(self, *a, **k):
            return None

        def open_camera_feed(self):
            calls.append("open")

        def close_camera_feed(self):
            calls.append("close")

        def update_detected_objects(self, payload):
            if not payload["objects"]:
                calls.append("cleared")

    class Sender(FakeSender):
        def __init__(self, *a, **k):
            super().__init__()
            self.n = 0

        def connect(self, timeout=5.0):
            pass

        def send_image(self, *a, **k):
            self.n += 1
            if self.n > 3:
                raise ConnectionResetError("UE went away")

        def disconnect(self, **k):
            calls.append("disconnect")

    monkeypatch.setattr(ubt, "BridgeClient", Client)
    monkeypatch.setattr(ubt.JointRig, "locate", staticmethod(lambda c: (_ for _ in ()).throw(ubt.JointsUnavailable("no"))))
    monkeypatch.setattr(streaming, "StreamSender", Sender)
    with pytest.raises(ConnectionResetError):
        bag_cli.main(["play", build(tmp_path), "--rate", "0"])
    assert calls[0] == "open" and {"cleared", "close", "disconnect"} <= set(calls)
