"""Offline tests for ramms_tools.ros_ue, the ROS 2 -> Unreal conversions a bag replay needs.

The expected values come from somewhere other than the module under test: rotation matrices built
here with numpy, UE's own Euler convention as ramms_tools.transforms reproduces it, and numbers
measured on the recorded RAMMP bags (nov_19_perception_bag) and on the RAMMP UI bridge, which is
the robot's real path into UE. So an agreement means the conversion is right, not that the code
agrees with itself.

Tests that read the real bags are opt-in -- they need gigabytes of data no CI machine has:

    RAMMS_BAG_DIR=/path/to/nov_19_perception_bag RAMMS_BAG_VO_DIR=/path/to/nov_19_perception_bag_vo \\
        python -m pytest tests/test_ros_ue.py

Their reader (sqlite3 + a few hand-written CDR layouts) lives in this file on purpose: ros_ue works
on decoded dicts and must not grow a bag dependency, and the bag decoder is its own module.
"""
from __future__ import annotations

import json
import math
import os
import re
import sqlite3
import struct

import numpy as np
import pytest

from ramms_tools import transforms
from ramms_tools.ros_ue import (
    OPTICAL_TO_BODY,
    SightingTracker,
    TfBuffer,
    camera_extrinsic,
    detection_to_ue_object,
    display_joint_targets,
    normalize_box,
    optical_to_body,
    quat_to_euler_zyx_deg,
    ros_to_ue_transform,
)

# ── independent reference math ───────────────────────────────────────

def qmul(a, b):
    """Hamilton product, (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def axis_quat(axis, deg):
    h = math.radians(deg) / 2.0
    s = math.sin(h)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(h))


def euler_quat(roll, pitch, yaw):
    """Intrinsic Z-Y-X: yaw about z, then pitch about the new y, then roll about the new x."""
    return qmul(qmul(axis_quat((0, 0, 1), yaw), axis_quat((0, 1, 0), pitch)), axis_quat((1, 0, 0), roll))


def quat_matrix(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def euler_matrix(roll, pitch, yaw):
    r, p, y = (math.radians(a) for a in (roll, pitch, yaw))
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def homogeneous(t, q):
    m = np.eye(4)
    m[:3, :3] = quat_matrix(q)
    m[:3, 3] = t
    return m


def random_quats(n, seed=7):
    """Uniform random unit quaternions (normalised 4D Gaussians)."""
    q = np.random.default_rng(seed).normal(size=(n, 4))
    return [tuple(v) for v in q / np.linalg.norm(q, axis=1, keepdims=True)]


MIRROR_Y = np.diag([1.0, -1.0, 1.0])  # ROS y-left <-> UE Y-right


def tf_msg(*edges):
    """A decoded TFMessage from (parent, child, translation, rotation[, stamp_ns]) tuples."""
    out = []
    for e in edges:
        parent, child, t, q = e[:4]
        stamp = e[4] if len(e) > 4 else 0
        out.append({"header": {"stamp_ns": stamp, "frame_id": parent}, "child_frame_id": child,
                    "transform": {"translation": dict(zip("xyz", t)), "rotation": dict(zip("xyzw", q))}})
    return {"transforms": out}


IDENT = (0.0, 0.0, 0.0, 1.0)
# The camera_frame -> optical_frame edge every RealSense driver publishes (x right, y down, z fwd).
REALSENSE_OPTICAL = (-0.5, 0.5, -0.5, 0.5)

# /tf_static of nov_19_perception_bag for the scene camera, at the precision it was recorded with.
BAG_STATIC = [
    ("base_link", "arm_mount", (0.6017, 0.2817, 0.502), (0.0, 0.0, 0.0, 1.0), 1795126701436710188),
    ("arm_mount", "scene_camera", (0.151329, -0.571954, 0.391385),
     (0.0137645, 0.0170993, 0.2148372, 0.9764032), 1795126701436710188),
    ("scene_camera", "scene_camera_color_camera_frame", (0.0, 0.0, 0.0), IDENT, 1795126711560902421),
    ("scene_camera_color_camera_frame", "scene_camera_color_optical_frame", (0.0, 0.0, 0.0),
     REALSENSE_OPTICAL, 1795126711560902421),
]
# What the RAMMP UI bridge sends UE for that camera, and the same at full precision (the bridge's
# figures are its log's rounding of these).
BRIDGE_EXTRINSIC = {"x": 75.3, "y": 29.03, "z": 89.34, "roll": 1.97, "pitch": -1.57, "yaw": -24.84}
GOLDEN_EXTRINSIC = {"x": 75.30, "y": 29.03, "z": 89.34, "roll": 1.962, "pitch": -1.575, "yaw": -24.845}


def assert_transform(got, want, cm=0.005, deg=0.0005):
    assert set(got) == {"x", "y", "z", "roll", "pitch", "yaw"}
    for k in "xyz":
        assert got[k] == pytest.approx(want[k], abs=cm), k
    for k in ("roll", "pitch", "yaw"):
        assert got[k] == pytest.approx(want[k], abs=deg), k


def assert_same_rotation(q, want):
    assert np.allclose(quat_matrix(q), quat_matrix(want), atol=1e-9)


# ── quat_to_euler_zyx_deg ────────────────────────────────────────────

@pytest.mark.parametrize("angles", [(30.0, 0.0, 0.0), (0.0, 30.0, 0.0), (0.0, 0.0, 30.0), (0.0, 0.0, -170.0),
                                    (12.0, -40.0, 135.0)])
def test_euler_recovers_the_angles_a_quaternion_was_built_from(angles):
    assert quat_to_euler_zyx_deg(*euler_quat(*angles)) == pytest.approx(angles, abs=1e-9)


def test_euler_rebuilds_the_same_rotation():
    for q in random_quats(500):
        assert np.allclose(euler_matrix(*quat_to_euler_zyx_deg(*q)), quat_matrix(q), atol=1e-9)


def test_euler_is_ue_rotator_of_the_y_mirrored_quaternion():
    # transforms.quat_to_euler is UE's FQuat::Rotator. Mirroring Y maps a ROS quaternion to
    # (-x, y, -z, w) in UE; its rotator must be the ROS angles with pitch and yaw negated -- the
    # rule ros_to_ue_transform applies.
    for x, y, z, w in random_quats(500, seed=11):
        roll, pitch, yaw = quat_to_euler_zyx_deg(x, y, z, w)
        ue = transforms.quat_to_euler(-x, y, -z, w)
        assert (roll, -pitch, -yaw) == pytest.approx((ue["roll"], ue["pitch"], ue["yaw"]), abs=1e-7)


@pytest.mark.parametrize("pitch, yaw_out", [(90.0, 30.0), (-90.0, 50.0)], ids=["nose-down", "nose-up"])
def test_gimbal_lock_keeps_the_rotation_for_both_signs(pitch, yaw_out):
    # roll 10, yaw 40: at pitch +90 only yaw - roll survives, at -90 only yaw + roll. Roll folds into
    # yaw and the PITCH SIGN IS KEPT -- the bridge's Python node returns -90 for +90 here, which is a
    # different rotation (its C++ node gets it right).
    q = euler_quat(10.0, pitch, 40.0)
    roll, got_pitch, yaw = quat_to_euler_zyx_deg(*q)
    assert (roll, got_pitch) == (0.0, pitch)
    assert yaw == pytest.approx(yaw_out, abs=1e-6)
    assert np.allclose(euler_matrix(roll, got_pitch, yaw), quat_matrix(q), atol=1e-7)


def test_gimbal_lock_wrong_sign_would_be_a_different_rotation():
    # Proves the test above can fail: the Python bridge's answer does not rebuild the rotation.
    q = euler_quat(0.0, 90.0, 40.0)
    assert not np.allclose(euler_matrix(0.0, -90.0, 40.0), quat_matrix(q), atol=1e-3)


def test_just_outside_gimbal_lock_is_still_exact():
    q = euler_quat(5.0, 89.9, -20.0)
    assert quat_to_euler_zyx_deg(*q) == pytest.approx((5.0, 89.9, -20.0), abs=1e-6)


def test_euler_ignores_quaternion_sign_and_scale():
    q = euler_quat(12.0, -40.0, 135.0)
    want = quat_to_euler_zyx_deg(*q)
    assert quat_to_euler_zyx_deg(*(-c for c in q)) == pytest.approx(want, abs=1e-9)
    assert quat_to_euler_zyx_deg(*(3.0 * c for c in q)) == pytest.approx(want, abs=1e-9)


def test_euler_rejects_a_zero_quaternion():
    with pytest.raises(ValueError):
        quat_to_euler_zyx_deg(0.0, 0.0, 0.0, 0.0)


# ── ros_to_ue_transform ──────────────────────────────────────────────

def test_ros_to_ue_transform_is_centimetres_with_y_mirrored():
    assert ros_to_ue_transform((1.0, 2.0, 3.0), IDENT) == {
        "x": 100.0, "y": -200.0, "z": 300.0, "roll": 0.0, "pitch": -0.0, "yaw": -0.0}


def test_ros_to_ue_transform_accepts_dicts_and_tuples():
    t, q = (0.1, -0.2, 0.3), euler_quat(5.0, 10.0, 15.0)
    assert ros_to_ue_transform(dict(zip("xyz", t)), dict(zip("xyzw", q))) == ros_to_ue_transform(t, q)
    assert ros_to_ue_transform(list(t), np.array(q)) == ros_to_ue_transform(t, q)


@pytest.mark.parametrize("q, ue", [
    (axis_quat((0, 0, 1), 30.0), {"roll": 0.0, "pitch": 0.0, "yaw": -30.0}),   # turn left = UE yaw -30
    (axis_quat((0, 1, 0), 20.0), {"roll": 0.0, "pitch": -20.0, "yaw": 0.0}),   # nose down = UE pitch -20
    (axis_quat((1, 0, 0), 15.0), {"roll": 15.0, "pitch": 0.0, "yaw": 0.0}),    # right side down = UE roll +15
], ids=["yaw", "pitch", "roll"])
def test_ros_to_ue_single_axis_signs(q, ue):
    got = ros_to_ue_transform((0, 0, 0), q)
    assert {k: got[k] for k in ue} == pytest.approx(ue, abs=1e-9)


def test_ue_rotation_is_the_y_mirror_of_the_ros_one():
    # UE's own matrix builder, fed the converted angles, must give M R M (M mirrors Y): the same
    # physical rotation seen in UE's left-handed axes.
    for q in random_quats(300, seed=3):
        ue = ros_to_ue_transform((0, 0, 0), q)
        r_ue = np.array(transforms.rotation_matrix_from_euler(ue["roll"], ue["pitch"], ue["yaw"]))
        assert np.allclose(r_ue, MIRROR_Y @ quat_matrix(q) @ MIRROR_Y, atol=1e-9)


# ── optical_to_body ──────────────────────────────────────────────────

def test_optical_to_body_constant_is_the_body_axes_in_optical_coordinates():
    m = quat_matrix(OPTICAL_TO_BODY)
    assert np.allclose(m @ [1, 0, 0], [0, 0, 1])    # body forward = optical +z (the view direction)
    assert np.allclose(m @ [0, 1, 0], [-1, 0, 0])   # body left    = optical -x (x is right)
    assert np.allclose(m @ [0, 0, 1], [0, -1, 0])   # body up      = optical -y (y is down)


def test_optical_to_body_undoes_the_realsense_optical_edge():
    assert_same_rotation(optical_to_body(REALSENSE_OPTICAL), IDENT)
    assert_same_rotation(optical_to_body(dict(zip("xyzw", REALSENSE_OPTICAL))), IDENT)


def test_body_frame_looks_where_the_optical_frame_looks():
    for q in random_quats(100, seed=5):
        body = optical_to_body(q)
        assert np.allclose(quat_matrix(body) @ [1, 0, 0], quat_matrix(q) @ [0, 0, 1], atol=1e-12)
        assert np.allclose(quat_matrix(body) @ [0, 0, 1], quat_matrix(q) @ [0, -1, 0], atol=1e-12)


# ── TfBuffer ─────────────────────────────────────────────────────────

def apply(pose, p):
    t, q = pose
    return quat_matrix(q) @ np.asarray(p, dtype=float) + np.asarray(t)


def test_lookup_maps_child_coordinates_into_the_parent():
    tf = TfBuffer()
    tf.add(tf_msg(("a", "b", (1.0, 0.0, 0.0), axis_quat((0, 0, 1), 90.0))), True)
    pose = tf.lookup("a", "b")
    assert np.allclose(pose[0], (1, 0, 0))
    assert np.allclose(apply(pose, (1, 0, 0)), (1, 1, 0))   # b's x axis is a's y after the 90 deg yaw


def test_lookup_inverts_an_edge_looked_up_backwards():
    tf = TfBuffer()
    tf.add(tf_msg(("a", "b", (1.0, 2.0, 3.0), euler_quat(10, 20, 30))), True)
    fwd, back = tf.lookup("a", "b"), tf.lookup("b", "a")
    p = (0.3, -0.7, 1.1)
    assert np.allclose(apply(back, apply(fwd, p)), p)


def test_lookup_composes_a_chain():
    tf = TfBuffer()
    tf.add(tf_msg(("a", "b", (1.0, 0.0, 0.0), axis_quat((0, 0, 1), 90.0)),
                  ("b", "c", (1.0, 0.0, 0.0), IDENT)), True)
    pose = tf.lookup("a", "c")
    assert np.allclose(pose[0], (1, 1, 0))
    assert_same_rotation(pose[1], axis_quat((0, 0, 1), 90.0))


def test_lookup_through_a_common_ancestor():
    tf = TfBuffer()
    tf.add(tf_msg(("root", "left", (0.0, 1.0, 0.0), IDENT),
                  ("root", "right", (0.0, -1.0, 0.0), axis_quat((0, 0, 1), 180.0))), True)
    pose = tf.lookup("left", "right")
    assert np.allclose(pose[0], (0, -2, 0))
    assert_same_rotation(pose[1], axis_quat((0, 0, 1), 180.0))


def test_lookup_agrees_with_matrices_on_a_random_tree():
    rng = np.random.default_rng(21)
    names = ["f0"]
    edges, world = [], {"f0": np.eye(4)}
    for i, q in enumerate(random_quats(12, seed=22), start=1):
        parent = names[int(rng.integers(len(names)))]
        t = tuple(rng.normal(size=3))
        edges.append((parent, f"f{i}", t, q))
        world[f"f{i}"] = world[parent] @ homogeneous(t, q)
        names.append(f"f{i}")
    tf = TfBuffer()
    tf.add(tf_msg(*edges), True)
    for a in names:
        for b in names:
            want = np.linalg.inv(world[a]) @ world[b]
            t, q = tf.lookup(a, b)
            assert np.allclose(homogeneous(t, q), want, atol=1e-9), (a, b)


def test_same_frame_is_identity():
    tf = TfBuffer()
    tf.add(tf_msg(("a", "b", (1.0, 2.0, 3.0), IDENT)), True)
    assert tf.lookup("b", "b") == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_frames_lists_parents_and_children():
    tf = TfBuffer()
    tf.add(tf_msg(("world", "odom", (0, 0, 0), IDENT)), True)
    tf.add(tf_msg(("odom", "base_link", (0, 0, 0), IDENT, 5)), False)
    assert tf.frames() == {"world", "odom", "base_link"}


def test_disconnected_frames_raise_naming_both():
    tf = TfBuffer()
    tf.add(tf_msg(("world", "odom", (0, 0, 0), IDENT), ("base_link", "arm_mount", (0, 0, 0), IDENT)), True)
    with pytest.raises(LookupError, match=r"'odom'.*'arm_mount'"):
        tf.lookup("odom", "arm_mount")
    with pytest.raises(LookupError, match=r"'nowhere'"):
        tf.lookup("base_link", "nowhere")


def yaw_of(q):
    return quat_to_euler_zyx_deg(*q)[2]


def dynamic_tf(*samples, parent="odom", child="base_link"):
    """One dynamic edge from (stamp_ns, x, yaw_deg) samples."""
    tf = TfBuffer()
    for stamp, x, yaw in samples:
        tf.add(tf_msg((parent, child, (x, 0.0, 0.0), axis_quat((0, 0, 1), yaw), stamp)), False)
    return tf


def test_dynamic_edge_interpolates_between_bracketing_samples():
    tf = dynamic_tf((1_000_000_000, 0.0, 0.0), (2_000_000_000, 1.0, 90.0), (3_000_000_000, 5.0, 90.0))
    t, q = tf.lookup("odom", "base_link", 1_250_000_000)
    assert t == pytest.approx((0.25, 0.0, 0.0))
    assert yaw_of(q) == pytest.approx(22.5)            # slerp: a constant angular rate
    t, q = tf.lookup("odom", "base_link", 2_000_000_000)
    assert t == pytest.approx((1.0, 0.0, 0.0)) and yaw_of(q) == pytest.approx(90.0)


def test_dynamic_edge_clamps_outside_its_samples():
    tf = dynamic_tf((1_000_000_000, 0.0, 0.0), (2_000_000_000, 1.0, 90.0))
    assert tf.lookup("odom", "base_link", 0)[0] == pytest.approx((0, 0, 0))
    assert tf.lookup("odom", "base_link", 9_000_000_000)[0] == pytest.approx((1, 0, 0))


def test_none_takes_the_newest_sample_even_when_added_out_of_order():
    tf = dynamic_tf((3_000_000_000, 3.0, 30.0), (1_000_000_000, 1.0, 10.0), (2_000_000_000, 2.0, 20.0))
    t, q = tf.lookup("odom", "base_link")
    assert t == pytest.approx((3.0, 0.0, 0.0)) and yaw_of(q) == pytest.approx(30.0)
    assert tf.lookup("odom", "base_link", 1_500_000_000)[0] == pytest.approx((1.5, 0, 0))


def test_slerp_takes_the_short_way_across_a_quaternion_sign_flip():
    tf = TfBuffer()
    tf.add(tf_msg(("odom", "base_link", (0, 0, 0), axis_quat((0, 0, 1), 10.0), 0)), False)
    tf.add(tf_msg(("odom", "base_link", (0, 0, 0), tuple(-c for c in axis_quat((0, 0, 1), 20.0)), 10)), False)
    assert yaw_of(tf.lookup("odom", "base_link", 5)[1]) == pytest.approx(15.0)


def test_static_edges_ignore_time():
    # /tf_static in a bag is stamped when the robot booted, hours before the recording.
    tf = TfBuffer()
    tf.add(tf_msg(("base_link", "cam", (1.0, 0.0, 0.0), IDENT, 5)), True)
    tf.add(tf_msg(("odom", "base_link", (0.0, 0.0, 0.0), IDENT, 10_000_000_000),
                  ("odom", "base_link", (2.0, 0.0, 0.0), IDENT, 12_000_000_000)), False)
    assert tf.lookup("odom", "cam", 11_000_000_000)[0] == pytest.approx((2.0, 0.0, 0.0))
    assert tf.lookup("base_link", "cam", 10**18)[0] == pytest.approx((1.0, 0.0, 0.0))


def test_reparenting_a_child_replaces_its_edge():
    tf = TfBuffer()
    tf.add(tf_msg(("a", "c", (1, 0, 0), IDENT)), True)
    tf.add(tf_msg(("b", "c", (2, 0, 0), IDENT)), True)
    assert tf.lookup("b", "c")[0] == pytest.approx((2, 0, 0))
    with pytest.raises(LookupError):
        tf.lookup("a", "c")


def test_a_bag_sized_tree_stays_exact():
    # 13 dynamic edges x 5000 samples, like the arm in /tf: a chain looked up at stamps between
    # samples must equal the matrices of the interpolated edges composed by hand.
    n, edges = 5000, 13
    stamps = np.arange(n, dtype=np.int64) * 20_000_000
    rng = np.random.default_rng(1)
    angles = np.cumsum(rng.normal(scale=0.5, size=(edges, n)), axis=1)     # degrees, smooth-ish
    tf = TfBuffer()
    for i in range(n):
        tf.add(tf_msg(*[(f"j{e}", f"j{e + 1}", (0.1, 0.0, 0.05), axis_quat((0, 0, 1), angles[e, i]),
                         int(stamps[i])) for e in range(edges)]), False)
    for k in rng.integers(0, n - 1, size=200):
        a = float(rng.random())
        stamp = int(stamps[k]) + int(a * 20_000_000)
        a = (stamp - int(stamps[k])) / 20_000_000
        want = np.eye(4)
        for e in range(edges):
            yaw = angles[e, k] + a * (angles[e, k + 1] - angles[e, k])   # all about z: slerp = linear angle
            want = want @ homogeneous((0.1, 0.0, 0.05), axis_quat((0, 0, 1), yaw))
        t, q = tf.lookup("j0", f"j{edges}", stamp)
        assert np.allclose(homogeneous(t, q), want, atol=1e-9)


# ── camera_extrinsic ─────────────────────────────────────────────────

def test_camera_extrinsic_of_a_body_turned_left():
    tf = TfBuffer()
    tf.add(tf_msg(("base_link", "cam", (1.0, 0.0, 0.5), axis_quat((0, 0, 1), 90.0)),
                  ("cam", "cam_optical", (0.0, 0.0, 0.0), REALSENSE_OPTICAL)), True)
    assert_transform(camera_extrinsic(tf, "base_link", "cam_optical"),
                     {"x": 100.0, "y": 0.0, "z": 50.0, "roll": 0.0, "pitch": 0.0, "yaw": -90.0}, cm=1e-9, deg=1e-9)


def test_camera_extrinsic_golden_from_the_bag_static_tf():
    tf = TfBuffer()
    tf.add(tf_msg(*BAG_STATIC), True)
    got = camera_extrinsic(tf, "base_link", "scene_camera_color_optical_frame")
    assert_transform(got, GOLDEN_EXTRINSIC)
    assert_transform(got, BRIDGE_EXTRINSIC, cm=0.005, deg=0.01)   # what the robot sends UE
    # From the optical frame or from the body link, the camera is in the same place.
    assert_transform(got, ros_to_ue_transform(*tf.lookup("base_link", "scene_camera")), cm=1e-9, deg=1e-9)


def test_camera_extrinsic_follows_a_dynamic_mount():
    tf = TfBuffer()
    tf.add(tf_msg(("link", "cam", (0.0, 0.0, 0.0), IDENT), ("cam", "cam_optical", (0, 0, 0), REALSENSE_OPTICAL)), True)
    tf.add(tf_msg(("base_link", "link", (0.0, 0.0, 1.0), axis_quat((0, 0, 1), 0.0), 0),
                  ("base_link", "link", (0.0, 0.0, 1.0), axis_quat((0, 0, 1), 40.0), 100)), False)
    assert camera_extrinsic(tf, "base_link", "cam_optical", 50)["yaw"] == pytest.approx(-20.0)


# ── display_joint_targets ────────────────────────────────────────────

ARM = [f"arm_joint_{n}" for n in range(1, 8)]
GRIP = "arm_robotiq_85_left_knuckle_joint"


def test_arm_joints_go_to_links_by_name_in_degrees_with_display_signs():
    got = display_joint_targets(ARM, [math.radians(10.0 * n) for n in range(1, 8)])
    assert got == pytest.approx({"link_0": -10.0, "link_1": 20.0, "link_2": -30.0, "link_3": 40.0,
                                 "link_4": -50.0, "link_5": -60.0, "link_6": -70.0})


@pytest.mark.parametrize("q, closed, fingers", [
    (0.0, 0.793, (-45.0, 45.0)),       # open
    (0.3965, 0.793, (-22.5, 22.5)),    # half
    (0.793, 0.793, (0.0, 0.0)),        # this robot's fully closed knuckle
    (0.95, 0.793, (0.0, 0.0)),         # past closed: clamped
    (-0.05, 0.793, (-45.0, 45.0)),     # past open: clamped
    (0.4, 0.8, (-22.5, 22.5)),         # a different gripper's travel
])
def test_gripper_fingers_follow_the_knuckle_in_proportion(q, closed, fingers):
    got = display_joint_targets([GRIP], [q], gripper_closed_rad=closed)
    assert (got["link_0_l"], got["link_0_r"]) == pytest.approx(fingers)


def test_closed_gripper_sends_plain_zero():
    got = display_joint_targets([GRIP], [0.793])
    assert math.copysign(1.0, got["link_0_l"]) == 1.0 and json.dumps(got) == '{"link_0_l": 0.0, "link_0_r": 0.0}'


def test_bag_message_maps_every_display_joint():
    # The first /joint_states of nov_19_perception_bag: the knuckle is at its 0.793 rad maximum.
    positions = [0.37344, -0.76451, -1.60918, -1.93069, -1.02586, -1.63715, -2.89204, 0.79301]
    got = display_joint_targets(ARM + [GRIP], positions)
    assert set(got) == {f"link_{i}" for i in range(7)} | {"link_0_l", "link_0_r"}
    assert got["link_0"] == pytest.approx(-math.degrees(0.37344))
    assert got["link_3"] == pytest.approx(math.degrees(-1.93069))
    assert (got["link_0_l"], got["link_0_r"]) == (0.0, 0.0)


def test_absent_unknown_and_unmeasured_joints_are_left_out():
    got = display_joint_targets(["arm_joint_2", "wheel_left", GRIP, "arm_joint_5"],
                                [0.5, 1.0, float("nan"), float("nan")])
    assert got == pytest.approx({"link_1": math.degrees(0.5)})


def test_joint_message_shape_errors():
    assert display_joint_targets(ARM, []) == {}           # velocity-only JointState
    with pytest.raises(ValueError):
        display_joint_targets(ARM, [0.0, 0.0])
    with pytest.raises(ValueError):
        display_joint_targets([GRIP], [0.1], gripper_closed_rad=0.0)


# ── detections ───────────────────────────────────────────────────────

# A /perception/object_detections object from nov_19_perception_bag (decoded; float32 box).
FRIDGE = {"instance_id": "fridge_14488", "perception_class": "fridge", "score": 0.495, "state": 0,
          "box": [1.09, 39.5, 145.23, 315.79], "mask_id": 0, "has_pose": True,
          "pose": {"position": {"x": -0.2116, "y": -0.0086, "z": 0.6538},
                   "orientation": {"x": 0.291, "y": 0.655, "z": -0.647, "w": 0.261}},
          "size": {"x": 0.345, "y": 0.216, "z": 0.632}}


def test_normalize_box():
    assert normalize_box([64, 40, 320, 200], 640, 400) == [0.1, 0.1, 0.5, 0.5]


def test_normalize_box_clamps_an_overhang():
    # RammsUI takes a box with any value > 1 as PIXELS: an unclamped 1.0005 would shrink it 640-fold.
    assert normalize_box([-0.4, 10, 640.3, 400.2], 640, 400) == [0.0, 0.025, 1.0, 1.0]


def test_normalize_box_rejects_an_empty_image():
    with pytest.raises(ValueError):
        normalize_box([0, 0, 1, 1], 0, 400)


def test_visible_object_with_3d():
    got = detection_to_ue_object(FRIDGE, 640, 400, "camera/scene/color", time_since_s=0.0666666666)
    assert got == {
        "instance_id": "fridge_14488", "perception_class": "fridge", "source": "camera/scene/color",
        "box": [0.001703, 0.09875, 0.226922, 0.789475], "centroid": [0.114312, 0.444113],
        "position_m": {"X": -0.2116, "Y": -0.0086, "Z": 0.6538}, "frame": "camera",
        "orientation": {"X": 0.291, "Y": 0.655, "Z": -0.647, "W": 0.261},
        "size_m": {"X": 0.345, "Y": 0.216, "Z": 0.632},
        "time_since_detection_s": 0.066667,
    }
    json.dumps(got, allow_nan=False)


def test_occluded_object_has_a_zero_box_and_no_centroid():
    occluded = dict(FRIDGE, state=1, box=[0.0, 0.0, 0.0, 0.0])
    got = detection_to_ue_object(occluded, 640, 400, "camera/scene/color", time_since_s=3.2)
    assert got["box"] == [0.0, 0.0, 0.0, 0.0] and "centroid" not in got
    assert got["position_m"] == {"X": -0.2116, "Y": -0.0086, "Z": 0.6538}   # the tracker still knows where
    assert got["time_since_detection_s"] == 3.2


def test_occluded_state_wins_over_a_stale_box():
    got = detection_to_ue_object(dict(FRIDGE, state=1), 640, 400, "camera/scene/color")
    assert got["box"] == [0.0, 0.0, 0.0, 0.0] and "centroid" not in got


def test_2d_only_objects():
    want_keys = {"instance_id", "perception_class", "source", "box", "centroid"}
    assert set(detection_to_ue_object(FRIDGE, 640, 400, "s", with_3d=False)) == want_keys
    assert set(detection_to_ue_object(dict(FRIDGE, has_pose=False), 640, 400, "s")) == want_keys


def test_time_since_is_omitted_when_unknown_and_kept_when_zero():
    assert "time_since_detection_s" not in detection_to_ue_object(FRIDGE, 640, 400, "s")
    assert detection_to_ue_object(FRIDGE, 640, 400, "s", time_since_s=0.0)["time_since_detection_s"] == 0.0


# ── SightingTracker ──────────────────────────────────────────────────

MS = 1_000_000


def det(iid, cls, box, state=0):
    return {"instance_id": iid, "perception_class": cls, "state": state, "box": list(box)}


CHAIR = (100.0, 100.0, 200.0, 300.0)


def test_tracker_frames_between_raw_frames_are_not_sightings():
    # 15 Hz tracker, 5 Hz segmenter: the tracker reports the chair "visible" every frame, but only
    # every third frame was a direct sighting -- the producer's 0.00 / 0.07 / 0.13 s pattern.
    tracked = [(i * 66_666_667, [det("chair_7", "chair", CHAIR)]) for i in range(7)]
    raw = [(tracked[i][0], [det("chair_1", "chair", (102, 98, 205, 301))]) for i in (0, 3, 6)]
    s = SightingTracker(raw, tracked)
    got = [s.time_since(stamp, "chair_7") for stamp, _ in tracked]
    assert got == pytest.approx([0.0, 0.066666667, 0.133333334, 0.0, 0.066666667, 0.133333334, 0.0])


def test_tracker_needs_overlap_whatever_the_segmenter_called_it():
    t = 100 * MS
    tracked = [(t, [det("chair_7", "chair", CHAIR), det("cup_2", "cup", (10, 10, 20, 20)),
                    det("tv_1", "tv", (0, 0, 10, 10))])]
    raw = [(t, [det("x_1", "table", CHAIR),                 # same box, another class: still a sighting
                det("x_2", "cup", (300, 300, 320, 320)),    # same class, elsewhere: not one
                det("x_3", "tv", (0, 0, 10, 3))])]          # IoU exactly 0.3: must EXCEED
    s = SightingTracker(raw, tracked)
    # the segmenter labels one mask "food container" one frame and "drink cup" the next
    assert [s.time_since(t, i) for i in ("chair_7", "cup_2", "tv_1")] == [0.0, None, None]
    assert SightingTracker(raw, tracked, iou=0.29).time_since(t, "tv_1") == 0.0


def test_tracker_stamp_tolerance():
    t = 1_795_133_217_000_000_000
    tracked = [(t, [det("chair_7", "chair", CHAIR)])]
    assert SightingTracker([(t + 900, [det("c", "chair", CHAIR)])], tracked).time_since(t, "chair_7") == 0.0
    assert SightingTracker([(t - 2000, [det("c", "chair", CHAIR)])], tracked).time_since(t, "chair_7") is None
    assert SightingTracker([(t - 2000, [det("c", "chair", CHAIR)])], tracked,
                           stamp_tolerance_ns=5000).time_since(t, "chair_7") == 0.0


def test_tracker_occluded_objects_are_never_sighted():
    t = 100 * MS
    tracked = [(t, [det("chair_7", "chair", (0, 0, 0, 0), state=1)])]
    raw = [(t, [det("c", "chair", (0, 0, 0, 0)), det("c2", "chair", CHAIR)])]
    assert SightingTracker(raw, tracked).time_since(t, "chair_7") is None


def test_time_since_is_a_pure_function_of_time():
    tracked = [(t * MS, [det("chair_7", "chair", CHAIR)]) for t in (100, 200, 300, 400)]
    raw = [(t * MS, [det("c", "chair", CHAIR)]) for t in (200, 400)]
    s = SightingTracker(list(reversed(raw)), list(reversed(tracked)))  # input order does not matter
    assert s.time_since(150 * MS, "chair_7") is None                    # before the first sighting
    assert s.time_since(390 * MS, "chair_7") == pytest.approx(0.19)
    assert s.time_since(250 * MS, "chair_7") == pytest.approx(0.05)     # seek backwards
    assert s.time_since(5_000 * MS, "chair_7") == pytest.approx(4.6)    # long after: still counting
    assert s.time_since(400 * MS, "nobody") is None


# ── opt-in: the recorded bags ────────────────────────────────────────

BAG_DIR = os.environ.get("RAMMS_BAG_DIR")
VO_DIR = os.environ.get("RAMMS_BAG_VO_DIR")
needs_bag = pytest.mark.skipif(not BAG_DIR, reason="set RAMMS_BAG_DIR to nov_19_perception_bag/")
needs_vo = pytest.mark.skipif(not VO_DIR, reason="set RAMMS_BAG_VO_DIR to nov_19_perception_bag_vo/")


class Cdr:
    """Just enough little-endian CDR for the topics read below (Humble, rosbag2 sqlite)."""

    def __init__(self, data):
        if bytes(data[:2]) != b"\x00\x01":
            raise ValueError("not CDR little-endian")
        self.b, self.p = memoryview(data), 4

    def _prim(self, fmt, size):
        self.p += -(self.p - 4) % size          # align relative to the end of the encapsulation
        v = struct.unpack_from("<" + fmt, self.b, self.p)[0]
        self.p += size
        return v

    def u8(self):
        self.p += 1
        return self.b[self.p - 1]

    def u16(self):
        return self._prim("H", 2)

    def u32(self):
        return self._prim("I", 4)

    def i32(self):
        return self._prim("i", 4)

    def f32(self):
        return self._prim("f", 4)

    def f64(self):
        return self._prim("d", 8)

    def string(self):
        n = self.u32()
        self.p += n
        return bytes(self.b[self.p - n:self.p - 1]).decode()

    def seq(self, fn):
        return [fn() for _ in range(self.u32())]

    def header(self):
        sec, nsec = self.i32(), self.u32()
        return {"stamp_ns": sec * 1_000_000_000 + nsec, "frame_id": self.string()}

    def xyz(self):
        return {"x": self.f64(), "y": self.f64(), "z": self.f64()}

    def xyzw(self):
        return {"x": self.f64(), "y": self.f64(), "z": self.f64(), "w": self.f64()}


def decode_tf(data):
    r = Cdr(data)

    def stamped():
        h = r.header()
        return {"header": h, "child_frame_id": r.string(),
                "transform": {"translation": r.xyz(), "rotation": r.xyzw()}}
    return {"transforms": r.seq(stamped)}


def decode_joint_state(data):
    r = Cdr(data)
    r.header()
    return r.seq(r.string), r.seq(r.f64)


def decode_detections(data):
    """ObjectDetections without its label image: (header stamp, objects)."""
    r = Cdr(data)
    stamp = r.header()["stamp_ns"]

    def obj():
        return {"instance_id": r.string(), "perception_class": r.string(), "score": r.f32(), "state": r.u8(),
                "box": [r.f32() for _ in range(4)], "mask_id": r.u16(), "has_pose": bool(r.u8()),
                "pose": {"position": r.xyz(), "orientation": r.xyzw()}, "size": r.xyz()}
    return stamp, r.seq(obj)


def decode_string(data):
    return Cdr(data).string()


def bag_messages(bag_dir, topic):
    """(bag timestamp, raw CDR) of one topic, in time order across a sequential split bag."""
    files = sorted((f for f in os.listdir(bag_dir) if f.endswith(".db3")),
                   key=lambda f: int(re.search(r"_(\d+)\.db3$", f).group(1)))
    assert files, f"no .db3 files in {bag_dir}"
    for name in files:
        # immutable=1: read-only and never writes a -journal/-wal next to the recording.
        con = sqlite3.connect(f"file:{os.path.join(bag_dir, name)}?mode=ro&immutable=1", uri=True)
        try:
            row = con.execute("select id from topics where name = ?", (topic,)).fetchone()
            if row is not None:
                yield from con.execute("select timestamp, data from messages where topic_id = ? "
                                       "order by timestamp", (row[0],))
        finally:
            con.close()


def bag_tf(bag_dir, dynamic=True):
    tf, dyn = TfBuffer(), []
    for _, data in bag_messages(bag_dir, "/tf_static"):
        tf.add(decode_tf(data), True)
    if dynamic:
        for _, data in bag_messages(bag_dir, "/tf"):
            msg = decode_tf(data)
            tf.add(msg, False)
            dyn.append(msg)
    return tf, dyn


@needs_bag
def test_bag_scene_camera_extrinsic_matches_the_bridge():
    tf, _ = bag_tf(BAG_DIR, dynamic=False)
    got = camera_extrinsic(tf, "base_link", "scene_camera_color_optical_frame")
    assert_transform(got, GOLDEN_EXTRINSIC)
    assert_transform(got, BRIDGE_EXTRINSIC, cm=0.005, deg=0.01)


@needs_bag
def test_bag_tf_chain_matches_the_recorded_edges():
    # At a recorded /tf stamp every arm edge is an exact sample, so base_link -> wrist optical frame
    # must equal that message's edges and the static ones multiplied as matrices.
    tf, dyn = bag_tf(BAG_DIR)
    static = {}
    for _, data in bag_messages(BAG_DIR, "/tf_static"):
        for e in decode_tf(data)["transforms"]:
            static[e["child_frame_id"]] = e
    assert len(dyn) > 5000 and len({e["child_frame_id"] for e in dyn[0]["transforms"]}) == 13
    for msg in (dyn[0], dyn[len(dyn) // 2], dyn[-1]):
        edges = dict(static, **{e["child_frame_id"]: e for e in msg["transforms"]})
        stamp = msg["transforms"][0]["header"]["stamp_ns"]
        frame, want = "wrist_camera_color_optical_frame", np.eye(4)
        while frame != "base_link":
            e = edges[frame]
            tr, ro = e["transform"]["translation"], e["transform"]["rotation"]
            want = homogeneous([tr[k] for k in "xyz"], [ro[k] for k in "xyzw"]) @ want
            frame = e["header"]["frame_id"]
        t, q = tf.lookup("base_link", "wrist_camera_color_optical_frame", stamp)
        assert np.allclose(homogeneous(t, q), want, atol=1e-9)
    # The static scene camera does not move with the arm, and this bag has no odom -> base_link.
    assert_transform(camera_extrinsic(tf, "base_link", "scene_camera_color_optical_frame", stamp), GOLDEN_EXTRINSIC)
    with pytest.raises(LookupError, match="'odom'.*'base_link'"):
        tf.lookup("odom", "base_link")


@needs_bag
def test_bag_joint_states_cover_the_display():
    n, fingers = 0, []
    for _, data in bag_messages(BAG_DIR, "/joint_states"):
        names, positions = decode_joint_state(data)
        got = display_joint_targets(names, positions)
        assert set(got) == {f"link_{i}" for i in range(7)} | {"link_0_l", "link_0_r"}
        fingers.append(got["link_0_r"])
        n += 1
    assert n == 5045
    # The knuckle runs the whole travel in this bag: the display must show it open AND closed
    # (the bridge's > 0.9 rad rule shows it open throughout).
    assert min(fingers) == 0.0 and max(fingers) == pytest.approx(45.0)


@needs_bag
def test_bag_detections_convert_to_valid_ui_objects():
    n_tracked = n_occluded = 0
    for topic in ("/perception/object_detections", "/perception/object_detections_raw"):
        for _, data in bag_messages(BAG_DIR, topic):
            _, objects = decode_detections(data)
            for obj in objects:
                got = detection_to_ue_object(obj, 640, 400, "camera/scene/color")
                json.dumps(got, allow_nan=False)
                zero_box = not any(obj["box"])
                assert (obj["state"] == 1) == zero_box        # the bag's own rule, which ros_ue relies on
                assert all(0.0 <= v <= 1.0 for v in got["box"])
                assert ("centroid" in got) == (not zero_box)
                assert ("position_m" in got) == obj["has_pose"]
                if topic.endswith("detections"):
                    n_tracked += 1
                    n_occluded += zero_box
    assert n_tracked > 1000 and 0 < n_occluded < n_tracked


@needs_vo
def test_vo_sightings_match_the_producers_time_since_detection():
    # The vo bag carries the producer's own UpdateDetectedObjects JSON (/perception/odom3d/ui_payload),
    # written at the same bag timestamp as the tracked message it was built from.
    raw = [decode_detections(d) for _, d in bag_messages(VO_DIR, "/perception/object_detections_raw")]
    tracked_by_bag_ts = {ts: decode_detections(d) for ts, d in bag_messages(VO_DIR, "/perception/object_detections")}
    tracker = SightingTracker(raw, list(tracked_by_bag_ts.values()))
    errors, unknown, unpaired = [], 0, 0
    for ts, data in bag_messages(VO_DIR, "/perception/odom3d/ui_payload"):
        if ts not in tracked_by_bag_ts:
            unpaired += 1
            continue
        stamp = tracked_by_bag_ts[ts][0]
        for o in json.loads(decode_string(data))["objects"]:
            if "time_since_detection_s" not in o:
                continue
            ours = tracker.time_since(stamp, o["instance_id"])
            if ours is None:
                unknown += 1
            else:
                errors.append(abs(ours - o["time_since_detection_s"]))
    total = len(errors) + unknown
    within = sum(e < 0.04 for e in errors)
    print(f"\nvo time_since: {total} objects, {within / total:.1%} within 0.04 s, {unknown} never sighted, "
          f"{unpaired} payloads unpaired, p50 {np.median(errors):.3f} s")
    assert unpaired == 0 and total > 5000
    assert within / total >= 0.85      # an object we never saw counts as a miss
