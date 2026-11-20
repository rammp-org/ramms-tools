"""World markers derived from 2D: the depth fallback in URammsDetectionWorldMarkers.

FakeUE stands in for the play session's marker component, camera provider and stream bridge, written
from RammsDetectionWorldMarkers.cpp and RammsStreamCameraBridge.cpp and checked against RammpUI PIE
on 2026-11-20. Its arithmetic is its own -- FRotator::Quaternion and a quaternion rotation, where the
tool uses FRotationMatrix's axes -- so the two agreeing means something; and the tool's arithmetic is
pinned separately to positions UE itself produced. Each switch is one measured defect, so every check
is shown both to pass on a fixed UE and to fail on the defect it is for.
"""

from __future__ import annotations

import json
import math
import socket
import sys
import threading
import types

import numpy as np
import pytest

import ramms_tools.scripts.ui_bridge_test as ubt
from ramms_tools.editor_exec import EditorExec, EditorExecError, make_msg, parse_msg
from ramms_tools.scripts.ui_bridge_test import (
    MARKER_BOX_MM, MARKER_CAMERA, MARKER_PREFIX, BridgeError, EditorMarkers, expected_derived,
    expected_sent_camera, expected_sent_world, marker_depth_image, run_marker_checks, sample_depth_cm,
)

SCENE_ID = "camera/scene/color"


# ── an independent model of the UE side ──────────────────────────────

def quat_from_rotator(pitch, yaw, roll):
    """FRotator::Quaternion, transcribed."""
    h = math.pi / 360.0
    sp, cp = math.sin(pitch * h), math.cos(pitch * h)
    sy, cy = math.sin(yaw * h), math.cos(yaw * h)
    sr, cr = math.sin(roll * h), math.cos(roll * h)
    return (cr * sp * sy - sr * cp * cy, -cr * sp * cy - sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_rotate(q, v):
    x, y, z, w = q
    t = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    return (v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]),
            v[2] + w * t[2] + (x * t[1] - y * t[0]))


def transform_position(transform, local):
    r = quat_rotate(quat_from_rotator(transform["pitch"], transform["yaw"], transform["roll"]), local)
    return (r[0] + transform["x"], r[1] + transform["y"], r[2] + transform["z"])


class FakeFeeder:
    def __init__(self):
        self.box_mm = MARKER_BOX_MM

    def set_box_mm(self, box_mm):
        self.box_mm = box_mm


class FakeUE:
    """The marker component as measured, one switch per defect (all False = today's UE)."""

    def __init__(self, feeder, asks_for_depth=False, reads_objects=False, normalizes_pixels=False,
                 refuses_outside=False, aliases=False, bound=False, scene_derivable=False,
                 stale_box_mm=None, bug=None, duplicate_markers=False, level_component=None, pose_from_depth=False,
                 camera=MARKER_CAMERA):
        self.feeder, self.camera = feeder, camera
        self.asks_for_depth, self.reads_objects = asks_for_depth, reads_objects
        self.normalizes_pixels, self.refuses_outside = normalizes_pixels, refuses_outside
        self.aliases, self.bound, self.scene_derivable = aliases, bound, scene_derivable
        self.bug = bug
        self.duplicate_markers = duplicate_markers
        # Today's UE reads the pose and intrinsics from the colour stream only; the UI bridge sends them
        # on depth only. Fixed: from whichever stream carries them.
        self.pose_from_depth = pose_from_depth
        # The level's own component, as setup describes it; `bound` = it receives bridge pushes.
        if level_component is None and bound:
            level_component = {}
        self.level_component = None if level_component is None else dict(
            {"path": "...BP_UI.RammsDetectionWorldMarkers", "accepted_sources": [], "spawn_markers": True,
             "marker_class": "RammsDetectionWorldMarker", "marker_is_ramms": True}, **level_component)
        self.projectors_dropped = []
        self.held = 0
        # What the provider holds: a stream's last forwarded raw frame, kept after forwarding stops.
        self.raw = None if stale_box_mm is None else marker_depth_image(camera, stale_box_mm)
        self.markers, self.bound_markers = {}, {}
        self.ops = []
        self.radius = 2

    # the provider
    def _frame(self):
        if self.held or self.asks_for_depth:
            self.raw = marker_depth_image(self.camera, self.feeder.box_mm)

    def _stream(self, source):
        if self.aliases and source in ("scene", ""):
            source = SCENE_ID
        if source == self.camera["color_id"]:
            return self.camera
        return None  # the scene stream: registered without intrinsics, unless scene_derivable

    def derive(self, source, point):
        self._frame()
        if self.scene_derivable and (source == SCENE_ID or (self.aliases and source == "scene")):
            source = self.camera["color_id"]
        camera = self._stream(source)
        if camera is None or (camera.get("meta_on", "depth") == "depth" and not self.pose_from_depth):
            return None
        if self.raw is None:
            return None
        nx, ny = point
        if self.refuses_outside and not (0.0 <= nx <= 1.0 and 0.0 <= ny <= 1.0):
            return None
        h, w = self.raw.shape
        px, py = min(max(math.floor(nx * w), 0), w - 1), min(max(math.floor(ny * h), 0), h - 1)
        window = self.raw[max(0, py - self.radius):py + self.radius + 1, max(0, px - self.radius):px + self.radius + 1]
        valid = np.sort(window[window > 0].astype(float) * 0.1)
        if valid.size == 0:
            return None
        d = valid[valid.size // 2]
        k = dict(camera["intrinsics"])
        transform = dict(camera["transform"])
        if self.bug == "swap_f":
            k["fx"], k["fy"] = k["fy"], k["fx"]
        if self.bug == "swap_c":
            k["cx"], k["cy"] = k["cy"], k["cx"]
        if self.bug == "no_roll":
            transform["roll"] = 0.0
        if self.bug == "y_not_flipped":
            local = (d, (nx * camera["size"][0] - k["cx"]) * d / k["fx"], (ny * camera["size"][1] - k["cy"]) * d / k["fy"])
        else:
            local = (d, (nx * camera["size"][0] - k["cx"]) * d / k["fx"], -(ny * camera["size"][1] - k["cy"]) * d / k["fy"])
        return transform_position(transform, local)

    # the payload, as Reresolve reads it
    def _numbers(self, obj, field, keys):
        value = obj.get(field)
        if isinstance(value, list) and len(value) >= len(keys):
            return [float(v) for v in value[:len(keys)]]
        if isinstance(value, dict) and self.reads_objects:
            return [float(value[k]) for k in keys]
        return None

    def place(self, payload, into):
        root = json.loads(payload)
        placed = {}
        for obj in root.get("objects", []):
            label = obj.get("perception_class") or obj.get("instance_id", "")
            source = obj.get("source") or root.get("stream_id", "")
            pos = obj.get("position_m")
            if pos is not None:
                p = (pos["X"], pos["Y"], pos["Z"])
                if obj.get("frame", "camera") == "world":
                    placed[label] = (p[0] * 100, -p[1] * 100, p[2] * 100)
                elif self._stream(source) is not None and (self._stream(source).get("meta_on", "depth") == "both"
                                                           or self.pose_from_depth):
                    placed[label] = transform_position(self.camera["transform"], (p[2] * 100, p[0] * 100, -p[1] * 100))
                continue
            point = self._numbers(obj, "centroid", "XY")
            if point is None:
                box = self._numbers(obj, "box", "XYZW")
                point = None if box is None else [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]
            if point is None:
                continue
            if self.normalizes_pixels and (point[0] > 1.0 or point[1] > 1.0):
                point = [point[0] / self.camera["size"][0], point[1] / self.camera["size"][1]]
            where = self.derive(source, point)
            if where is not None:
                placed[label] = where
        into.clear()
        into.update(placed)

    # the editor-side protocol EditorMarkers speaks
    def call(self, op, **params):
        self.ops.append(op)
        if op == "setup":
            self.markers.clear()
            self.projectors_dropped.append(("setup", tuple(params.get("stream_ids", ()))))
            return {"world": "/Game/Maps/UEDPIE_0_Map_UI.Map_UI", "component": "...RammsDetectionWorldMarkers_9",
                    "level_components": [self.level_component] if self.level_component else [],
                    "bridges": ["...CameraBridge"], "sample_radius": self.radius}
        if op == "derive":
            return {"points": [None if (g := self.derive(params["source"], p)) is None else list(g)
                               for p in params["points"]]}
        if op == "apply":
            self.place(params["payload"], self.markers)
            return {"markers": self._listed(params.get("prefix", ""))}
        if op == "markers":
            return {"markers": self._listed(params.get("prefix", ""))}
        if op == "depth":
            self.held = self.held + 1 if params.get("request") else 0
            return {"held": self.held}
        if op == "teardown":
            self.markers.clear()
            self.held = 0
            self.projectors_dropped.append(("teardown", tuple(params.get("stream_ids", ()))))
            return {"ok": True}
        raise AssertionError(op)

    def _listed(self, prefix):
        both = list(self.markers.items()) + list(self.bound_markers.items())
        if self.duplicate_markers:
            both += list(self.markers.items())  # a second actor for the same object
        return [{"label": k, "location": list(v)} for k, v in both if k.startswith(prefix)]


class FakeClient:
    """Remote Control: the bridge receiver's UpdateDetectedObjects, and the scene stream setting."""

    def __init__(self, ue):
        self.ue = ue
        self.pushes = []

    def update_detected_objects(self, payload):
        self.pushes.append(payload)
        level = self.ue.level_component
        if self.ue.bound and level:
            accepted = level["accepted_sources"]
            objects = [o for o in payload["objects"] if not accepted or o.get("source", "") in accepted]
            self.ue.place(json.dumps(dict(payload, objects=objects)), self.ue.bound_markers)

    def get_setting(self, name):
        return SCENE_ID if name == "SceneColorStreamID" else ""


def _run(feeder=None, **switches):
    feeder = feeder or FakeFeeder()
    ue = FakeUE(feeder, **switches)
    check, failures = ubt._reporter()
    clock = iter(range(0, 100000))
    run_marker_checks(ue, FakeClient(ue), check, feeder, camera=ue.camera, timeout=2.0, sleep=lambda s: None,
                      clock=lambda: next(clock) * 0.25, unaided_box_mm=1500)
    return failures, ue


FIXED = dict(asks_for_depth=True, reads_objects=True, normalizes_pixels=True, refuses_outside=True,
             aliases=True, bound=True, scene_derivable=True, pose_from_depth=True)
COLOUR_META = ubt.MARKER_CAMERA_COLOUR_META

PUSHED = "a 2D object pushed through the receiver gets a derived marker from the level's component"

TODAY = {
    "a derived position from the current depth, with only what the component asks for itself",
    "no position for a point outside the image (1.25, 0.5)",
}


def _named(failures, needle):
    return [f for f in failures if needle in f]


# ── the checks against the model ─────────────────────────────────────

def test_on_todays_ue_exactly_the_measured_defects_fail(capsys):
    # measured with the intrinsics and pose on both streams, as the first version of the test sent them
    failures, ue = _run(camera=COLOUR_META)
    out = capsys.readouterr().out
    assert set(failures) - {f for f in failures if f.startswith("centroid as {") or f.startswith("centroid in pixels")
                            or f.startswith("box only, as {")} == TODAY
    assert len(_named(failures, 'centroid as {"X", "Y"}')) == 1
    assert len(_named(failures, "centroid in pixels")) == 1
    assert len(_named(failures, 'box only, as {"X"')) == 1
    assert len(failures) == 5
    assert "no depth bytes reach the camera provider" in out
    # the projection itself, the controls and the array forms pass
    for line in ("pass  the box in front of the wall, image centre", "pass  the wall, top-left pixel",
                 "pass  centroid as [x, y]", "pass  box only, as [x0, y0, x1, y1]",
                 "pass  sent position_m, world frame", "pass  sent position_m, camera frame",
                 "pass  a centroid where the depth has no return: no marker"):
        assert line in out, line
    assert "warn  the level's world-marker component places markers from bridge pushes -- there is none" in out
    assert 'warn  the bridge\'s source "scene"' in out
    assert ue.ops[-1] == "teardown" and ue.held == 0


def test_on_todays_ue_the_bridges_metadata_places_nothing_derived(capsys):
    # the UI bridge sends the intrinsics and pose on the depth frames only; today's UE reads the colour's
    failures, _ = _run()
    assert "a derived position from the current depth, with depth forwarded" in failures
    out = capsys.readouterr().out
    assert "pass  the wall" not in out and "pass  sent position_m, world frame" not in out  # stopped before them


@pytest.mark.parametrize("camera", [MARKER_CAMERA, COLOUR_META], ids=["depth-meta", "colour-meta"])
def test_a_fixed_ue_places_both_kinds_of_producer(camera):
    failures, _ = _run(**dict(FIXED, camera=camera))
    assert failures == []


def test_without_the_depth_fallback_the_bridges_form_fails_but_colour_metadata_passes():
    assert _run(**dict(FIXED, pose_from_depth=False))[0]
    assert _run(**dict(FIXED, pose_from_depth=False, camera=COLOUR_META))[0] == []


def test_on_a_fixed_ue_everything_passes(capsys):
    failures, _ = _run(**FIXED)
    out = capsys.readouterr().out
    assert failures == [], failures
    assert "warn" not in out
    assert 'pass  the bridge\'s source "scene" places objects on the scene camera' in out
    assert "pass  the level's world-marker component places markers from bridge pushes" in out


@pytest.mark.parametrize("missing, fails", [
    ("asks_for_depth", "a derived position from the current depth, with only what the component asks for itself"),
    ("refuses_outside", "no position for a point outside the image (1.25, 0.5)"),
    ("reads_objects", 'centroid as {"X", "Y"}'),
    ("normalizes_pixels", "centroid in pixels"),
])
def test_each_defect_alone_fails_its_own_check(missing, fails):
    failures, _ = _run(**dict(FIXED, **{missing: False}))
    # The pushed object is the bridge's own form -- {"X", "Y"} pixels -- so it fails with either reader defect too.
    also = {"reads_objects": {'box only, as {"X"', PUSHED}, "normalizes_pixels": {PUSHED}}.get(missing, set())
    own = [f for f in failures if not any(a in f for a in also)]
    assert own and all(fails in f for f in own), failures
    assert len(own) == 1 and len(failures) == 1 + len(also), failures


def test_the_alias_check_fails_when_the_scene_camera_derives_but_scene_does_not(capsys):
    failures, _ = _run(**dict(FIXED, aliases=False))
    assert failures == ['the bridge\'s source "scene" places objects on the scene camera']
    assert "looks `source` up as a stream id" in capsys.readouterr().out


def test_without_a_bound_component_it_warns_and_still_checks_with_its_own(capsys):
    failures, ue = _run(**dict(FIXED, bound=False))
    out = capsys.readouterr().out
    assert failures == [] and "warn  the level's world-marker component places markers from bridge pushes -- there is none" in out
    assert ue.bound_markers == {}


def test_a_level_component_that_does_not_receive_pushes_fails(capsys):
    failures, _ = _run(**dict(FIXED, bound=False, level_component={}))
    assert failures == ["the level's world-marker component places markers from bridge pushes", PUSHED]
    assert "not receiving bridge pushes" in capsys.readouterr().out


def test_the_probe_is_tagged_with_a_source_the_component_accepts():
    feeder = FakeFeeder()
    ue = FakeUE(feeder, **dict(FIXED, level_component={"accepted_sources": ["scene", "camera/scene/color"]}))
    client = FakeClient(ue)
    check, failures = ubt._reporter()
    run_marker_checks(ue, client, check, feeder, timeout=2.0, sleep=lambda s: None,
                      clock=iter(range(100000)).__next__, unaided_box_mm=1500)
    assert client.pushes[0]["objects"][0]["source"] == "scene" and failures == []
    # the pushed-object check can't use the test camera's id with this component, and says so
    assert len(client.pushes) == 2


@pytest.mark.parametrize("component, says", [
    ({"spawn_markers": False}, "has bSpawnMarkers off"),
    ({"marker_is_ramms": False, "marker_class": "BP_Ghost_C"}, "MarkerClass (BP_Ghost_C) is not a RammsDetectionWorldMarker"),
])
def test_a_component_whose_markers_cannot_be_read_back_is_not_failed(component, says, capsys):
    failures, _ = _run(**dict(FIXED, bound=False, level_component=component))
    out = capsys.readouterr().out
    assert failures == [] and says in out and "warn  the level's world-marker component" in out


def test_the_pushed_object_check_reports_where_the_level_component_put_it(capsys):
    failures, _ = _run(**FIXED)
    assert failures == [] and f"pass  {PUSHED}, at (238.6, 122.9, 139.3)" in capsys.readouterr().out


def test_in_scene_mode_the_real_scene_cameras_projector_is_left_alone():
    camera = dict(MARKER_CAMERA, color_id=SCENE_ID, depth_id="camera/scene/depth", group="camera/scene", own_ids=False)
    feeder = FakeFeeder()
    ue = FakeUE(feeder, **dict(FIXED, camera=camera))
    check, _ = ubt._reporter()
    run_marker_checks(ue, FakeClient(ue), check, feeder, camera=camera, timeout=2.0, sleep=lambda s: None,
                      clock=iter(range(100000)).__next__, unaided_box_mm=1500)
    assert ue.projectors_dropped == [("setup", ()), ("teardown", ())]


def test_in_scene_mode_an_object_exactly_as_the_bridge_sends_it_is_checked(capsys):
    camera = dict(MARKER_CAMERA, color_id=SCENE_ID, depth_id="camera/scene/depth", group="camera/scene", own_ids=False)
    feeder = FakeFeeder()
    ue = FakeUE(feeder, **dict(FIXED, camera=camera))
    check, failures = ubt._reporter()
    run_marker_checks(ue, FakeClient(ue), check, feeder, camera=camera, timeout=2.0, sleep=lambda s: None,
                      clock=iter(range(100000)).__next__, unaided_box_mm=1500)
    out = capsys.readouterr().out
    assert failures == [], failures
    assert 'pass  the bridge\'s source "scene" places objects on the scene camera' in out
    assert "pass  an object exactly as the bridge sends it" in out
    ue = FakeUE(feeder, **dict(FIXED, camera=camera, aliases=False))
    check, failures = ubt._reporter()
    run_marker_checks(ue, FakeClient(ue), check, feeder, camera=camera, timeout=2.0, sleep=lambda s: None,
                      clock=iter(range(100000)).__next__, unaided_box_mm=1500)
    assert 'the bridge\'s source "scene" places objects on the scene camera' in failures


def test_the_test_cameras_projector_is_dropped_before_and_after():
    _, ue = _run(**FIXED)
    ids = (MARKER_CAMERA["color_id"], MARKER_CAMERA["depth_id"])
    assert ue.projectors_dropped == [("setup", ids), ("teardown", ids)]


def test_the_bound_probe_goes_through_the_receiver_and_is_cleared():
    feeder = FakeFeeder()
    ue = FakeUE(feeder, **FIXED)
    client = FakeClient(ue)
    check, _ = ubt._reporter()
    run_marker_checks(ue, client, check, feeder, timeout=2.0, sleep=lambda s: None,
                      clock=iter(range(100000)).__next__, unaided_box_mm=1500)
    probe, clear, pushed, clear_again = client.pushes
    assert probe["objects"][0]["instance_id"] == MARKER_PREFIX + "bound" and clear["objects"] == []
    # the bridge's own form: an {"X", "Y"} centroid in pixels
    assert pushed["objects"][0]["centroid"] == {"X": 256.0, "Y": 48.0} and clear_again["objects"] == []
    assert ue.bound_markers == {}


def test_two_markers_for_one_object_fail(capsys):
    failures, _ = _run(**dict(FIXED, duplicate_markers=True))
    placed = [f for f in failures if ": marker at" in f]
    assert placed and "2 marker(s)" in capsys.readouterr().out


def test_depth_from_an_old_frame_is_caught(capsys):
    # an earlier run's frame, still held by the provider (measured with metadata on both streams)
    failures, _ = _run(stale_box_mm=MARKER_BOX_MM, camera=COLOUR_META)
    out = capsys.readouterr().out
    assert "a derived position from the current depth, with only what the component asks for itself" in failures
    assert "derived from an OLD depth frame" in out


@pytest.mark.parametrize("bug", ["swap_f", "swap_c", "no_roll", "y_not_flipped"])
def test_a_projection_bug_is_not_hidden_by_symmetry(bug):
    failures, _ = _run(**dict(FIXED, bug=bug))
    direct = [f for f in failures if f.startswith(("the box in front", "the wall,"))]
    assert direct, f"{bug} slipped past the direct derive checks: {failures}"


def test_a_held_depth_request_is_released_even_when_a_check_raises():
    feeder = FakeFeeder()
    ue = FakeUE(feeder, **FIXED)
    real = ue.call

    def failing(op, **params):
        if op == "apply":
            raise BridgeError("the editor's apply step failed: boom")
        return real(op, **params)
    ue.call = failing
    check, _ = ubt._reporter()
    with pytest.raises(BridgeError):
        run_marker_checks(ue, FakeClient(ue), check, feeder, timeout=2.0, sleep=lambda s: None,
                          clock=iter(range(100000)).__next__, unaided_box_mm=1500)
    assert ue.ops[-1] == "teardown" and ue.held == 0 and feeder.box_mm == MARKER_BOX_MM


# ── the tool's arithmetic, pinned to what UE produced ────────────────

PROBE_CAMERA = dict(MARKER_CAMERA, intrinsics={"fx": 250.0, "fy": 250.0, "cx": 160.0, "cy": 120.0},
                    transform={"x": 100.0, "y": -50.0, "z": 120.0, "pitch": -10.0, "yaw": 30.0, "roll": 0.0})


def test_expected_positions_match_what_derive_world_location_returned_in_pie():
    # DeriveWorldLocation in RammpUI PIE, 2026-11-20, 320x240 stream, flat 2 m depth
    assert expected_derived(0.5, 0.5, 200.0, PROBE_CAMERA) == pytest.approx((270.573706, 48.480775, 85.270364), abs=1e-3)
    assert expected_derived(0.75, 0.25, 200.0, PROBE_CAMERA) == pytest.approx((245.792126, 108.073957, 132.541137), abs=1e-3)


def test_expected_positions_match_the_live_markers_run_with_roll_and_asymmetry():
    # `markers` in RammpUI PIE, 2026-11-20 (MARKER_CAMERA; printed to 0.1 cm)
    depth = marker_depth_image()
    for point, live in (((0.5, 0.5), (202.0, 10.0, 100.1)), ((0.8, 0.2), (238.6, 122.9, 139.3)),
                        ((0.0, 0.0), (343.5, -39.0, 195.4))):
        assert expected_derived(*point, sample_depth_cm(depth, *point, 2)) == pytest.approx(live, abs=0.051)
    assert expected_sent_camera((0.10, -0.05, 1.50)) == pytest.approx((223.3, 33.2, 98.0), abs=0.051)
    assert expected_sent_world((1.0, 0.5, 0.8)) == pytest.approx((100.0, -50.0, 80.0))


def test_the_matrix_and_the_quaternion_forms_agree_for_any_pose():
    rng = np.random.default_rng(7)
    for _ in range(50):
        t = dict(zip(("x", "y", "z", "pitch", "yaw", "roll"), rng.uniform(-180, 180, 6)))
        v = tuple(rng.uniform(-500, 500, 3))
        assert ubt.camera_local_to_world(v, t) == pytest.approx(transform_position(t, v), abs=1e-6)


def test_depth_sampling_follows_sample_depth_uu():
    depth = marker_depth_image()
    assert sample_depth_cm(depth, 0.5, 0.5, 2) == 120.0                 # the box
    assert sample_depth_cm(depth, 0.8, 0.2, 2) == 200.0                 # the wall
    assert sample_depth_cm(depth, 0.22, 0.22, 2) is None                # no return
    assert sample_depth_cm(depth, 1.25, 0.5, 2) == 200.0                # clamped to the edge, as UE does
    edge = np.array([[0, 0, 0, 1000, 1000]], np.uint16)                 # median of the VALID samples only:
    assert sample_depth_cm(edge, 0.4, 0.0, 2) == 100.0                  # pixel 2's window holds all five
    assert sample_depth_cm(edge, 0.1, 0.0, 2) is None                   # pixel 0's holds only the zeros


# ── the streamed camera ──────────────────────────────────────────────

class RecordingSender:
    def __init__(self):
        self.sent = []

    def send_image(self, channel, data, w, h, **kw):
        self.sent.append(("image", channel, len(data), w, h, kw))

    def send_depth_uint16(self, channel, data, w, h, **kw):
        self.sent.append(("depth", channel, data, w, h, kw))


def test_the_feeder_sends_both_streams_with_the_metadata_the_bridge_reads():
    feeder = ubt.MarkerCameraFeeder("127.0.0.1", 30030, COLOUR_META)
    feeder.sender = RecordingSender()
    feeder.send_once(*feeder.frames())
    (kind_c, ch_c, n_c, w, h, kw_c), (kind_d, ch_d, d, _, _, kw_d) = feeder.sender.sent
    assert (kind_c, kind_d) == ("image", "depth") and (ch_c, ch_d) == COLOUR_META["channels"]
    assert n_c == w * h * 4 and len(d) == w * h * 2
    for kw, role, sid in ((kw_c, "color", COLOUR_META["color_id"]), (kw_d, "depth", COLOUR_META["depth_id"])):
        assert kw["group"] == COLOUR_META["group"] and kw["role"] == role and kw["stream_id"] == sid
        assert kw["metadata"]["intrinsics"] == COLOUR_META["intrinsics"]   # {fx, fy, cx, cy}
        assert set(kw["metadata"]["transform"]) == {"x", "y", "z", "pitch", "yaw", "roll"}
    depth = np.frombuffer(d, np.uint16).reshape(h, w)
    assert depth[h // 2, w // 2] == MARKER_BOX_MM                        # millimetres, as the format says
    feeder.set_box_mm(1500)
    assert np.frombuffer(feeder.frames()[1], np.uint16).reshape(h, w)[h // 2, w // 2] == 1500


def test_by_default_the_feeder_sends_the_metadata_on_depth_only_as_the_bridge_does():
    feeder = ubt.MarkerCameraFeeder("127.0.0.1", 30030)
    feeder.sender = RecordingSender()
    feeder.send_once(*feeder.frames())
    (_, _, _, _, _, kw_c), (_, _, _, _, _, kw_d) = feeder.sender.sent
    assert kw_c["metadata"] is None
    assert kw_d["metadata"]["intrinsics"] == MARKER_CAMERA["intrinsics"] and "transform" in kw_d["metadata"]


# ── the editor-side script, run against a stub of the unreal module ──

class _Obj:
    def __init__(self, name, world=None):
        self.name, self.world, self.valid = name, world, True

    def get_path_name(self):
        return self.name

    def get_world(self):
        return self.world


class _StubWorld(_Obj):
    def __init__(self):
        super().__init__("/Game/Maps/UEDPIE_0_Map_UI.Map_UI")
        self.world = self
        self.actors = []


def _stub_unreal(world):
    u = types.ModuleType("unreal")

    class Actor(_Obj):
        def __init__(self, name, comps=()):
            super().__init__(name, world)
            self.comps = list(comps)

        def get_components_by_class(self, cls):
            return [c for c in self.comps if isinstance(c, cls) and c.valid]

    class RammsStreamCameraBridge(_Obj):
        def __init__(self, owner):
            super().__init__(owner.name + ".CameraBridge", world)
            self.owner, self.held = owner, 0

        def get_owner(self):
            return self.owner

        def request_raw_data_forwarding(self):
            self.held += 1

        def release_raw_data_forwarding(self):
            self.held -= 1

    class RammsDetectionWorldMarkers(_Obj):
        made = []

        def __init__(self, outer):
            super().__init__(outer.name + f".RammsDetectionWorldMarkers_{len(type(self).made)}", world)
            self.outer, self.payloads = outer, []
            type(self).made.append(self)
            outer.comps.append(self)

        def get_editor_property(self, name):
            return {"depth_sample_radius": 2, "marker_class": types.SimpleNamespace(get_name=lambda: "RammsDetectionWorldMarker"),
                    "accepted_sources": [], "spawn_markers": True}[name]

        def derive_world_location(self, source, point):
            return u.Vector(point.x * 10, point.y * 10, 1.0) if source == "cam" else None

        def set_detected_objects_json(self, payload):
            self.payloads.append(payload)

        def destroy_component(self, obj):
            self.valid = False

        def is_being_destroyed(self):
            return not self.valid

    class RammsDetectionWorldMarker:
        @staticmethod
        def static_class():
            return "RammsDetectionWorldMarker"

    class RammsCameraProjectionManager(_Obj):
        def __init__(self, owner):
            super().__init__(owner.name + ".ProjectionManager", world)
            self.removed = []

        def remove_projector(self, stream_id):
            self.removed.append(stream_id)

    class Vector:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class Vector2D:
        def __init__(self, x, y):
            self.x, self.y = x, y

    receiver = Actor("/Game/Maps/UEDPIE_0_Map_UI.Map_UI:PersistentLevel.RammsStreamReceiver_0")
    receiver.comps.append(RammsStreamCameraBridge(receiver))
    receiver.comps.append(RammsCameraProjectionManager(receiver))
    world.actors = [receiver]
    u.Actor, u.RammsStreamCameraBridge, u.RammsDetectionWorldMarkers = Actor, RammsStreamCameraBridge, RammsDetectionWorldMarkers
    u.RammsDetectionWorldMarker, u.Vector, u.Vector2D = RammsDetectionWorldMarker, Vector, Vector2D
    u.RammsCameraProjectionManager = RammsCameraProjectionManager
    u.MathLibrary = types.SimpleNamespace(class_is_child_of=lambda cls, parent: True)
    u.UnrealEditorSubsystem = object
    u.get_editor_subsystem = lambda cls: types.SimpleNamespace(get_game_world=lambda: world)
    u.GameplayStatics = types.SimpleNamespace(
        get_all_actors_of_class=lambda w, cls: [a for a in w.actors if isinstance(a, cls)])
    u.SystemLibrary = types.SimpleNamespace(is_valid=lambda o: getattr(o, "valid", False))
    u.new_object = lambda cls, outer: cls(outer)
    u.receiver = receiver
    return u


class ExecEditor:
    """Runs each script as the editor does: fresh globals every time, sys.modules shared."""

    def __init__(self):
        self.scripts = 0

    def run(self, code):
        lines = []
        g = {"__name__": "__main__", "print": lambda *a: lines.append(" ".join(str(x) for x in a))}
        exec(compile(code, "<editor>", "exec"), g)
        self.scripts += 1
        return lines


@pytest.fixture
def stub_editor(monkeypatch):
    world = _StubWorld()
    u = _stub_unreal(world)
    monkeypatch.setitem(sys.modules, "unreal", u)
    monkeypatch.delitem(sys.modules, "_ramms_markers_test", raising=False)
    yield EditorMarkers(ExecEditor()), u
    sys.modules.pop("_ramms_markers_test", None)


def test_the_editor_script_creates_its_own_component_and_destroys_it(stub_editor):
    markers, u = stub_editor
    ids = ["test/markers/color", "test/markers/depth"]
    setup = markers.call("setup", stream_ids=ids)
    comp = u.RammsDetectionWorldMarkers.made[0]
    manager = u.receiver.comps[1]
    assert setup["component"] == comp.name and setup["sample_radius"] == 2 and setup["bridges"]
    assert setup["level_components"] == []          # its own is made after the level's are listed
    assert manager.removed == ids
    assert markers.call("derive", source="cam", points=[[0.5, 0.25], [0.1, 0.1]])["points"] == [[5.0, 2.5, 1.0], [1.0, 1.0, 1.0]]
    assert markers.call("derive", source="nope", points=[[0.5, 0.5]])["points"] == [None]
    applied = json.dumps({"objects": [{"instance_id": MARKER_PREFIX + "a", "perception_class": MARKER_PREFIX + "a"}]})
    markers.call("apply", payload=applied, prefix=MARKER_PREFIX)
    assert comp.payloads == [applied]
    markers.call("teardown", stream_ids=ids)
    # cleared BEFORE it goes: an unregistered component never ends play, so destroying it alone
    # would leave its markers in the world
    assert comp.valid is False and comp.payloads == [applied, '{"objects": []}']
    assert manager.removed == ids + ids


def test_the_level_components_are_described_and_the_dead_runs_is_not_among_them(stub_editor):
    markers, u = stub_editor
    level = u.RammsDetectionWorldMarkers(u.receiver)          # one the level carries
    markers.call("setup")
    second = markers.call("setup")                            # the first run's own is destroyed now
    assert [c["path"] for c in second["level_components"]] == [level.name]
    assert second["level_components"][0]["marker_is_ramms"] and second["level_components"][0]["spawn_markers"]


def test_a_depth_request_left_by_a_run_that_died_is_released_by_the_next(stub_editor):
    markers, u = stub_editor
    bridge = u.receiver.comps[0]
    markers.call("setup")
    markers.call("depth", request=True)
    assert bridge.held == 1
    # the run dies here: no teardown. The next run's setup finds the request in the shared module.
    markers.call("setup")
    assert bridge.held == 0
    assert u.RammsDetectionWorldMarkers.made[0].valid is False   # and the dead run's component is gone


def test_an_editor_side_error_surfaces_as_a_bridge_error(stub_editor):
    markers, u = stub_editor
    with pytest.raises(BridgeError, match="the editor's derive step failed"):
        markers.call("derive", source="cam", points=[[0.5, 0.5]])   # no setup: no component yet


def test_a_play_session_is_required(stub_editor, monkeypatch):
    markers, u = stub_editor
    monkeypatch.setattr(u, "get_editor_subsystem", lambda cls: types.SimpleNamespace(get_game_world=lambda: None))
    with pytest.raises(BridgeError, match="no play session"):
        markers.call("setup")


# ── the command ──────────────────────────────────────────────────────

def test_markers_without_the_editors_python_says_so(monkeypatch, capsys):
    class Feeder(FakeFeeder):
        error = None

        def __init__(self, *a, **k):
            super().__init__()

        def start(self):
            return None

        def stop(self):
            self.stopped = True

    class NoEditor:
        def __init__(self, **kw):
            pass

        def __enter__(self):
            raise EditorExecError("no RammpUI editor answered Python remote execution")

        def __exit__(self, *exc):
            return False
    monkeypatch.setattr(ubt, "MarkerCameraFeeder", Feeder)
    monkeypatch.setattr("ramms_tools.editor_exec.EditorExec", NoEditor)
    args = types.SimpleNamespace(host="127.0.0.1", stream_port=30030, editor_project="RammpUI", timeout=1.0)
    assert ubt.cmd_markers(object(), args) == 1
    assert "cannot use the editor's Python: no RammpUI editor answered" in capsys.readouterr().out


def test_markers_without_rmss_says_so(monkeypatch, capsys):
    class Feeder:
        def __init__(self, *a, **k):
            pass

        def start(self):
            return "cannot connect to RMSS on 127.0.0.1:30030 (refused)"
    monkeypatch.setattr(ubt, "MarkerCameraFeeder", Feeder)
    args = types.SimpleNamespace(host="127.0.0.1", stream_port=30030, editor_project="RammpUI", timeout=1.0)
    assert ubt.cmd_markers(object(), args) == 1
    assert "cannot stream the test camera" in capsys.readouterr().out


# ── the remote-execution client ──────────────────────────────────────

def test_messages_round_trip_and_a_partial_one_is_not_taken():
    raw = make_msg("command_result", "a", "b", {"success": True})
    assert parse_msg(raw)["data"] == {"success": True}
    assert parse_msg(raw[:-3]) is None
    assert parse_msg(b'{"magic": "other", "version": 1}') is None


def _editor_answering(result_data, chunked=False):
    ours, theirs = socket.socketpair()
    ours.settimeout(5)  # a fake that never answers fails the test instead of hanging it
    editor = EditorExec(project="RammpUI", command_timeout=5)
    editor._conn, editor.remote_id = ours, "remote"

    def answer():
        request = parse_msg(theirs.recv(1 << 20))
        if not (request and request["type"] == "command" and request["data"]["exec_mode"] == "ExecuteFile"):
            theirs.close()  # the client sees EOF and raises
            return
        reply = make_msg("command_result", "remote", editor.node_id, result_data)
        if chunked:
            for i in range(0, len(reply), 7):
                theirs.sendall(reply[i:i + 7])
        else:
            theirs.sendall(reply)
    threading.Thread(target=answer, daemon=True).start()
    return editor, theirs


def test_run_returns_the_printed_lines_even_when_the_result_arrives_in_pieces():
    editor, other = _editor_answering({"success": True, "output": [{"type": "Info", "output": "one\n"},
                                                                   {"type": "Info", "output": "two"}]}, chunked=True)
    try:
        assert editor.run("print('one')\nprint('two')") == ["one", "two"]
    finally:
        other.close()
        editor._conn.close()


def test_run_raises_with_the_editors_output_when_the_script_failed():
    editor, other = _editor_answering({"success": False, "output": [{"type": "Info", "output": "started"}],
                                       "result": "Traceback (most recent call last):\nValueError: boom"})
    try:
        with pytest.raises(EditorExecError, match="(?s)started.*ValueError: boom"):
            editor.run("raise ValueError('boom')")
    finally:
        other.close()
        editor._conn.close()


def test_run_without_a_connection_says_how_to_get_one():
    with pytest.raises(EditorExecError, match="context manager"):
        EditorExec().run("print(1)")
