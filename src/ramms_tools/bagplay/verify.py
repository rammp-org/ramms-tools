"""Check that a replay landed: world markers where the bag says the objects are, joints as sent.

World markers are read through the editor's Python (the project's Remote Control allow-list covers only
the bridge receiver), so this needs an editor play session. Expected positions come from the bag alone:
the tracker's camera-frame pose, through the camera extrinsic the player sent ("relative", so composed
with the UE stream receiver's own transform, as RammsStreamCameraBridge does).
"""

from __future__ import annotations

import json
import math

from ramms_tools.scripts.ui_bridge_test import ue_rotate

SENT_TOLERANCE_CM = 1.0
DERIVED_TOLERANCE_CM = 1.0   # against the position RammsUI must derive from the same depth frame

EDITOR_SCRIPT = r'''
import json, unreal
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
out = {"error": "no play session: start PIE first"} if world is None else {}
if world is not None:
    receivers = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
                 if a.get_class().get_name() == "RammsStreamReceiver"]
    if receivers:
        t = receivers[0].get_actor_transform()
        r = t.rotation.rotator()
        out["receiver"] = {"location": [t.translation.x, t.translation.y, t.translation.z],
                           "rotation": [r.pitch, r.yaw, r.roll], "scale": [t.scale3d.x, t.scale3d.y, t.scale3d.z]}
    level = [c for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
             for c in a.get_components_by_class(unreal.RammsDetectionWorldMarkers)]
    out["depth_sample_radius"] = int(level[0].get_editor_property("depth_sample_radius")) if level else None
    out["markers"] = []
    for m in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.RammsDetectionWorldMarker):
        loc = m.get_actor_location()
        out["markers"].append({"label": str(m.get_label_text()), "id": str(m.get_detection().instance_id),
                               "location": [loc.x, loc.y, loc.z]})
print("RAMMS_BAG_VERIFY " + json.dumps(out))
'''


def transform_point(location, rotation, point):
    """A UE transform (location cm, FRotator pitch/yaw/roll deg) applied to a point."""
    r = ue_rotate(rotation[0], rotation[1], rotation[2], point)
    return (r[0] + location[0], r[1] + location[1], r[2] + location[2])


def expected_marker(obj, extrinsic, receiver):
    """Where a camera-frame position_m lands in UE: optical -> UE-local (z, x, -y) cm, the camera's extrinsic,
    then the receiver actor (the extrinsic is sent "relative")."""
    p = obj["position_m"]
    local = (p["Z"] * 100.0, p["X"] * 100.0, -p["Y"] * 100.0)
    camera = transform_point((extrinsic["x"], extrinsic["y"], extrinsic["z"]),
                             (extrinsic["pitch"], extrinsic["yaw"], extrinsic["roll"]), local)
    return transform_point(receiver["location"], receiver["rotation"], camera)


def expected_derived_marker(centroid, depth_mm, radius, intrinsics, extrinsic, receiver, image_size=None):
    """Where RammsUI must put a derived marker: the median of the valid depth in a (2r+1)^2 window at the
    centroid's pixel, unprojected through the intrinsics, then the camera's pose and the receiver's
    (RammsDetectionWorldMarkers::DeriveWorldLocation). None where the window holds no depth."""
    from ramms_tools.scripts.ui_bridge_test import sample_depth_cm
    # Depth is sampled on its own grid; the point is unprojected through the colour image's size and
    # intrinsics (DeriveWorldLocation), which are the same here but need not be.
    w, h = image_size or (depth_mm.shape[1], depth_mm.shape[0])
    d = sample_depth_cm(depth_mm, centroid[0], centroid[1], radius)
    if d is None:
        return None
    u, v = centroid[0] * w, centroid[1] * h
    local = (d, (u - intrinsics["cx"]) * d / intrinsics["fx"], -(v - intrinsics["cy"]) * d / intrinsics["fy"])
    camera = transform_point((extrinsic["x"], extrinsic["y"], extrinsic["z"]),
                             (extrinsic["pitch"], extrinsic["yaw"], extrinsic["roll"]), local)
    return transform_point(receiver["location"], receiver["rotation"], camera)


def read_editor(editor):
    lines = editor.run(EDITOR_SCRIPT)
    found = next((json.loads(l[len("RAMMS_BAG_VERIFY "):]) for l in lines if l.startswith("RAMMS_BAG_VERIFY ")), None)
    if found is None:
        raise RuntimeError(f"the editor printed no result: {lines[-3:]}")
    if "error" in found:
        raise RuntimeError(found["error"])
    return found


def check_markers(state, readback, check, positions="sent", depth=None, intrinsics=None, log=print, image_size=None):
    """Each 3D object of the last push has a marker of its class where the bag says.

    state: PlayStats.last_3d_payload ({"payload", "extrinsic", "objects"}); readback: read_editor()'s result.
    positions="derived": the push carried no 3D, and RammsUI derived each marker from the depth frame of
    the same stamp. `depth` (that frame, mm) and `intrinsics` give the exact position it must have derived;
    how far that is from the tracker's own 3D centre is reported too, as information -- the depth at a
    box's centre is whatever the middle of the box sees, which for a table or a doorway is behind it.
    """
    receiver = readback.get("receiver")
    if receiver is None:
        check(False, "a RammsStreamReceiver in the play session", "none found")
        return
    if any(abs(s - 1.0) > 1e-6 for s in receiver.get("scale", [1, 1, 1])):
        check(False, "the stream receiver is unscaled", f"scale {receiver['scale']}: positions are not comparable")
        return
    markers = readback["markers"]
    payload, extrinsic = state["payload"], state["extrinsic"]
    if extrinsic is None:
        check(False, "the camera's pose for the last push", "the bag's TF has no pose for its frame")
        return
    tracker = {o["instance_id"]: o for o in state.get("objects") or []}
    radius = readback.get("depth_sample_radius")
    camera = payload.get("stream_id")
    for obj in payload["objects"]:
        if camera and obj.get("source", camera) != camera:
            continue  # another camera's 2D boxes (the segmenter's, on the overlay): no 3D marker is expected of them
        name = obj.get("perception_class") or obj.get("instance_id")
        if positions == "sent":
            if "position_m" not in obj:
                continue
            want = expected_marker(obj, extrinsic, receiver)
        else:
            if "centroid" not in obj:
                continue  # occluded: nothing to derive from
            if depth is None or intrinsics is None or radius is None:
                check(False, f"{obj.get('instance_id')}: the depth to derive it from", "not available to the check")
                continue
            want = expected_derived_marker(obj["centroid"], depth, radius, intrinsics, extrinsic, receiver, image_size)
            if want is None:
                log(f"        {obj.get('instance_id')}: no depth at its centre, so no marker is expected")
                continue
        # By instance id where the editor reports it -- two food containers share a label.
        mine = [m for m in markers if m.get("id") == obj.get("instance_id")] or \
            [m for m in markers if "id" not in m and m["label"] == name]
        what = (f"{obj.get('instance_id')} ({name}): a marker within 1.0 cm of "
                f"({want[0]:.1f}, {want[1]:.1f}, {want[2]:.1f})")
        if not mine:
            check(False, what, "no marker of that class")
            continue
        nearest = min(mine, key=lambda m: math.dist(m["location"], want))
        best = math.dist(nearest["location"], want)
        check(best <= (SENT_TOLERANCE_CM if positions == "sent" else DERIVED_TOLERANCE_CM), what,
              f"the nearest is {best:.1f} cm away")
        own = tracker.get(obj.get("instance_id"))
        if positions == "derived" and own and own.get("has_pose"):
            p = own["pose"]["position"]
            centre = expected_marker({"position_m": {"X": p["x"], "Y": p["y"], "Z": p["z"]}}, extrinsic, receiver)
            log(f"        derived from depth, it is {math.dist(nearest['location'], centre):.0f} cm from the "
                f"tracker's own 3D centre")


def check_joints(sinks, check):
    """The display holds the last joint vector the replay sent (every joint within its limits)."""
    from ramms_tools.scripts.ui_bridge_test import within_limits
    rig, sent = sinks.rig, sinks.sent_joint_vector
    if rig is None or sent is None:
        return
    landed = rig.targets()
    wrong = [(rig.joints[i].get("JointName"), s, g) for i, (s, g) in enumerate(zip(sent, landed))
             if within_limits(rig.joints[i], s) and abs(s - g) > 1e-3]
    check(not wrong and len(landed) == len(sent), "the robot display holds the replayed joints",
          f"e.g. {wrong[:3]}")
