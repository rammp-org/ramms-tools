"""ROS 2 -> Unreal conversions for replaying recorded RAMMP bags into the RammsUI HMI.

The robot's own path into UE is the RAMMP UI bridge (``rammp-UI-bridge``): a ROS node that turns TF,
``/joint_states`` and perception into RMSS stream metadata and Remote Control payloads. A bag replay
on a machine with no ROS runtime has to make the same conversions without that node, so they live
here, on plain dicts in the shapes a decoded bag message has (see ``TfBuffer.add`` and
``detection_to_ue_object`` for the shapes). Nothing here decodes CDR or touches a bag file; nothing
here talks to UE either.

Frames
------
* ROS: metres, right-handed, x forward / y LEFT / z up (REP 103). A camera's *optical* frame is x
  right / y down / z forward (REP 103 again), its *body* frame (``scene_camera``) is x forward.
* UE: centimetres, left-handed, X forward / Y RIGHT / Z up; an ``FRotator`` is applied roll, then
  pitch, then yaw (intrinsic Z-Y-X), with positive pitch nose-up and positive yaw turning X toward Y.

So a ROS pose becomes a UE one by mirroring Y. On the angles that is: roll kept, pitch and yaw
negated -- positive ROS pitch (right-hand rule about y-left) puts the nose DOWN, positive ROS yaw
turns toward y-left, which is UE's -Y, and positive ROS roll lowers the right side, which is what a
positive UE roll does too. ``ros_to_ue_transform`` is that rule, and the tests check it against
``ramms_tools.transforms.rotation_matrix_from_euler`` (UE's own matrix builder) rather than against
itself.

Provenance: the semantics -- the extrinsic's units and signs, the joint index layout and signs --
are those of the RAMMP UI bridge (``ui_bridge_node.py`` / ``camera_stream_node.cpp``), which is
the contract the robot really sends UE. The bridge is not MIT, so nothing is copied from it: the
Euler extraction is re-derived below from the rotation matrix, and where the bridge has a defect
(the Python node's gimbal branch, the gripper threshold) this module follows the behaviour that is
correct and says so.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Mapping, Sequence

import numpy as np

__all__ = [
    "OPTICAL_TO_BODY",
    "SightingTracker",
    "TfBuffer",
    "camera_extrinsic",
    "detection_to_ue_object",
    "display_joint_targets",
    "normalize_box",
    "optical_to_body",
    "quat_to_euler_zyx_deg",
    "ros_to_ue_transform",
]

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # (x, y, z, w), ROS order
Pose = tuple[Vec3, Quat]

# ── quaternion arithmetic (x, y, z, w) ───────────────────────────────
#
# Plain-float helpers: a lookup composes a handful of edges, and per-element numpy calls on
# 4-vectors cost more than they save. numpy is used where it pays -- the per-edge sample arrays.

_IDENTITY: Pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _vec3(v) -> Vec3:
    """``{x, y, z}`` dict or a 3-sequence -> a float tuple."""
    if isinstance(v, Mapping):
        return (float(v["x"]), float(v["y"]), float(v["z"]))
    x, y, z = v
    return (float(x), float(y), float(z))


def _quat4(q) -> Quat:
    """``{x, y, z, w}`` dict or a 4-sequence (x, y, z, w) -> a float tuple."""
    if isinstance(q, Mapping):
        return (float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"]))
    x, y, z, w = q
    return (float(x), float(y), float(z), float(w))


def _q_normalize(q: Quat) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if not n > 1e-12:  # also catches NaN
        raise ValueError(f"not a rotation: quaternion {q!r} has no length")
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _q_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton product a*b: rotate by b first, then by a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _q_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def _q_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate vector v by unit quaternion q (v' = q v q*), expanded so no temporaries are built."""
    x, y, z, w = q
    # t = 2 (q_vec x v); v' = v + w t + q_vec x t
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def _compose(a: Pose, b: Pose) -> Pose:
    """a*b for poses: maps b's child coordinates through b, then through a."""
    (at, aq), (bt, bq) = a, b
    rt = _q_rotate(aq, bt)
    return ((at[0] + rt[0], at[1] + rt[1], at[2] + rt[2]), _q_mul(aq, bq))


def _invert(p: Pose) -> Pose:
    t, q = p
    qi = _q_conj(q)
    rt = _q_rotate(qi, t)
    return ((-rt[0], -rt[1], -rt[2]), qi)


def _slerp(q0: Quat, q1: Quat, a: float) -> Quat:
    """Spherical interpolation along the SHORT arc: q and -q are the same rotation, and a TF stream
    is free to flip sign between samples, which a naive slerp would turn into a 360-degree spin."""
    dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot
    if dot > 0.9995:
        # Nearly the same rotation: sin(theta) -> 0 makes the slerp weights ill-conditioned, and
        # a normalised lerp is indistinguishable at this separation.
        w0, w1 = 1.0 - a, a
    else:
        theta = math.acos(dot)
        s = math.sin(theta)
        w0, w1 = math.sin((1.0 - a) * theta) / s, math.sin(a * theta) / s
    return _q_normalize(tuple(w0 * c0 + w1 * c1 for c0, c1 in zip(q0, q1)))


# ── Euler angles and the RMSS extrinsic ──────────────────────────────

# Inside this distance of |sin(pitch)| = 1 the roll/yaw split is numerically meaningless; the value
# is the C++ bridge's, so an extrinsic never takes a different branch here than on the robot.
_GIMBAL_EPS = 1e-8


def quat_to_euler_zyx_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """A ROS quaternion as intrinsic Z-Y-X Euler angles ``(roll, pitch, yaw)`` in degrees.

    The rotation is ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`` in ROS's own right-handed frame; these
    are the angles the RAMMP UI bridge feeds its extrinsic (``ros_to_ue_transform`` then flips
    pitch and yaw for UE). Expanding that product, its bottom row is
    ``(-sin p, cos p sin r, cos p cos r)`` and its first column ``(cos y cos p, sin y cos p, -sin p)``,
    so ``sin p`` comes straight from ``R[2][0]`` and roll and yaw from atan2 of the remaining pairs.

    At pitch = +-90 degrees (gimbal lock) ``cos p`` is zero and those pairs vanish; only yaw - roll
    (pitch +90) or yaw + roll (pitch -90) is defined, read from ``R[0][1]`` and ``R[1][1]``. Roll is
    then pinned to 0 and the whole remainder goes to yaw, with the pitch taking the SIGN OF sin p --
    as the bridge's C++ node does. (The bridge's Python node returns the opposite pitch there, a
    different rotation; the C++ one is right, and the tests rebuild the matrix to prove it.)

    The quaternion is normalised first, so a slightly-off-unit input from a float32 producer gives
    the angles of the rotation it means; for a unit quaternion this is a no-op.
    """
    x, y, z, w = _q_normalize((float(x), float(y), float(z), float(w)))
    # The rotation-matrix entries the extraction needs (standard unit-quaternion -> matrix).
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - w * z)
    r10 = 2.0 * (x * y + w * z)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    sin_pitch = -r20
    if abs(sin_pitch) >= 1.0 - _GIMBAL_EPS:
        # At pitch +90, R[0][1] = -sin(yaw - roll) and R[1][1] = cos(yaw - roll); at pitch -90 the
        # same with yaw + roll. With roll := 0 both read yaw = atan2(-R[0][1], R[1][1]).
        pitch = math.copysign(90.0, sin_pitch)
        return 0.0, pitch, math.degrees(math.atan2(-r01, r11))
    return (math.degrees(math.atan2(r21, r22)),
            math.degrees(math.asin(sin_pitch)),
            math.degrees(math.atan2(r10, r00)))


def ros_to_ue_transform(translation, rotation) -> dict:
    """A ROS pose as the RMSS metadata ``"transform"``: ``{"x","y","z"}`` cm, ``{"roll","pitch","yaw"}`` deg.

    ``translation`` is ``{x, y, z}`` or a 3-tuple in metres; ``rotation`` is ``{x, y, z, w}`` or a
    4-tuple (x, y, z, w). Y is mirrored (see the module docstring): ``x*100, -y*100, z*100`` and
    ``roll, -pitch, -yaw`` of ``quat_to_euler_zyx_deg`` -- the bridge's ``tf_to_ue_extrinsics``.
    """
    tx, ty, tz = _vec3(translation)
    roll, pitch, yaw = quat_to_euler_zyx_deg(*_quat4(rotation))
    return {"x": tx * 100.0, "y": -ty * 100.0, "z": tz * 100.0,
            "roll": roll, "pitch": -pitch, "yaw": -yaw}


# The camera BODY frame's orientation expressed in its OPTICAL frame (x, y, z, w). Its columns as a
# matrix are the body axes in optical coordinates: forward = optical +z, left = optical -x,
# up = optical -y. Right-multiplying an optical frame's orientation by it gives the body frame's.
# (Its inverse, (-0.5, 0.5, -0.5, 0.5), is the familiar camera_frame -> optical_frame TF edge every
# RealSense driver publishes -- in this robot's /tf_static too.)
OPTICAL_TO_BODY: Quat = (0.5, -0.5, 0.5, 0.5)


def optical_to_body(rotation) -> Quat:
    """The orientation of a camera's BODY frame in some parent, from its OPTICAL frame's.

    UE wants the body frame (X forward), but detections, images and depth are stamped in the
    optical frame. ``rotation`` is ``{x, y, z, w}`` or a 4-tuple; returns ``(x, y, z, w)``.
    """
    return _q_normalize(_q_mul(_q_normalize(_quat4(rotation)), OPTICAL_TO_BODY))


# ── TF ───────────────────────────────────────────────────────────────

class _StaticEdge:
    __slots__ = ("parent", "pose")

    def __init__(self, parent: str, pose: Pose):
        self.parent, self.pose = parent, pose

    def at(self, stamp_ns: int | None) -> Pose:
        return self.pose


class _DynamicEdge:
    """One /tf edge's samples, as growable numpy arrays sorted by stamp.

    Appends double the capacity when full, so adding a bag's /tf message by message -- interleaved
    with lookups, if the caller streams -- stays amortised O(1) and never copies the history per
    message. A sample older than the newest only marks the arrays unsorted; they are sorted once,
    at the next lookup.
    """

    __slots__ = ("parent", "n", "stamps", "trans", "rots", "sorted")

    def __init__(self, parent: str, capacity: int = 64):
        self.parent = parent
        self.n = 0
        self.stamps = np.empty(capacity, dtype=np.int64)
        self.trans = np.empty((capacity, 3), dtype=np.float64)
        self.rots = np.empty((capacity, 4), dtype=np.float64)
        self.sorted = True

    def append(self, stamp_ns: int, pose: Pose) -> None:
        if self.n == len(self.stamps):
            cap = 2 * len(self.stamps)
            self.stamps = np.resize(self.stamps, cap)
            self.trans = np.resize(self.trans, (cap, 3))
            self.rots = np.resize(self.rots, (cap, 4))
        if self.n and stamp_ns < self.stamps[self.n - 1]:
            self.sorted = False
        self.stamps[self.n] = stamp_ns
        self.trans[self.n] = pose[0]
        self.rots[self.n] = pose[1]
        self.n += 1

    def _ensure_sorted(self) -> None:
        if not self.sorted:
            order = np.argsort(self.stamps[:self.n], kind="stable")  # stable: later adds win ties
            self.stamps[:self.n] = self.stamps[:self.n][order]
            self.trans[:self.n] = self.trans[:self.n][order]
            self.rots[:self.n] = self.rots[:self.n][order]
            self.sorted = True

    def _sample(self, i: int) -> Pose:
        return tuple(self.trans[i].tolist()), tuple(self.rots[i].tolist())

    def at(self, stamp_ns: int | None) -> Pose:
        self._ensure_sorted()
        n = self.n
        if stamp_ns is None:
            return self._sample(n - 1)
        # Index of the first sample AFTER stamp: the bracket is [i-1, i]. Outside the recorded
        # range, clamp to the end sample rather than extrapolate -- a joint held still at the end
        # of a bag is the best guess, a velocity projected off its last two samples is not.
        i = int(np.searchsorted(self.stamps[:n], stamp_ns, side="right"))
        if i == 0:
            return self._sample(0)
        if i == n:
            return self._sample(n - 1)
        s0, s1 = int(self.stamps[i - 1]), int(self.stamps[i])
        (t0, q0), (t1, q1) = self._sample(i - 1), self._sample(i)
        a = (stamp_ns - s0) / (s1 - s0)  # s1 > s0: side="right" puts equal stamps left of i
        return (tuple(c0 + a * (c1 - c0) for c0, c1 in zip(t0, t1)), _slerp(q0, q1, a))


class TfBuffer:
    """A ROS TF tree built from decoded ``TFMessage`` dicts, queried at any time in the bag.

    ``add`` takes a message in the decoded shape::

        {"transforms": [{"header": {"stamp_ns": int, "frame_id": parent},
                         "child_frame_id": child,
                         "transform": {"translation": {x, y, z}, "rotation": {x, y, z, w}}}]}

    Each child frame has exactly one parent (TF is a tree); a child re-sent under a different
    parent, or switching between /tf and /tf_static, replaces its edge. Static edges have no time:
    a bag's /tf_static is typically stamped when the robot booted, hours before the recording, and
    must still apply at every instant of it. Dynamic edges keep every sample and are interpolated.
    """

    def __init__(self) -> None:
        self._edges: dict[str, _StaticEdge | _DynamicEdge] = {}  # by child frame
        self._frames: set[str] = set()

    def add(self, msg: dict, static: bool) -> None:
        """Add a decoded TFMessage: ``static=True`` for /tf_static, ``False`` for /tf."""
        for ts in msg.get("transforms", ()):
            parent = ts["header"]["frame_id"]
            child = ts["child_frame_id"]
            tf = ts["transform"]
            pose = (_vec3(tf["translation"]), _q_normalize(_quat4(tf["rotation"])))
            self._frames.update((parent, child))
            edge = self._edges.get(child)
            if static:
                self._edges[child] = _StaticEdge(parent, pose)
                continue
            if not isinstance(edge, _DynamicEdge) or edge.parent != parent:
                edge = self._edges[child] = _DynamicEdge(parent)
            edge.append(int(ts["header"]["stamp_ns"]), pose)

    def frames(self) -> set[str]:
        """Every frame named in the tree, as a parent or a child."""
        return set(self._frames)

    def _ancestry(self, frame: str) -> list[str]:
        """``[frame, parent, grandparent, ..., root]``."""
        chain = [frame]
        seen = {frame}
        edge = self._edges.get(frame)
        while edge is not None:
            if edge.parent in seen:
                raise LookupError(f"TF tree has a cycle through {edge.parent!r}")
            chain.append(edge.parent)
            seen.add(edge.parent)
            edge = self._edges.get(edge.parent)
        return chain

    def _down(self, chain: list[str], depth: int, stamp_ns: int | None) -> Pose:
        """The pose of ``chain[0]`` in ``chain[depth]``: the edges composed root-side first."""
        pose = _IDENTITY
        for k in range(depth - 1, -1, -1):
            pose = _compose(pose, self._edges[chain[k]].at(stamp_ns))
        return pose

    def lookup(self, parent: str, child: str, stamp_ns: int | None = None) -> Pose:
        """The pose of ``child`` in ``parent``: ``((x, y, z), (qx, qy, qz, qw))``, metres.

        It maps child coordinates into parent ones (``p_parent = R p_child + t``), which is
        tf2's ``lookup_transform(target=parent, source=child)``. The path runs through the two
        frames' nearest common ancestor, inverting the edges on the parent's side. ``stamp_ns``
        picks the instant for dynamic edges (interpolated between the bracketing samples, clamped
        outside them); ``None`` takes each dynamic edge's newest sample. Raises ``LookupError``,
        naming both frames, when they are not in one tree.
        """
        missing = [f for f in (parent, child) if f not in self._frames]
        if missing:
            raise LookupError(f"no TF from {parent!r} to {child!r}: unknown frame(s) "
                              f"{', '.join(repr(m) for m in missing)}")
        if parent == child:
            return _IDENTITY
        up_child, up_parent = self._ancestry(child), self._ancestry(parent)
        depth_in_parent = {f: i for i, f in enumerate(up_parent)}
        for depth_c, frame in enumerate(up_child):
            if frame in depth_in_parent:
                depth_p = depth_in_parent[frame]
                break
        else:
            raise LookupError(f"no TF from {parent!r} to {child!r}: they are in separate trees "
                              f"(rooted at {up_parent[-1]!r} and {up_child[-1]!r})")
        ancestor_to_child = self._down(up_child, depth_c, stamp_ns)
        ancestor_to_parent = self._down(up_parent, depth_p, stamp_ns)
        return _compose(_invert(ancestor_to_parent), ancestor_to_child)


def camera_extrinsic(tf: TfBuffer, base_frame: str, optical_frame: str, stamp_ns: int | None = None) -> dict:
    """The RMSS ``"transform"`` of a camera: its BODY pose in ``base_frame``, in UE units.

    Takes the image's own optical ``frame_id`` and turns it to the body frame here, rather than
    looking up a ``<camera>`` link by name the way the bridge's C++ node does: a driver is free to
    put the colour sensor off the link (RealSense publishes ``camera_link -> color_frame`` with a
    real offset), and the image is where it is. On this robot the two are the same pose.
    """
    t, q = tf.lookup(base_frame, optical_frame, stamp_ns)
    return ros_to_ue_transform(t, optical_to_body(q))


# ── joints ───────────────────────────────────────────────────────────

_GRIPPER_JOINT = "arm_robotiq_85_left_knuckle_joint"

# arm_joint_N -> the RAMMP display's link_{N-1}, with the sign the display skeleton needs: it
# authors joints 1, 3, 5, 6 and 7 mirrored from the Kinova driver (the bridge's Gen1 array negates
# those indices; verified on the live display, per the bridge's notes).
_ARM_DISPLAY = {f"arm_joint_{n}": (f"link_{n - 1}", sign)
                for n, sign in zip(range(1, 8), (-1.0, 1.0, -1.0, 1.0, -1.0, -1.0, -1.0))}

# Each finger's bone swings 45 degrees between open and closed, mirrored left/right.
_FINGER_OPEN_DEG = 45.0


def display_joint_targets(names, positions, gripper_closed_rad: float = 0.793) -> dict[str, float]:
    """A ``/joint_states`` message as RAMMP UE display joint targets, by name, in degrees.

    ``arm_joint_1..7`` (radians) -> ``link_0..link_6``, signs (-, +, -, +, -, -, -). The gripper's
    one driven joint, ``arm_robotiq_85_left_knuckle_joint`` (0 = open, ``gripper_closed_rad`` =
    closed), sets both finger bones in proportion: with ``f = clamp(q / gripper_closed_rad, 0, 1)``,
    ``link_0_l = -45 (1 - f)`` and ``link_0_r = +45 (1 - f)``. Proportional rather than the
    bridge's open/closed switch at ``q > 0.9``, which never fires on this robot: its knuckle tops
    out at 0.793 rad, so the bridge always shows the gripper open.

    Only joints present in ``names`` appear in the result, so it can be merged over the display's
    current targets without zeroing anything else; unknown names and NaN (or infinite) positions --
    a driver's "not measured" -- are skipped.
    """
    if not gripper_closed_rad > 0.0:
        raise ValueError(f"gripper_closed_rad must be positive, got {gripper_closed_rad!r}")
    names, positions = list(names), list(positions)
    if not positions:
        return {}  # JointState allows an empty position array (a velocity-only message)
    if len(names) != len(positions):
        raise ValueError(f"/joint_states has {len(names)} names but {len(positions)} positions")
    out: dict[str, float] = {}
    for name, q in zip(names, positions):
        q = float(q)
        if not math.isfinite(q):
            continue
        arm = _ARM_DISPLAY.get(name)
        if arm is not None:
            out[arm[0]] = arm[1] * math.degrees(q)
        elif name == _GRIPPER_JOINT:
            opening = 1.0 - min(max(q / gripper_closed_rad, 0.0), 1.0)
            # "+ 0.0" turns the -0.0 of a fully closed left finger into 0.0 on the wire.
            out["link_0_l"] = -_FINGER_OPEN_DEG * opening + 0.0
            out["link_0_r"] = _FINGER_OPEN_DEG * opening
    return out


# ── detections ───────────────────────────────────────────────────────

def normalize_box(box, width: int, height: int) -> list[float]:
    """A pixel box ``[x0, y0, x1, y1]`` as fractions of the image, ``[x0/w, y0/h, x1/w, y1/h]``.

    Clamped to [0, 1]: RammsUI reads a box as pixels as soon as ANY value exceeds 1.0
    (``RammsBridgeSubsystem.h``), so a segmenter box that overhangs the frame edge by a fraction
    of a pixel would otherwise be divided by the frame size a second time and shrink to a speck.
    """
    if not (width > 0 and height > 0):
        raise ValueError(f"image size must be positive, got {width}x{height}")
    x0, y0, x1, y1 = (float(v) for v in box)
    return [min(max(v, 0.0), 1.0) for v in (x0 / width, y0 / height, x1 / width, y1 / height)]


def _r6(v: float) -> float:
    return round(float(v), 6)


def detection_to_ue_object(obj: dict, width: int, height: int, source: str,
                           time_since_s: float | None = None, with_3d: bool = True) -> dict:
    """A decoded ``DetectedObject`` as one ``UpdateDetectedObjects`` object (RammsUI schema).

    ``obj`` is the decoded shape: ``instance_id``, ``perception_class``, ``state`` (0 visible,
    1 occluded), ``box`` ``[x0, y0, x1, y1]`` in source pixels (all zero when occluded),
    ``has_pose``, ``pose`` ``{"position": {x,y,z}, "orientation": {x,y,z,w}}`` in the detection
    header's frame (the colour OPTICAL frame), ``size`` ``{x,y,z}`` full extents in metres.
    ``width``/``height`` are the source image's, ``source`` the stream id the object belongs to
    (``"camera/scene/color"``).

    The box goes out normalised and the centroid with it, both as arrays -- the form the
    producer's own ``/perception/odom3d/ui_payload`` uses and every RammsUI reader accepts. An
    occluded object keeps a zero box and gets no centroid, as the producer does: a ``[0, 0]``
    centroid would put a POI in the image corner. With ``with_3d`` and a pose, the camera-frame
    position, orientation and FULL size go too (``frame: "camera"``, which UE reads as the optical
    frame of ``source``). ``time_since_s`` becomes ``time_since_detection_s``; leave it ``None``
    when unknown -- absent means unknown to UE, while 0 would claim the robot is seeing it now.
    """
    box = obj.get("box")
    occluded = obj.get("state") == 1 or box is None or not any(float(v) for v in box)
    out: dict = {"instance_id": obj["instance_id"], "perception_class": obj["perception_class"],
                 "source": source}
    if occluded:
        out["box"] = [0.0, 0.0, 0.0, 0.0]
    else:
        x0, y0, x1, y1 = normalize_box(box, width, height)
        out["box"] = [_r6(x0), _r6(y0), _r6(x1), _r6(y1)]
        out["centroid"] = [_r6((x0 + x1) / 2.0), _r6((y0 + y1) / 2.0)]
    if with_3d and obj.get("has_pose"):
        pose = obj["pose"]
        px, py, pz = _vec3(pose["position"])
        qx, qy, qz, qw = _quat4(pose["orientation"])
        sx, sy, sz = _vec3(obj["size"])
        out["position_m"] = {"X": _r6(px), "Y": _r6(py), "Z": _r6(pz)}
        out["frame"] = "camera"
        out["orientation"] = {"X": _r6(qx), "Y": _r6(qy), "Z": _r6(qz), "W": _r6(qw)}
        out["size_m"] = {"X": _r6(sx), "Y": _r6(sy), "Z": _r6(sz)}
    if time_since_s is not None:
        out["time_since_detection_s"] = _r6(time_since_s)
    return out


def _box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    ix = min(a[2], b[2]) - max(a[0], b[0])
    iy = min(a[3], b[3]) - max(a[1], b[1])
    if ix <= 0.0 or iy <= 0.0:
        return 0.0
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0.0 else 0.0


class SightingTracker:
    """``time_since_detection_s`` for tracked objects: how long since the robot last SAW each one.

    RammsUI's contract for that field is "since the robot last saw the object directly; a tracker
    carrying it forward does not reset it". On RAMMP the direct sightings are the 5 Hz segmenter's
    ``/perception/object_detections_raw`` -- whose ids are per-frame, so they cannot be matched by
    id -- and the 15 Hz tracker's ``/perception/object_detections`` carries persistent ids between
    them. A tracked object counts as SIGHTED at its message stamp S when a raw message stamped
    within ``stamp_tolerance_ns`` of S (the two are stamped from the same image; the tolerance
    absorbs float rounding of the stamps) holds an object whose box overlaps it with IoU above
    ``iou``. Of any class: the segmenter calls the same mask "food container" in one frame and
    "drink cup" in the next, and requiring the classes to agree marked objects the robot was looking
    at as stale (on the Nov 19 vo bag, 20 visible objects dimmed for want of a matching label). The tracker's own ``state`` is not used: it stays
    "visible" while the tracker predicts between segmenter frames, which is exactly the
    carrying-forward the contract excludes.

    ``raw`` and ``tracked`` are ``[(stamp_ns, objects), ...]`` with ``objects`` in the decoded
    ``DetectedObject`` shape. Everything is computed up front, so ``time_since`` is a pure function
    of time and id and a seeking player can ask about any instant in any order.
    """

    def __init__(self, raw: list[tuple[int, list[dict]]], tracked: list[tuple[int, list[dict]]],
                 iou: float = 0.3, stamp_tolerance_ns: int = 1000):
        raw_sorted = sorted(((int(s), objs) for s, objs in raw), key=lambda r: r[0])
        raw_stamps = [s for s, _ in raw_sorted]
        sightings: dict[str, list[int]] = {}
        for stamp, objects in tracked:
            stamp = int(stamp)
            lo = bisect.bisect_left(raw_stamps, stamp - stamp_tolerance_ns)
            hi = bisect.bisect_right(raw_stamps, stamp + stamp_tolerance_ns)
            if lo == hi:
                continue  # a tracker-only frame: nothing was seen directly at this instant
            seen = [[float(v) for v in r["box"]] for _, objs in raw_sorted[lo:hi] for r in objs]
            for obj in objects:
                box = [float(v) for v in obj["box"]]
                if not any(box):
                    continue  # occluded: no box to see
                if any(_box_iou(box, rb) > iou for rb in seen):
                    sightings.setdefault(obj["instance_id"], []).append(stamp)
        self._sightings = {iid: np.unique(np.asarray(stamps, dtype=np.int64))
                           for iid, stamps in sightings.items()}

    def time_since(self, stamp_ns: int, instance_id: str) -> float | None:
        """Seconds from ``instance_id``'s last sighting at or before ``stamp_ns`` to ``stamp_ns``;
        ``None`` if it has not been sighted by then (unknown, which is not the same as stale)."""
        stamps = self._sightings.get(instance_id)
        if stamps is None:
            return None
        i = int(np.searchsorted(stamps, int(stamp_ns), side="right"))
        if i == 0:
            return None
        return (int(stamp_ns) - int(stamps[i - 1])) / 1e9
