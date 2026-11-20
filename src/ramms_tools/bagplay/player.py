"""Turn a recorded RAMMP bag into what RammpUI receives from the robot, on the bag's own clock.

What goes where (RAMMP Gen 1.5 topic names, RammsUI's default stream ids):

  /scene_camera/color/image_raw       -> RMSS ch 0 camera/scene/color   (+ intrinsics, pose from TF)
  /scene_camera/depth/image_rect_raw  -> RMSS ch 1 camera/scene/depth   (16UC1 mm; + intrinsics, pose)
  /perception/object_detections       -> UpdateDetectedObjects: 2D boxes AND 3D markers
  /joint_states                       -> SetAllJointTargets on the robot display, by joint name
  the overlay camera slot, one of (--overlay):
    wrist         /wrist_camera/color, /depth   -> ch 2 engine/overlay, ch 4 engine/overlay/depth
    segmentation  the scene frame + /perception/object_detections_raw label_image
                                                -> ch 2 engine/overlay, ch 3 engine/overlay/segmentation
    segmenter     /perception/scene_segmenter/overlay/compressed (needs Pillow) -> ch 2 engine/overlay

Both camera streams carry their intrinsics and the camera's pose. The UI bridge sends them on the depth
frames only; RammsUI takes them from either, and on both the colour stream gets a projector of its own.
The pose is the camera's body frame in base_link, sent "relative", so the UE stream receiver -- not the
world origin -- stands for the robot's base.

Staleness (time_since_detection_s) is counted from the last time the 5 Hz segmenter actually saw an
object, not from the tracker's state: a tracker carrying an object forward is not a sighting (the
RammsDetectionWorldMarkers contract), and that rule matches the producer's own values (~90%).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from ramms_tools.ros_ue import (
    SightingTracker,
    TfBuffer,
    camera_extrinsic,
    detection_to_ue_object,
    display_joint_targets,
)
from ramms_tools.rosbag import open_bag

SCENE_COLOR = "/scene_camera/color/image_raw"
SCENE_DEPTH = "/scene_camera/depth/image_rect_raw"
SCENE_INFO = "/scene_camera/color/camera_info"
SCENE_DEPTH_INFO = "/scene_camera/depth/camera_info"
WRIST_COLOR = "/wrist_camera/color/image_raw"
WRIST_DEPTH = "/wrist_camera/depth/image_rect_raw"
WRIST_INFO = "/wrist_camera/color/camera_info"
WRIST_DEPTH_INFO = "/wrist_camera/depth/camera_info"
TRACKED = "/perception/object_detections"
RAW = "/perception/object_detections_raw"
SEGMENTER_JPEG = "/perception/scene_segmenter/overlay/compressed"
JOINTS = "/joint_states"
TF = "/tf"
TF_STATIC = "/tf_static"
BASE_FRAME = "base_link"
ODOM_FRAME = "odom"

OVERLAY_MODES = ("auto", "wrist", "segmentation", "segmenter", "none")

# Video later than this is dropped rather than sent: UE coalesces to one frame per tick anyway, and a
# replay that falls behind should catch up, not play back slowly. Detections and joints are never
# dropped (the newest of each is what counts, and the sinks keep only the newest).
LATE_DROP_S = 0.1
MIN_LOOP_PASS_S = 0.05   # --loop: no pass is shorter, whatever its window holds


@dataclass
class StreamIds:
    """RammsUI's stream ids for the two camera sets (RammsCameraStreamSettings defaults)."""
    scene_color: str = "camera/scene/color"
    scene_depth: str = "camera/scene/depth"
    overlay_color: str = "engine/overlay"
    overlay_depth: str = "engine/overlay/depth"
    overlay_segmentation: str = "engine/overlay/segmentation"


# One channel per stream: the UE sink keeps only the latest message per (channel, type) per tick.
CHANNELS = {"scene_color": 0, "scene_depth": 1, "overlay_color": 2, "overlay_segmentation": 3, "overlay_depth": 4}
GROUPS = {"scene": "camera/scene", "overlay": "overlay"}


@dataclass
class PlayOptions:
    rate: float = 1.0               # playback speed; 0 = as fast as possible
    start_s: float = 0.0            # from the bag's start
    end_s: Optional[float] = None
    loop: bool = False
    overlay: str = "auto"           # see OVERLAY_MODES
    positions: str = "sent"         # "sent": the tracker's 3D poses; "derived": 2D only, UE unprojects through depth
    objects: bool = True
    joints: bool = True
    joints_hz: float = 10.0
    gripper_closed_rad: float = 0.793


@dataclass
class PlayStats:
    sent: dict = field(default_factory=dict)        # what -> count
    late_dropped: int = 0
    passes: int = 0
    last_3d_payload: Optional[dict] = None          # the newest push carrying a 3D object, with its extrinsic
    last_scene_depth_ns: Optional[int] = None       # the stamp of the last scene depth frame sent: what UE derives from
    last_joint_values: Optional[dict] = None
    warnings: list = field(default_factory=list)

    def count(self, what, n=1):
        self.sent[what] = self.sent.get(what, 0) + n


def intrinsics_of(info):
    """RMSS metadata intrinsics {fx, fy, cx, cy} from a CameraInfo's K."""
    k = info["k"]
    return {"fx": float(k[0]), "fy": float(k[4]), "cx": float(k[2]), "cy": float(k[5])}


def packed(image, bytes_per_pixel):
    """An Image's pixels as tightly packed rows (step may carry padding)."""
    data, w, h, step = image["data"], image["width"], image["height"], image["step"]
    row = w * bytes_per_pixel
    if step == row:
        return bytes(data[:row * h])
    return b"".join(bytes(data[y * step:y * step + row]) for y in range(h))


class BagSession:
    """A bag opened for playback, with what has to be known before the first frame.

    Preloaded (small): the TF tree, each camera's intrinsics, and every detection's objects (without the
    label images) -- the staleness of an object depends on sightings before it, so it is worked out up
    front and stays a pure function of time.
    """

    def __init__(self, path, log=print):
        self.path = path
        self.reader = open_bag(path)
        self.topics = self.reader.topics
        self.log = log
        self.tf = TfBuffer()
        for topic, static in ((TF_STATIC, True), (TF, False)):
            if topic in self.topics:
                for raw in self.reader.messages(topics=[topic]):
                    self.tf.add(self.reader.decode(raw), static=static)
        self.info = {t: self.reader.first(t) for t in (SCENE_INFO, SCENE_DEPTH_INFO, WRIST_INFO, WRIST_DEPTH_INFO)
                     if self.has(t)}
        tracked = self._objects(TRACKED)
        raw = self._objects(RAW)
        self.tracker = SightingTracker(raw, tracked) if tracked else None
        self.tracked_ids = sorted({o["instance_id"] for _, objs in tracked for o in objs})
        # Classes get fixed mask values for the whole bag, so a class keeps its colour from frame to
        # frame: the segmenter's own mask ids are per frame (fridge_1 now, fridge_2 a moment later).
        self.classes = sorted({o["perception_class"] for _, objs in raw for o in objs})
        self.class_value = {c: i + 1 for i, c in enumerate(self.classes)}
        self.scene_size = None
        if self.has(SCENE_COLOR):
            first = self.reader.decode(next(self.reader.messages(topics=[SCENE_COLOR])), keep_data=False)
            self.scene_size = (first["width"], first["height"])
        self.has_wrist = self.has(WRIST_COLOR)
        self.has_segmentation = self.has(RAW)
        self.has_segmenter_jpeg = self.has(SEGMENTER_JPEG)

    def has(self, topic):
        """Messages on the topic. A recorder lists a topic it subscribed to even when nothing was published."""
        t = self.topics.get(topic)
        return bool(t and t.count)

    def _objects(self, topic):
        if not self.has(topic):
            return []
        return [(m["header"]["stamp_ns"], m["objects"])
                for m in (self.reader.decode(r, keep_data=False) for r in self.reader.messages(topics=[topic]))]

    @property
    def start_ns(self):
        return self.reader.start_ns

    @property
    def end_ns(self):
        return self.reader.end_ns

    def frozen_3d(self):
        """Whether the tracked 3D poses can be trusted to follow the robot.

        The tracker keeps objects in odom; with no odom->base_link in /tf (the Nov 19 main bag has none,
        and its /base/odom is a stale burst) every object stays where the robot was when the bag began,
        while the camera drives on.
        """
        if not self.tracked_ids:
            return False
        try:
            self.tf.lookup(ODOM_FRAME, BASE_FRAME)
        except LookupError:
            return True
        return False

    def overlay_mode(self, requested):
        if requested != "auto":
            return requested
        if self.has_wrist:
            return "wrist"
        if self.has_segmentation:
            return "segmentation"
        return "none"

    def depth_at(self, stamp_ns, topic=SCENE_DEPTH, tolerance_ns=1_000_000):
        """The depth frame (uint16 mm, h x w) stamped within tolerance of stamp_ns, or None."""
        import numpy as np
        if topic not in self.topics:
            return None
        for raw in self.reader.messages(topics=[topic], start_ns=stamp_ns - 1_000_000_000,
                                        end_ns=stamp_ns + 1_000_000_000):
            m = self.reader.decode(raw)
            if abs(m["header"]["stamp_ns"] - stamp_ns) <= tolerance_ns:
                return np.frombuffer(packed(m, 2), "<u2").reshape(m["height"], m["width"])
        return None

    def close(self):
        self.reader.close()


class Player:
    """Send a bag's messages to `sinks` on the bag's clock, scaled by options.rate."""

    def __init__(self, session, sinks, options=None, ids=None, clock=time.monotonic, sleep=time.sleep, log=print):
        self.session = session
        self.sinks = sinks
        self.options = options or PlayOptions()
        self.ids = ids or StreamIds()
        self.clock, self.sleep, self.log = clock, sleep, log
        self.stats = PlayStats()
        self.overlay = session.overlay_mode(self.options.overlay)
        self._scene_frames = {}         # stamp -> scene colour image, for the segmentation overlay
        self._overlay_objects = []      # the segmenter's latest boxes, shown on the overlay in segmentation mode
        self._overlay_depth_cleared = False
        # The segmenter's boxes go out only once the overlay has settled: from the scene frame AFTER the first
        # overlay copy and its depth clear. The push travels over HTTP while those frames are still crossing
        # RMSS, and a push that beats them is resolved against the previous replay's overlay (the wrist's
        # colour and depth): phantom 3D markers for the boxes, or a "not registered to colour" warning.
        self._overlay_settled = None    # None: no overlay copy yet; False: sent, waiting a frame; True
        # Cameras ("scene", "overlay") whose colour frame has gone out. Nothing that leans on a camera goes
        # before it: RammsUI checks a push's objects and a depth frame against the colour stream as it stands,
        # and warns ONCE per stream -- about a pose a push arrived too early for, or about depth not
        # registered to the colour another replay left on the overlay -- then stays silent for good.
        self._colour_sent = set()
        self._colour_unsendable = set()   # cameras whose colour encoding cannot be sent: nothing to wait for
        self._last_joints_ns = None
        self._warned = set()

    # ── what is played ───────────────────────────────────────────

    def topics(self):
        o, wanted = self.options, [SCENE_COLOR, SCENE_DEPTH]
        if o.objects:
            wanted.append(TRACKED)
        if o.joints:
            wanted.append(JOINTS)
        wanted += {"wrist": [WRIST_COLOR, WRIST_DEPTH], "segmentation": [RAW], "segmenter": [SEGMENTER_JPEG]}.get(
            self.overlay, [])
        return [t for t in wanted if t in self.session.topics]

    def warn(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.stats.warnings.append(text)
            self.log(f"  warning: {text}")

    # ── playing ──────────────────────────────────────────────────

    def run(self, close=True):
        """Play, then (close=True) leave the UI as it was. close=False leaves that to the caller -- to verify
        what landed before the cleanup clears it."""
        s, o = self.session, self.options
        if o.overlay == "wrist" and not s.has_wrist:
            raise ValueError("this bag has no wrist camera (the _vo bag dropped it)")
        if o.overlay in ("segmentation", "segmenter") and not (s.has_segmentation if o.overlay == "segmentation"
                                                               else s.has_segmenter_jpeg):
            raise ValueError(f"this bag has nothing for --overlay {o.overlay}")
        if o.objects and o.positions == "sent" and s.frozen_3d():
            self.warn("frozen", "this bag has no odom->base_link in /tf, so the tracker's 3D poses are frozen where "
                                "the robot began while the camera moves on; the markers will not follow the objects "
                                "(use the _vo bag, or --positions derived)")
        start = s.start_ns + int(o.start_s * 1e9)
        end = s.end_ns + 1 if o.end_s is None else s.start_ns + int(o.end_s * 1e9)
        if start >= min(end, s.end_ns + 1):
            raise ValueError(f"--start {o.start_s:g} / --end {o.end_s} select no part of this "
                             f"{s.reader.duration_s:.1f} s bag")
        topics = self.topics()
        try:
            if self.overlay != "none":
                self.sinks.feed(True)
            while True:
                played = self._pass(topics, start, end)
                if not played:
                    raise ValueError("the --start/--end window holds none of the played topics' messages")
                self.stats.passes += 1
                if not o.loop:
                    break
        finally:
            if close:
                self.sinks.close()
        return self.stats

    def _pass(self, topics, start, end):
        """One pass over [start, end); the number of messages it played."""
        rate = self.options.rate
        b0, w0 = start, self.clock()
        self._last_joints_ns = None   # each pass starts the joint throttle again (--loop)
        played = 0
        for raw in self.session.reader.messages(topics=topics, start_ns=start, end_ns=end):
            played += 1
            if rate > 0:
                due = w0 + (raw.timestamp_ns - b0) / 1e9 / rate
                now = self.clock()
                if due > now:
                    self.sleep(due - now)
                elif now - due > LATE_DROP_S and raw.topic not in (TRACKED, JOINTS) \
                        and not self._first_colour(raw.topic):
                    self.stats.late_dropped += 1
                    continue
            self.dispatch(raw)
        if self.options.loop and played:
            # A looping pass lasts its whole window, not just until its last message: a window whose
            # messages all sit near its start otherwise looped as fast as the CPU allowed, flooding the UI.
            span = (min(end, self.session.end_ns + 1) - start) / 1e9 / rate if rate > 0 else 0.0
            due = w0 + max(span, MIN_LOOP_PASS_S)
            now = self.clock()
            if due > now:
                self.sleep(due - now)
        return played

    def _first_colour(self, topic):
        """A camera's first colour frame is never dropped as late: objects and its depth wait for it."""
        group = {SCENE_COLOR: "scene", WRIST_COLOR: "overlay"}.get(topic)
        return group is not None and not self._colour_done(group)

    def _colour_done(self, group):
        """The camera's colour frame went out, or never can: what waits for it may go."""
        return group in self._colour_sent or group in self._colour_unsendable

    def dispatch(self, raw):
        t = raw.topic
        if t == SCENE_COLOR:
            image = self.session.reader.decode(raw)
            if self.overlay == "segmentation":
                self._remember_scene_frame(image)
                if self._overlay_settled is False:
                    self._overlay_settled = True
            self._camera(image, "scene_color", self.ids.scene_color, "scene", SCENE_INFO, "color")
        elif t == SCENE_DEPTH:
            self._camera(self.session.reader.decode(raw), "scene_depth", self.ids.scene_depth, "scene",
                         SCENE_DEPTH_INFO, "depth")
        elif t == WRIST_COLOR:
            self._camera(self.session.reader.decode(raw), "overlay_color", self.ids.overlay_color, "overlay",
                         WRIST_INFO, "color")
        elif t == WRIST_DEPTH:
            self._camera(self.session.reader.decode(raw), "overlay_depth", self.ids.overlay_depth, "overlay",
                         WRIST_DEPTH_INFO, "depth")
        elif t == RAW:
            self._segmentation(self.session.reader.decode(raw))
        elif t == SEGMENTER_JPEG:
            self._segmenter_jpeg(self.session.reader.decode(raw))
        elif t == TRACKED:
            self._objects(self.session.reader.decode(raw, keep_data=False))
        elif t == JOINTS:
            self._joints(raw)

    # ── cameras ──────────────────────────────────────────────────

    def camera_metadata(self, image, info_topic, with_pose=True):
        """Intrinsics from the camera's CameraInfo, and its body pose in base_link from TF at the frame's stamp."""
        meta = {}
        info = self.session.info.get(info_topic)
        if info:
            if (info["width"], info["height"]) != (image["width"], image["height"]):
                self.warn(f"size:{info_topic}", f"{info_topic} is {info['width']}x{info['height']} but the images are "
                                                f"{image['width']}x{image['height']}; sending no intrinsics")
            else:
                meta["intrinsics"] = intrinsics_of(info)
        if with_pose:
            frame = image["header"]["frame_id"]
            try:
                meta["transform"] = camera_extrinsic(self.session.tf, BASE_FRAME, frame, image["header"]["stamp_ns"])
                meta["transform_space"] = "relative"
            except LookupError as exc:
                self.warn(f"tf:{frame}", f"no pose for {frame} in {BASE_FRAME} ({exc}); its frames go without one")
        return meta

    def _camera(self, image, slot, stream_id, group, info_topic, role):
        enc = image["encoding"].lower()
        w, h = image["width"], image["height"]
        if role == "depth" and not self._colour_done(group):
            return  # its colour first (the start of a window can open between the two)
        meta = self.camera_metadata(image, info_topic)
        if role == "depth":
            if enc not in ("16uc1", "mono16"):
                self.warn(f"enc:{info_topic}", f"depth encoding {image['encoding']} is not sent (16UC1 mm only)")
                return
            self.sinks.video(channel=CHANNELS[slot], kind="depth", stream_id=stream_id, group=GROUPS[group], width=w,
                             height=h, data=packed(image, 2), fmt="16uc1", metadata=meta)
            if slot == "scene_depth":
                self.stats.last_scene_depth_ns = image["header"]["stamp_ns"]
        else:
            bpp = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}.get(enc)
            if bpp is None:
                self._colour_unsendable.add(group)
                self.warn(f"enc:{stream_id}", f"colour encoding {image['encoding']} is not sent; this camera's depth "
                                              f"and the objects go out without it")
                return
            self.sinks.video(channel=CHANNELS[slot], kind="color", stream_id=stream_id, group=GROUPS[group], width=w,
                             height=h, data=packed(image, bpp), fmt=enc, metadata=meta)
            self._colour_sent.add(group)
        self.stats.count(slot)

    # ── the overlay's segmentation ───────────────────────────────

    def _clear_overlay_depth(self, scene_frame):
        """In segmentation mode the overlay shows the scene frame and has no depth of its own: say so, with one
        frame of no returns. Otherwise the depth an earlier replay left on the overlay's channel (the wrist
        camera's, with the wrist's pose) stays the overlay's, and RammsUI unprojects every one of the
        segmenter's 2D boxes through it: dozens of phantom 3D markers, from another camera's depth at another
        camera's pose. It carries the scene camera's intrinsics and pose, as a real camera that saw nothing
        would: with no geometry RammsUI warns that the overlay is missing camera metadata, and with the
        wrist's left over that its depth is not registered to its colour. Depth streams get no projector of
        their own, so the pose places nothing."""
        w, h = scene_frame["width"], scene_frame["height"]
        self.sinks.video(channel=CHANNELS["overlay_depth"], kind="depth", stream_id=self.ids.overlay_depth,
                         group=GROUPS["overlay"], width=w, height=h, data=bytes(w * h * 2), fmt="16uc1",
                         metadata=self.camera_metadata(scene_frame, SCENE_INFO))
        self._overlay_depth_cleared = True

    def _remember_scene_frame(self, image):
        self._scene_frames[image["header"]["stamp_ns"]] = image
        while len(self._scene_frames) > 8:
            self._scene_frames.pop(next(iter(self._scene_frames)))

    def _segmentation(self, msg):
        """The scene frame the segmenter ran on, and its masks as one fixed value per class."""
        import numpy as np
        label = msg["label_image"]
        if not label.get("width") or not label.get("data"):
            return
        stamp = msg["header"]["stamp_ns"]
        frame = next((img for s, img in self._scene_frames.items() if abs(s - stamp) <= 1_000_000), None)
        if frame is None:
            self.warn("seg:frame", "a segmentation arrived with no scene frame of its stamp; its overlay is skipped")
            return
        w, h = label["width"], label["height"]
        dtype = {"mono16": "u2", "16uc1": "u2", "mono8": "u1", "8uc1": "u1"}.get(label["encoding"].lower())
        if dtype is None:
            self.warn("seg:enc", f"segmentation label encoding {label['encoding']} is not handled; no overlay")
            return
        order = ">" if label.get("is_bigendian") else "<"
        ids = np.frombuffer(packed(label, np.dtype(dtype).itemsize), dtype=order + dtype).reshape(h, w)
        lut = np.zeros(int(ids.max()) + 1, dtype=np.uint8)
        for o in msg["objects"]:
            if 0 < o["mask_id"] < len(lut):
                lut[o["mask_id"]] = self.session.class_value.get(o["perception_class"], 0)
        mask = lut[ids]
        # No pose on the overlay copy: the projection manager would give it a projector of its own, a
        # second one on the scene camera's pose.
        meta_frame = self.camera_metadata(frame, SCENE_INFO, with_pose=False)
        enc = frame["encoding"].lower()
        bpp = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}.get(enc)
        if bpp is None:
            self.warn("seg:frame-enc", f"scene encoding {frame['encoding']} cannot back the overlay")
            return
        self.sinks.video(channel=CHANNELS["overlay_color"], kind="color", stream_id=self.ids.overlay_color,
                         group=GROUPS["overlay"], width=frame["width"], height=frame["height"],
                         data=packed(frame, bpp), fmt=enc, metadata=meta_frame)
        self._colour_sent.add("overlay")
        if not self._overlay_depth_cleared:
            self._clear_overlay_depth(frame)   # after the colour it belongs to, before any segmenter box
            self._overlay_settled = False
        # The segmenter's boxes, for the overlay window: 2D only (it has no 3D), and its ids prefixed --
        # they are per frame (fridge_1 now, fridge_2 next) and would otherwise collide with the tracker's.
        self._overlay_objects = [
            dict(detection_to_ue_object(o, w, h, source=self.ids.overlay_color, with_3d=False),
                 instance_id="seg:" + o["instance_id"])
            for o in msg["objects"]]
        self.sinks.video(channel=CHANNELS["overlay_segmentation"], kind="mask", stream_id=self.ids.overlay_segmentation,
                         group=GROUPS["overlay"], width=w, height=h, data=mask.tobytes(), fmt="mono8",
                         metadata={}, material_params={
                             "NumMaskIDs": float(len(self.session.classes) + 1),
                             # An 8-bit mask texture is sampled as id/255, and RammsUI's mask materials
                             # colour the RAW id (MaskScale * DataTexture.R). RammsUI now sets MaskScale from
                             # the texture's format; a build from before that defaults it to 1, where every
                             # class gets almost the same dark colour and the masks vanish.
                             "MaskScale": 255.0})
        self.stats.count("overlay_color")
        self.stats.count("overlay_segmentation")

    def _segmenter_jpeg(self, msg):
        try:
            from PIL import Image
        except ImportError:
            self.warn("pillow", "--overlay segmenter needs Pillow (pip install 'ramms-tools[compression]')")
            return
        import io
        img = Image.open(io.BytesIO(msg["data"])).convert("RGB")
        self.sinks.video(channel=CHANNELS["overlay_color"], kind="color", stream_id=self.ids.overlay_color,
                         group=GROUPS["overlay"], width=img.width, height=img.height, data=img.tobytes(), fmt="rgb8",
                         metadata={})
        self._colour_sent.add("overlay")
        self.stats.count("overlay_color")

    # ── objects ──────────────────────────────────────────────────

    def objects_payload(self, msg):
        """One UpdateDetectedObjects push: every tracked object, 2D always, 3D when positions are sent."""
        # The image's own size: boxes are pixels of the image, whatever its CameraInfo says.
        w, h = self.session.scene_size or (640, 400)
        stamp = msg["header"]["stamp_ns"]
        with_3d = self.options.positions == "sent"
        objects = []
        for o in msg["objects"]:
            since = self.session.tracker.time_since(stamp, o["instance_id"]) if self.session.tracker else None
            objects.append(detection_to_ue_object(o, w, h, source=self.ids.scene_color, time_since_s=since,
                                                  with_3d=with_3d))
        # One push carries every camera's objects: a push replaces each camera widget's boxes, so the
        # overlay's (the segmenter's, in segmentation mode) ride along with the scene's.
        if self._overlay_settled:
            objects += self._overlay_objects
        return {"stream_id": self.ids.scene_color, "selected_instance_id": "", "objects": objects}

    def _objects(self, msg):
        if self.session.scene_size and not self._colour_done("scene"):
            return  # placed through the scene camera's pose, which only its frames carry
        payload = self.objects_payload(msg)
        self.sinks.objects(payload)
        self.stats.count("objects")
        if any(o.get("has_pose") for o in msg["objects"]):
            # Kept for verification: the push, the camera's pose it was placed through, and the tracker's
            # own 3D (which a "derived" push leaves out, and its markers are compared against).
            try:
                extrinsic = camera_extrinsic(self.session.tf, BASE_FRAME, msg["header"]["frame_id"],
                                             msg["header"]["stamp_ns"])
            except LookupError:
                extrinsic = None
            self.stats.last_3d_payload = {"payload": payload, "extrinsic": extrinsic, "objects": msg["objects"],
                                          "stamp_ns": msg["header"]["stamp_ns"]}

    # ── joints ───────────────────────────────────────────────────

    def _joints(self, raw):
        period_ns = 1e9 / self.options.joints_hz if self.options.joints_hz > 0 else 0
        if self._last_joints_ns is not None and raw.timestamp_ns - self._last_joints_ns < period_ns:
            return
        msg = self.session.reader.decode(raw)
        values = display_joint_targets(msg["name"], msg["position"], gripper_closed_rad=self.options.gripper_closed_rad)
        if not values:
            return
        self._last_joints_ns = raw.timestamp_ns
        self.sinks.joints(values)
        self.stats.last_joint_values = values
        self.stats.count("joints")
