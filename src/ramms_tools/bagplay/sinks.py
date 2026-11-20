"""Where a bag replay's messages go: the live UI (RMSS + Remote Control), or a recording.

Both sinks take the same calls, so the player never knows whether it is playing into RammpUI, a
dry run, or a test:

  video(channel, kind, stream_id, group, width, height, data, fmt, metadata, material_params=None)
  objects(payload)   one UpdateDetectedObjects push
  joints(values)     {display joint name: degrees}, every other joint held
  feed(open)         OpenCameraFeed / CloseCameraFeed (the overlay camera's window)
  close()            leave the UI as it was found
"""

from __future__ import annotations

import threading


class RecordingSinks:
    """Keeps everything that would be sent -- for --dry-run and for tests."""

    def __init__(self):
        self.frames = []
        self.pushes = []
        self.joint_sends = []
        self.feed_calls = []
        self.closed = False

    def video(self, channel, kind, stream_id, group, width, height, data, fmt, metadata, material_params=None):
        self.frames.append({"channel": channel, "kind": kind, "stream_id": stream_id, "group": group, "width": width,
                            "height": height, "bytes": len(data), "fmt": fmt, "metadata": metadata,
                            "material_params": material_params})

    def objects(self, payload):
        self.pushes.append(payload)

    def joints(self, values):
        self.joint_sends.append(dict(values))

    def feed(self, open_):
        self.feed_calls.append(bool(open_))

    def close(self):
        self.closed = True


class LiveSinks:
    """The running UI: RMSS for the cameras, Remote Control for objects, joints and the overlay window.

    Remote Control goes through a worker that keeps only the NEWEST detections and joints: an HTTP call
    to an editor in the background can take tens of milliseconds, and video must not wait for it -- the
    UI shows the latest set anyway, so a set superseded before it was sent is not worth sending.

    close() puts the UI back: an empty detections push (which clears boxes, POIs and world markers on
    every camera), the overlay window closed if this opened it, the joints restored, RMSS disconnected.
    """

    def __init__(self, client, sender, rig=None, log=print):
        self.client = client
        self.sender = sender
        self.rig = rig
        self.log = log
        self.original_joints = rig.targets() if rig else None
        self.sent_joint_vector = None
        # Every joint the replay has set so far: a /joint_states message may name only some joints,
        # and the others must keep their replayed values, not snap back to the display's originals.
        self._joint_state = {}
        self.errors = {}
        self._feed_open = False
        self._slots = {}
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = False
        self._idle = threading.Event()
        self._idle.set()
        self._thread = threading.Thread(target=self._run, name="ramms-bag-rc", daemon=True)
        self._thread.start()

    # ── RMSS ─────────────────────────────────────────────────────

    def video(self, channel, kind, stream_id, group, width, height, data, fmt, metadata, material_params=None):
        if kind == "depth":
            self.sender.send_depth_uint16(channel, data, width, height, metadata=metadata, group=group, role="depth",
                                          stream_id=stream_id)
        elif kind == "mask":
            self.sender.send_mask(channel, data, width, height, fmt=fmt, metadata=metadata, group=group,
                                  stream_id=stream_id, material_params=material_params)
        else:
            self.sender.send_image(channel, data, width, height, fmt=fmt, metadata=metadata, group=group,
                                   role="color", stream_id=stream_id, material_params=material_params)

    # ── Remote Control ───────────────────────────────────────────

    def objects(self, payload):
        self._post("objects", payload)

    def joints(self, values):
        if self.rig is None:
            return
        with self._lock:
            self._joint_state.update(values)
            snapshot = dict(self._joint_state)
        self._post("joints", snapshot)

    def feed(self, open_):
        call = self.client.open_camera_feed if open_ else self.client.close_camera_feed
        try:
            call()
            self._feed_open = bool(open_)
        except Exception as exc:  # the feed is optional on a preset; the cameras still stream
            self._error("feed", exc)

    def _post(self, slot, value):
        with self._lock:
            self._slots[slot] = value
            self._idle.clear()
        self._wake.set()

    def _run(self):
        while True:
            self._wake.wait()
            with self._lock:
                self._wake.clear()
                slots, self._slots = self._slots, {}
                if not slots:
                    self._idle.set()
                    if self._stop:
                        return
                    continue
            for slot, value in slots.items():
                try:
                    if slot == "objects":
                        self.client.update_detected_objects(value)
                    else:
                        self._send_joints(value)
                except Exception as exc:
                    self._error(slot, exc)
            self._wake.set()  # look again: more may have arrived while sending

    def _send_joints(self, values):
        from ramms_tools.scripts.ui_bridge_test import JOINT_FUNCTION, with_joints
        rig = self.rig
        known = {name: v for name, v in values.items() if name in rig.index}
        vector = with_joints(self.original_joints, rig.index, known)
        # As JointRig.send routes it, without its read-back on every frame: in PIE the game's own
        # component (the preset's entry names the editor's level), in a game the preset.
        if rig.world == "play session":
            self.client.call_object(rig.pose_path, JOINT_FUNCTION, Values=vector)
        else:
            self.client.call(JOINT_FUNCTION, Values=vector)
        self.sent_joint_vector = vector

    def _error(self, what, exc):
        first = what not in self.errors
        self.errors[what] = self.errors.get(what, 0) + 1
        if first:
            self.log(f"  error ({what}): {exc} -- further ones are counted, not printed")

    def drain(self, timeout=5.0):
        """Wait until every queued push has been sent."""
        self._wake.set()
        self._idle.wait(timeout)

    # ── leaving ──────────────────────────────────────────────────

    def close(self):
        self.drain()
        with self._lock:
            self._stop = True
        self._wake.set()
        self._thread.join(timeout=5.0)
        steps = [("objects", lambda: self.client.update_detected_objects(
            {"stream_id": "", "selected_instance_id": "", "objects": []}))]
        if self._feed_open:
            steps.append(("feed", self.client.close_camera_feed))
        if self.rig is not None and self.sent_joint_vector is not None:
            steps.append(("joints", lambda: self.rig.restore(self.original_joints)))
        steps.append(("rmss", lambda: self.sender.disconnect()))
        for what, step in steps:
            try:
                step()
            except Exception as exc:
                self._error(f"cleanup:{what}", exc)
