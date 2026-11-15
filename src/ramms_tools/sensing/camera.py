"""
One camera's RMSS streams as decoded numpy frames.

    from ramms_tools.sensing import CameraStreams, Sensing

    sensing = Sensing()
    status = sensing.start_streams()                          # capture on, RMSS server up
    cam = next(s for s in robot.sensors.list() if s.name == "front_camera")
    with CameraStreams.from_sensor(cam, port=status.port) as streams:   # this robot's camera: its actor and channel
        frames = streams.grab()                               # {"color": Frame, "depth": Frame, "motion": ..., "mask": ...}
        rgb = frames["color"].array                           # (H, W, 3) uint8 RGB
        depth_cm = frames["depth"].array                      # (h, w) float32 cm
        flow = frames["motion"].array                         # (h, w, 2) float32 px / frame
        mask = frames["mask"].array                           # (H, W) uint8 class values
        print(frames["mask"].classes)                         # {3: "Car", ...}

Stream ids carry only the camera's name (``camera/<name>/color`` ...), and two
robots can both have a ``front_camera``: a camera is picked by its actor and
its base channel, never by name alone. Every frame's metadata names its
``actor``; frames of another actor are dropped. Without a known base channel it
subscribes to every channel and keeps the frames whose stream id (or group)
names the camera -- of the given actor. A camera placed by hand can stream
under another name (its stream source's StreamName): from_sensor / from_status
take the stream ids the sim reports.

The sim sends a connection no frames until it has subscribed; connect() always
sends a SUBSCRIBE (for the camera's channels, or every channel).

``grab()`` returns a ``FrameSet``: the frames of ONE capture (when the sender
numbers its frames, as the sim does), with ``complete`` False and ``missing``
naming the roles that had not arrived by the timeout -- never frames of
different captures mixed without saying so (``same_capture``).
"""

from __future__ import annotations

import collections
import json
import logging
import time
from pathlib import Path
from typing import Iterator, Optional, Sequence

import numpy as np

from ramms_tools.streaming.client import StreamClient
from ramms_tools.streaming.frames import (ROLE_CHANNEL_OFFSETS, ROLES, Frame, camera_channels, camera_stream_ids,
                                          decode_message, decode_rgbd, message_role)
from ramms_tools.streaming.protocol import MessageType, StreamMessage

from .bridge import SensingError, same_actor

logger = logging.getLogger(__name__)

_FRAME_MESSAGES = frozenset({MessageType.FRAME_RGB, MessageType.FRAME_DEPTH, MessageType.FRAME_MOTION,
                             MessageType.FRAME_DATA, MessageType.IMAGE_DATA, MessageType.FRAME_RGBD})


def _group_of(stream_ids: dict[str, str]) -> str:
    """The camera name a set of stream ids is under (camera/<name>/color, motion/<name> ...)."""
    for sid in stream_ids.values():
        parts = str(sid).split("/")
        if len(parts) == 3 and parts[0] == "camera":
            return parts[1]
        if len(parts) == 2 and parts[0] in ("motion", "mask"):
            return parts[1]
    return ""


class FrameSet(dict):
    """
    What CameraStreams.grab() returns: {role: Frame}.

    complete: every role asked for is in it; missing: the ones that are not (they had not arrived by the timeout).
    frame_number: the capture every frame in it belongs to (None when the sender does not number its frames).
    same_capture: the frames are of one capture -- False only when the sender gives no frame numbers, or grab() was
    asked for the newest of each role (same_frame=False).
    """

    def __init__(self, frames: dict, wanted: Sequence[str], frame_number: Optional[int] = None,
                 same_capture: bool = True):
        super().__init__(frames)
        self.missing: tuple[str, ...] = tuple(r for r in wanted if r not in frames)
        self.complete = not self.missing
        self.frame_number = frame_number
        self.same_capture = same_capture


class CameraStreams:
    """
    Subscribe to one camera's streams and decode them.

    Args:
        camera: the camera's name (the mount name; its stream ids are camera/<name>/color ... mask/<name>).
        base_channel: its color channel (depth +100, motion +200, mask +300); None (or -1) = subscribe to
            everything and filter by stream id.
        roles: which of color, depth, motion, mask to take.
        actor: the camera's actor (path or name): frames whose metadata names another actor are dropped. None = any
            (only safe when no other actor has a camera of this name).
        client: an existing StreamClient (connected or not); else one is made for host:port.
        rgbd: its stream source sends one combined FRAME_RGBD (bStreamRGBD) on the base channel instead of separate
            colour and depth (from_status / from_sensor set it from what the sim reports).
        stream_ids: {role: stream id} as the sim reports them, for a camera whose stream source names its streams
            otherwise than after the camera (from_status / from_sensor pass them); default camera/<name>/color ...
    """

    def __init__(self, camera: str, base_channel: Optional[int] = None, roles: Sequence[str] = ROLES,
                 host: str = "127.0.0.1", port: int = 30030, client: Optional[StreamClient] = None,
                 compression: str = "none", actor: Optional[str] = None, rgbd: bool = False,
                 stream_ids: Optional[dict[str, str]] = None):
        bad = [r for r in roles if r not in ROLES]
        if bad:
            raise ValueError(f"unknown roles {bad} (roles: {', '.join(ROLES)})")
        if base_channel is not None and base_channel < 0:
            base_channel = None
        self.camera = camera
        self.base_channel = base_channel
        self.roles = tuple(roles)
        self.actor = actor or None
        self.rgbd = rgbd
        self.compression = compression
        self.client = client or StreamClient(host, port)
        self._own_client = client is None
        ids = camera_stream_ids(camera)
        ids.update({r: str(sid) for r, sid in (stream_ids or {}).items() if r in ROLES and sid})
        self.group = _group_of(ids) or camera     # the name the streams go out under (metadata "group")
        self.stream_ids = {r: sid for r, sid in ids.items() if r in self.roles}
        self._by_id = {sid: r for r, sid in self.stream_ids.items()}
        self.channels = ({r: ch for r, ch in camera_channels(base_channel).items() if r in self.roles}
                         if base_channel is not None else {})
        self._by_channel = {ch: r for r, ch in self.channels.items()}
        self._pending: collections.deque[Frame] = collections.deque()   # the second half of a combined RGBD frame
        self._warned: set[str] = set()
        self.skipped = 0
        self.other_actor = 0              # frames of a camera of this name on another actor, dropped

    @classmethod
    def from_status(cls, status, camera: str, actor: Optional[str] = None, **kw) -> "CameraStreams":
        """
        Use a StreamStatus (Sensing.status / start_streams) for the camera's channels and the server's port. With
        several actors streaming a camera of this name, `actor` must say which (SensingError otherwise). Raises
        SensingError when the RMSS server is not running.
        """
        if "port" not in kw and "client" not in kw:
            if status.port is None:
                raise SensingError("the RMSS server is not running: start the camera streams first "
                                   "(Sensing.start_streams, ramms-sensors streams start)")
            kw["port"] = status.port
        actors = status.camera_actors(camera)
        if actor is None and len(actors) > 1:
            raise SensingError(f"cameras named {camera!r} stream on {len(actors)} actors ({', '.join(actors)}): "
                               "say which with actor=")
        if actor is None and len(actors) == 1:
            actor = actors[0] or None
        base = None
        kw.setdefault("rgbd", any(s.role == "rgbd" for s in status.camera_streams(camera, actor)))
        kw.setdefault("stream_ids", {s.role: s.stream_id for s in status.camera_streams(camera, actor)
                                     if s.role in ROLES and s.stream_id})
        for s in status.camera_streams(camera, actor):
            if s.role in ROLE_CHANNEL_OFFSETS and s.channel >= 0:
                base = s.channel - ROLE_CHANNEL_OFFSETS[s.role]
                break
            if s.role == "rgbd" and s.channel >= 0:
                base = s.channel
                break
        if base is None:
            logger.warning("camera %r%s is not among the streams the sim reports; subscribing to every channel", camera,
                           f" of {actor}" if actor else "")
        return cls(camera, base_channel=base, actor=actor, **kw)

    @classmethod
    def from_sensor(cls, sensor, **kw) -> "CameraStreams":
        """Use a camera's SensorInfo (Sensing.list): its name, actor and the channel it actually streams on; only the
        roles it streams."""
        if sensor.kind != "Camera":
            raise ValueError(f"{sensor.name} is a {sensor.kind}, not a camera")
        roles = [st.role for st in sensor.streams if st.role in ROLES]
        if any(st.role == "rgbd" for st in sensor.streams):
            roles += [r for r in ("color", "depth") if r not in roles]
            kw.setdefault("rgbd", True)
        if roles:
            kw.setdefault("roles", [r for r in ROLES if r in roles])
        kw.setdefault("actor", sensor.actor or None)
        # A camera placed by hand streams under its stream source's StreamName, not its component name.
        kw.setdefault("stream_ids", {st.role: st.stream_id for st in sensor.streams if st.role in ROLES and st.stream_id})
        return cls(sensor.name, base_channel=sensor.channel, **kw)

    def __repr__(self) -> str:
        return f"CameraStreams({self.camera!r}, base_channel={self.base_channel}, roles={self.roles}, actor={self.actor!r})"

    # ── connection ──────────────────────────────────────────────────

    def connect(self, timeout: float = 5.0) -> "CameraStreams":
        if not self.client.connected:
            self.client.connect(timeout=timeout)
        try:
            channels = sorted(self.channels.values()) if self.channels else None
            if channels is not None and self.rgbd and {"color", "depth"} & set(self.roles):
                channels = sorted(set(channels) | {self.base_channel})     # the combined RGBD comes on the base channel
            self.client.subscribe(channels=channels, compression=self.compression)
            self.client.start()
        except BaseException:
            if self._own_client:
                self.client.disconnect()
            raise
        return self

    def close(self) -> None:
        if self._own_client:
            self.client.disconnect()

    def __enter__(self) -> "CameraStreams":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # ── frames ──────────────────────────────────────────────────────

    def _actor_ok(self, meta: dict) -> bool:
        a = str(meta.get("actor", "") or "")
        if self.actor is None or not a or same_actor(a, self.actor):
            return True
        self.other_actor += 1
        return False

    def role_of(self, msg: StreamMessage, meta: Optional[dict] = None) -> str:
        """The role a message plays for this camera ("rgbd" for a combined frame on its base channel), or "" when it
        is another camera's (or another actor's) / not a frame."""
        if msg.header.message_type not in _FRAME_MESSAGES:
            return ""
        meta = msg.get_metadata_json() if meta is None and msg.metadata else (meta or {})
        if not self._actor_ok(meta):
            return ""
        if msg.header.message_type == MessageType.FRAME_RGBD:
            # A combined frame has no stream id: it is this camera's when it comes on its base channel.
            return "rgbd" if self.base_channel is not None and msg.header.channel_id == self.base_channel else ""
        sid = str(meta.get("stream_id", "") or "")
        if sid:
            role = self._by_id.get(sid, "")
            if role and self._by_channel and msg.header.channel_id not in self._by_channel:
                return ""              # the same stream id from another camera of this name, on its own channel
            return role
        if str(meta.get("group", "") or "") == self.group:
            role = message_role(msg, meta)
            return role if role in self.roles else ""
        return self._by_channel.get(msg.header.channel_id, "")

    def decode(self, msg: StreamMessage) -> Optional[Frame]:
        """A message as this camera's Frame (None when it is not one of its streams). A combined RGBD frame gives its
        colour half here and keeps the depth half for the next poll."""
        meta = msg.get_metadata_json() if msg.metadata else {}
        role = self.role_of(msg, meta)
        if not role:
            return None
        if role == "rgbd":
            halves = [f for f in decode_rgbd(msg) if f.role in self.roles]
            if not halves:
                return None
            for f in halves:
                f.camera = f.camera or self.camera
            self._pending.extend(halves[1:])
            return halves[0]
        frame = decode_message(msg, role=role)
        if not frame.camera:
            frame.camera = self.camera
        return frame

    def _skip(self, e: ValueError) -> None:
        self.skipped += 1
        key = "compressed" if "compressed" in str(e) else str(e).split(":", 1)[0]
        if key not in self._warned:              # once per kind of problem, not once per frame
            self._warned.add(key)
            logger.warning("skipping frames: %s", e)

    def poll(self, timeout: float = 1.0) -> Optional[Frame]:
        """The next frame of this camera within `timeout` seconds (None if none came; timeout 0 looks once)."""
        if self._pending:
            return self._pending.popleft()
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            msg = self.client.poll(timeout=max(0.0, left))
            if msg is None:
                if left <= 0:
                    return None
                continue
            try:
                frame = self.decode(msg)
            except ValueError as e:
                self._skip(e)
                continue
            if frame is not None:
                return frame
            if self._pending:
                return self._pending.popleft()

    def frames(self, timeout: float = 1.0, max_frames: int = 0) -> Iterator[Frame]:
        """Yield this camera's frames as they arrive; stops after `timeout` s without one, or max_frames."""
        n = 0
        while True:
            f = self.poll(timeout)
            if f is None:
                return
            yield f
            n += 1
            if 0 < max_frames <= n:
                return

    def grab(self, roles: Optional[Sequence[str]] = None, timeout: float = 5.0, same_frame: bool = True
             ) -> FrameSet:
        """
        One set of frames, a frame of each role (default: this camera's roles), the newest available: what is
        already queued is read first. With same_frame (the default) the set is one capture -- every frame carries the
        same metadata "frame" number -- when the sender numbers its frames (the sim does); otherwise, or for frames
        without a number, the newest of each role (FrameSet.same_capture False).

        Waits up to `timeout` seconds. If no capture is complete by then, the answer is the newest capture with the
        most roles, with complete False and missing naming the roles left out -- not a mix of captures (without
        frame numbers, the newest frame of each role that came, same_capture False). Empty when nothing came.
        """
        want = set(roles or self.roles)
        latest: dict[str, Frame] = {}
        by_frame: dict[int, dict[str, Frame]] = {}

        def take(f: Frame) -> None:
            if f.role not in want:
                return
            latest[f.role] = f
            if f.frame_number is not None:
                by_frame.setdefault(f.frame_number, {})[f.role] = f
                if len(by_frame) > 32:                     # keep the recent captures only
                    for fn in sorted(by_frame)[:-32]:
                        del by_frame[fn]

        order = [r for r in (roles or self.roles) if r in want]

        def complete() -> Optional[FrameSet]:
            if same_frame and by_frame:
                full = [fn for fn, d in by_frame.items() if want <= set(d)]
                return FrameSet(by_frame[max(full)], order, max(full)) if full else None
            if want <= set(latest):
                return FrameSet(latest, order, None, same_capture=False)
            return None

        def best_partial() -> FrameSet:
            if same_frame and by_frame:
                fn = max(by_frame, key=lambda k: (len(by_frame[k]), k))     # the most roles, the newest of those
                return FrameSet(by_frame[fn], order, fn)
            return FrameSet(latest, order, None, same_capture=False)

        while True:                                        # what is queued now, newest last
            f = self.poll(0)
            if f is None:
                break
            take(f)
        end = time.monotonic() + timeout
        while True:
            got = complete()
            if got is not None:
                return got
            left = end - time.monotonic()
            if left <= 0:
                return best_partial()
            f = self.poll(min(left, 0.5))
            if f is not None:
                take(f)


# ── saving ────────────────────────────────────────────────────────────


def _has_pillow() -> bool:
    try:
        import PIL.Image  # noqa: F401
        return True
    except ImportError:
        return False


def save_frame(frame: Frame, out_dir: Path, index: int, png: Optional[bool] = None) -> list[Path]:
    """
    Write a frame under out_dir as <role>_<index>.<ext> plus a .json of its metadata:
    color .png (Pillow) else .npy; depth .npy (float32 cm); motion .npy (float32 x, y); mask .png (Pillow) else
    .npy. Returns the files written.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    png = _has_pillow() if png is None else png
    stem = out_dir / f"{frame.role}_{index:06d}"
    written: list[Path] = []
    arr = frame.array
    if frame.role in ("color", "mask") and png and arr.dtype == np.uint8:
        from PIL import Image
        p = stem.with_suffix(".png")
        Image.fromarray(np.ascontiguousarray(arr)).save(p)      # (H, W, 3) -> RGB, (H, W) -> L
    else:
        p = stem.with_suffix(".npy")
        np.save(p, arr)
    written.append(p)
    meta = dict(frame.meta)
    meta.update(_role=frame.role, _channel=frame.channel, _sequence=frame.sequence, _timestamp_us=frame.timestamp_us,
                _shape=list(arr.shape), _dtype=str(arr.dtype))
    j = stem.with_suffix(".json")
    j.write_text(json.dumps(meta, indent=1), encoding="utf-8")
    written.append(j)
    return written
