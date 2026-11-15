"""
IMU samples at the physics rate: buffered over Remote Control, or streamed over RMSS.

Every IMU point the sim tracks -- an IMU sensor component's, or a body point read
with ``Sensing.body_imu`` -- keeps a ring buffer of its physics-step samples
(about 2 s: 1024 samples at MuJoCo's 500 Hz). Two ways to get every one of them:

- **Over Remote Control** (``ReadImuSamplesJson``): ask for the samples since a
  cursor. ``Sensing.read_imu_samples`` makes one call; ``Sensing.follow_imu_samples``
  keeps calling with the cursor it was handed and yields every sample, counting
  the ones lost when it was too slow (``dropped``).
- **Over RMSS** (``StartImuStreamJson``): the sim sends, every game tick with new
  samples, one FrameData message (fmt ``imu_f64x18``) holding them.
  ``ImuStream`` subscribes and yields the samples.

    from ramms_tools.sensing import Sensing, ImuStream

    s = Sensing()
    for sample in s.follow_imu_samples(robot.actor_path, "chassis", duration=2.0):
        print(sample.seq, sample.sim_time, sample.linear_acceleration)        # cm/s^2, gravity included

    info = s.start_imu_stream(robot.actor_path, "chassis")                    # channel >= 1000
    with ImuStream.from_info(info) as stream:
        for sample in stream.samples(duration=2.0):
            ...
    s.stop_imu_stream(robot.actor_path, "chassis")

A target is an IMU sensor's name (as Sensing.list names it) or a body (a MuJoCo
body, a component, a bone; "" the actor's main body, as body_imu takes).
Accelerations are cm/s^2 in the sensor frame with gravity in them (+g up at
rest), angular velocity deg/s and angular acceleration deg/s^2 in the sensor
frame, velocity cm/s and orientation in the world.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterator, Optional

import numpy as np

from ramms_tools._rc import as_float, short_name, vec3
from ramms_tools.streaming.client import StreamClient
from ramms_tools.streaming.frames import IMU_FIELDS, IMU_FORMAT, Frame, decode_message, message_role
from ramms_tools.streaming.protocol import MessageType, StreamMessage

from .bridge import SensingError, StreamInfo, same_actor

logger = logging.getLogger(__name__)

__all__ = ["ImuSample", "ImuSamples", "ImuSampleFollower", "ImuStream", "IMU_FIELDS", "IMU_FORMAT", "imu_stream_id"]


def imu_stream_id(actor: str, target: str) -> str:
    """The stream id the sim gives an IMU stream: imu/<actor name>/<target>."""
    return f"imu/{short_name(actor)}/{target}"


def _quat(v: Any) -> tuple[float, float, float, float]:
    if isinstance(v, dict):
        return tuple(as_float(v.get(k, v.get(k.upper(), 1.0 if k == "w" else 0.0))) for k in "xyzw")  # type: ignore[return-value]
    if isinstance(v, (list, tuple)) and len(v) >= 4:
        return (as_float(v[0]), as_float(v[1]), as_float(v[2]), as_float(v[3]))
    return (0.0, 0.0, 0.0, 1.0)


@dataclass
class ImuSample:
    """One physics-step IMU sample."""
    seq: int
    sim_time: float                                              # s, the engine's clock
    linear_acceleration: tuple[float, float, float]              # sensor frame, cm/s^2, gravity included
    angular_velocity: tuple[float, float, float]                 # sensor frame, deg/s
    angular_acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0)   # sensor frame, deg/s^2
    linear_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)        # world, cm/s
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # world, x y z w

    @classmethod
    def from_json(cls, d: dict) -> "ImuSample":
        return cls(int(as_float(d.get("seq"), -1)), as_float(d.get("sim_time")), vec3(d.get("linear_acceleration")),
                   vec3(d.get("angular_velocity")), vec3(d.get("angular_acceleration")), vec3(d.get("linear_velocity")),
                   _quat(d.get("orientation")))

    @classmethod
    def from_row(cls, row) -> "ImuSample":
        """From one row of an imu_f64x18 frame (IMU_FIELDS order)."""
        r = [float(x) for x in row]
        return cls(int(r[0]), r[1], (r[2], r[3], r[4]), (r[5], r[6], r[7]), (r[8], r[9], r[10]), (r[11], r[12], r[13]),
                   (r[14], r[15], r[16], r[17]))

    def linear_acceleration_ms2(self) -> tuple[float, float, float]:
        return tuple(v * 0.01 for v in self.linear_acceleration)  # type: ignore[return-value]

    def flat(self) -> dict:
        """One row by IMU_FIELDS name (CSV / JSON lines)."""
        values = [self.seq, self.sim_time, *self.linear_acceleration, *self.angular_velocity, *self.angular_acceleration,
                  *self.linear_velocity, *self.orientation]
        return dict(zip(IMU_FIELDS, values))


@dataclass
class ImuSamples:
    """What ReadImuSamplesJson answered: the samples since the cursor, oldest first, and the cursor to pass next."""
    actor: str = ""
    target: str = ""
    body: str = ""
    source: str = ""
    samples: list[ImuSample] = field(default_factory=list)
    next: int = -1                   # pass back as `since`
    dropped: int = 0                 # samples lost between `since` and the oldest kept (asked too late)
    rate_hz: float = 0.0             # measured from the samples' sim_time
    units: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_json(cls, d: dict) -> "ImuSamples":
        d = d or {}
        return cls(str(d.get("actor", "") or ""), str(d.get("target", "") or ""), str(d.get("body", "") or ""),
                   str(d.get("source", "") or ""), [ImuSample.from_json(s) for s in (d.get("samples") or [])],
                   int(as_float(d.get("next"), -1)), int(as_float(d.get("dropped"), 0)), as_float(d.get("rate_hz")),
                   dict(d.get("units") or {}), raw=d)


class ImuSampleFollower:
    """
    Iterates over every physics-step sample of an IMU target, polling ReadImuSamplesJson with the cursor it was
    handed (see Sensing.follow_imu_samples). `dropped` counts samples lost because a poll came after the sim's buffer
    had moved past them; `last` is the last answer (source, rate_hz, units).
    """

    def __init__(self, read: Callable[[int], ImuSamples], period: float = 0.1, duration: Optional[float] = None,
                 should_stop: Optional[Callable[[], bool]] = None):
        self._read = read
        self.period = period
        self.duration = duration
        self.should_stop = should_stop
        self.dropped = 0
        self.count = 0
        self.last: Optional[ImuSamples] = None

    def __iter__(self) -> Iterator[ImuSample]:
        t0 = time.monotonic()
        res = self._read(-1)                     # starts now: no samples, only the cursor
        self.last = res
        cursor = res.next
        while True:
            if self.duration is not None and time.monotonic() - t0 >= self.duration:
                return
            if self.should_stop is not None and self.should_stop():
                return
            t = time.monotonic()
            res = self._read(cursor)
            self.last = res
            if res.dropped:
                self.dropped += res.dropped
                logger.warning("IMU samples: %d lost (polled too late for the sim's buffer)", res.dropped)
            for s in res.samples:
                self.count += 1
                yield s
            cursor = res.next if res.next >= 0 else cursor
            delay = self.period - (time.monotonic() - t)
            if delay > 0:
                time.sleep(delay)


@dataclass
class ImuBatch:
    """One IMU stream message: its samples (an (n, 18) float64 array in IMU_FIELDS order) and metadata."""
    array: np.ndarray
    meta: dict = field(default_factory=dict, repr=False)
    channel: int = 0
    first_seq: int = -1
    dropped: int = 0                 # the sim's count of samples lost before this message

    @property
    def samples(self) -> list[ImuSample]:
        return [ImuSample.from_row(r) for r in self.array]


class ImuStream:
    """
    Subscribe to an IMU stream over RMSS (Sensing.start_imu_stream) and decode it.

    Args:
        channel: the stream's channel (from start_imu_stream / the stream status); None = every channel, keeping the
            frames whose stream id (imu/<actor name>/<target>) or metadata names this actor and target.
        actor, target: whose IMU (actor path or name; target as given to start_imu_stream).
        client: an existing StreamClient; else one is made for host:port.
    """

    def __init__(self, channel: Optional[int] = None, actor: Optional[str] = None, target: Optional[str] = None,
                 host: str = "127.0.0.1", port: int = 30030, client: Optional[StreamClient] = None):
        self.channel = channel if channel is not None and channel >= 0 else None
        self.actor = actor or None
        self.target = target
        self.client = client or StreamClient(host, port)
        self._own_client = client is None
        self._last_seq: Optional[int] = None
        self.dropped = 0                 # samples missing between messages (by seq): the sim's losses and ours
        self.count = 0

    @classmethod
    def from_info(cls, info: StreamInfo, host: str = "127.0.0.1", port: Optional[int] = None, **kw) -> "ImuStream":
        """From what start_imu_stream answered (or an "imu" entry of the stream status)."""
        port = port or getattr(info, "port", None) or 30030
        return cls(info.channel, info.actor or None, info.target, host=host, port=port, **kw)

    def __repr__(self) -> str:
        return f"ImuStream(channel={self.channel}, actor={self.actor!r}, target={self.target!r})"

    def connect(self, timeout: float = 5.0) -> "ImuStream":
        if not self.client.connected:
            self.client.connect(timeout=timeout)
        try:
            self.client.subscribe(channels=[self.channel] if self.channel is not None else None)
            self.client.start()
        except BaseException:
            if self._own_client:
                self.client.disconnect()
            raise
        return self

    def close(self) -> None:
        if self._own_client:
            self.client.disconnect()

    def __enter__(self) -> "ImuStream":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    def _mine(self, msg: StreamMessage, meta: dict) -> bool:
        if msg.header.message_type != MessageType.FRAME_DATA or message_role(msg, meta) != "imu":
            return False
        if self.channel is not None and msg.header.channel_id != self.channel:
            return False
        a, t = str(meta.get("actor", "") or ""), meta.get("target")
        if self.actor is not None and a and not same_actor(a, self.actor):
            return False
        if self.target is not None and t is not None and str(t) != self.target:
            return False
        return True

    def poll(self, timeout: float = 1.0) -> Optional[ImuBatch]:
        """The next message of this IMU stream within `timeout` seconds (None if none came)."""
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            msg = self.client.poll(timeout=max(0.0, left))
            if msg is None:
                if left <= 0:
                    return None
                continue
            meta = msg.get_metadata_json() if msg.metadata else {}
            if not self._mine(msg, meta):
                continue
            try:
                frame: Frame = decode_message(msg, role="imu")
            except ValueError as e:
                logger.warning("skipped an IMU frame: %s", e)
                continue
            arr = frame.array
            first = int(arr[0, 0]) if len(arr) else int(as_float(meta.get("first_seq"), -1))
            if self._last_seq is not None and len(arr) and first > self._last_seq + 1:
                self.dropped += first - self._last_seq - 1
            if len(arr):
                self._last_seq = int(arr[-1, 0])
            self.count += len(arr)
            return ImuBatch(arr, meta, msg.header.channel_id, first, int(as_float(meta.get("dropped"), 0)))

    def samples(self, timeout: float = 2.0, duration: Optional[float] = None,
                should_stop: Optional[Callable[[], bool]] = None) -> Iterator[ImuSample]:
        """Every sample as it arrives; stops after `timeout` s without a message, after `duration` s, or when
        should_stop() is true."""
        t0 = time.monotonic()
        while True:
            if duration is not None and time.monotonic() - t0 >= duration:
                return
            if should_stop is not None and should_stop():
                return
            batch = self.poll(timeout if duration is None else min(timeout, max(0.0, duration - (time.monotonic() - t0))))
            if batch is None:
                if duration is not None and time.monotonic() - t0 < duration:
                    continue
                return
            for s in batch.samples:
                yield s
