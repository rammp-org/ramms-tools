"""
Robot sensing over Remote Control (the RammsSensingBridge) and RMSS (camera frames).

Sensors -- ToF, sonar, IMU and capture cameras -- are put on an actor at
runtime from mounts (``SensorMount``, or a rig preset), read over Remote
Control, and the cameras stream colour, depth, motion vectors and
segmentation masks over RMSS:

    from ramms_tools.robot import Robot
    from ramms_tools.sensing import Sensing, SensorMount, CameraStreams

    robot = Robot.find()
    robot.sensors.apply_rig("rammp_gen1")                      # front ToF, sonar, IMU, RGB-D camera
    robot.sensors.add(SensorMount("left_sonar", "Sonar", parent="chassis", location=(30, 30, 25), rotation=(0, 60, 0)))
    for s in robot.sensors.list():
        print(s.name, s.kind, s.summary())
    print(robot.sensors.read("front_tof", measure=True).grid())  # (8, 8) cm, NaN = no hit

    sensing = Sensing()
    status = sensing.start_streams(motion=True, segmentation=True)
    with CameraStreams.from_status(status, "front_camera", actor=robot.actor_path) as cam:
        frames = cam.grab()                                    # color / depth / motion / mask Frames of one capture
    classes = sensing.segmentation_classes().names()           # {value: fine class}

Every physics-step IMU sample (ramms_tools.sensing.imu), over Remote Control or as an RMSS stream:

    for sample in sensing.follow_imu_samples(robot.actor_path, "chassis", duration=2.0):    # ReadImuSamplesJson
        print(sample.seq, sample.sim_time, sample.linear_acceleration)
    info = sensing.start_imu_stream(robot.actor_path, "chassis")                          # FrameData imu_f64x18
    with ImuStream.from_info(info) as stream:
        for sample in stream.samples(duration=2.0):
            ...
    sensing.stop_imu_stream(robot.actor_path, "chassis")

Camera stream ids carry only the camera's name; two robots can both have a
"front_camera". Pick a robot's camera by its actor (CameraStreams.from_sensor,
or actor= on CameraStreams / from_status), never by name alone.

Rig presets (ramms_tools.sensing.rigs): "rammp_gen1" (the rig the Gen1 pawn ships with) and "chair".
The bridge is /Script/RammsCore.Default__RammsSensingBridge; it must be
allow-listed in the project's Config/DefaultRemoteControl.ini. Sensors added
through it live on a sensor rig component (named from "SensorRig") and only
those can be removed; each camera gets a stream source "<name>_stream".
Adding / removing sensors and starting streams need a play world (PIE or a game)
on sims from the fix round (older sims keep a mount added in the editor world and
build it at BeginPlay: "built" false); the segmentation classes always do.

Units: ToF / sonar distances in cm (-1 = no hit); a sonar's cone is a
half-angle; IMU values are the component's own (cm/s^2, deg/s; see
SensorReading.units and linear_acceleration_ms2()).
"""

from __future__ import annotations

from ramms_tools.streaming.frames import ROLES, Frame, camera_channels, camera_stream_ids

from .bridge import (BRIDGE, KINDS, ActorSensors, SegmentationClass, SegmentationTable, Sensing, SensingError,
                     SensorInfo, SensorMount, SensorReading, StreamInfo, StreamStatus, as_mount, canonical_kind)
from .camera import CameraStreams, save_frame
from .imu import ImuBatch, ImuSample, ImuSampleFollower, ImuSamples, ImuStream, imu_stream_id
from .rigs import RIG_DESCRIPTIONS, RIG_PRESETS, apply_rig, rig_mounts, rig_names

CameraFrame = Frame

__all__ = [
    "Sensing", "ActorSensors", "SensorMount", "SensorInfo", "SensorReading", "StreamInfo", "StreamStatus",
    "SegmentationClass", "SegmentationTable", "SensingError", "BRIDGE", "KINDS", "as_mount", "canonical_kind",
    "CameraStreams", "CameraFrame", "Frame", "save_frame", "ROLES", "camera_channels", "camera_stream_ids",
    "RIG_PRESETS", "RIG_DESCRIPTIONS", "rig_mounts", "rig_names", "apply_rig",
    "ImuSample", "ImuSamples", "ImuSampleFollower", "ImuStream", "ImuBatch", "imu_stream_id",
]
