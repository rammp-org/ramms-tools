#!/usr/bin/env python3
"""
Stream IMU-like data (orientation, linear acceleration, angular velocity)
from an Unreal Engine actor, scene component, or skeletal mesh bone.

Data is read via the UE Remote Control API by polling the object's
transform and physics state at a configurable rate.

Usage:
    # Stream from an actor (by path or auto-discovered by name/class)
    ramms-imu --actor BP_Mebot_Ramms_C_0
    ramms-imu --actor BP_Mebot_Ramms_C_0 --rate 30

    # Stream from a specific component on an actor
    ramms-imu --actor BP_Mebot_Ramms_C_0 --component ArmSkMesh

    # Stream from a skeletal mesh bone
    ramms-imu --actor BP_Mebot_Ramms_C_0 --component ArmSkMesh --bone end_effector

    # Output as CSV (one row per sample)
    ramms-imu --actor BP_Mebot_Ramms_C_0 --format csv

    # Output as JSON lines
    ramms-imu --actor BP_Mebot_Ramms_C_0 --format json

Requires Unreal Engine running with Remote Control API plugin enabled (port 30010).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time

from ramms_tools.transforms import angle_diff, quat_to_euler, rotation_matrix_from_euler, world_to_local
from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError


def resolve_target(ue: UnrealRemote, actor_hint: str,
                   component_hint: str = "", bone_name: str = "") -> dict:
    """
    Resolve the target object to stream IMU data from.

    Returns a dict with:
      - object_path: str (actor or component path)
      - bone_name: str or None
      - label: str (human-readable description)
    """
    # Find the actor
    actor_path = None
    if "/" in actor_hint:
        # Full path provided
        actor_path = actor_hint
    else:
        # Search by name substring
        actors = ue.find_actors()
        for a in actors:
            if actor_hint.lower() in a.object_path.lower():
                actor_path = a.object_path
                break
        if not actor_path:
            raise RuntimeError(f"No actor matching '{actor_hint}' found")

    target_path = actor_path
    label = actor_path.rsplit(".", 1)[-1]
    is_component = False

    # Optionally resolve a component
    if component_hint:
        comps = ue.find_components(actor_path, component_hint)
        if not comps:
            raise RuntimeError(
                f"No component matching '{component_hint}' on {actor_path}")
        target_path = comps[0]["path"]
        label = f"{label}.{comps[0]['name']}"
        is_component = True

    bone = bone_name if bone_name else None
    if bone:
        label = f"{label}[{bone}]"

    return {
        "object_path": target_path,
        "bone_name": bone,
        "label": label,
        "is_component": is_component,
    }


def read_imu_sample(ue: UnrealRemote, target: dict,
                    prev_sample: dict | None, dt: float,
                    frame: str = "world") -> dict:
    """
    Read one IMU sample from the target.

    Args:
        frame: Coordinate frame for velocity/acceleration/angular velocity.
            "world" — all values in world frame (default).
            "local" — velocity, acceleration, and angular velocity are
                      transformed into the body's local coordinate frame
                      (like a real IMU).
            "both"  — includes both world and local variants.

    Returns a dict with:
      - timestamp: float (seconds since epoch)
      - orientation: {roll, pitch, yaw} in degrees (always world frame)
      - linear_acceleration: {x, y, z} in cm/s²
      - angular_velocity: {x, y, z} in degrees/s
      - position: {x, y, z} in cm (always world frame)
      - linear_velocity: {x, y, z} in cm/s
    When frame="both", also includes *_local variants of the above vectors.
    """
    obj_path = target["object_path"]
    bone = target["bone_name"]
    is_component = target.get("is_component", False)
    now = time.time()

    if bone:
        # For bones, call GetBoneTransform-like functions
        # Use GetSocketTransform which works for both sockets and bones
        try:
            result = ue._call_function(obj_path, "GetSocketTransform", {
                "InSocketName": bone,
                "TransformSpace": "RTS_World",
            })

            pos = result.get("Translation", {})
            rot = result.get("Rotation", {})
            position = {
                "x": pos.get("X", 0.0),
                "y": pos.get("Y", 0.0),
                "z": pos.get("Z", 0.0),
            }
            # Rotation from quaternion to euler
            qx = rot.get("X", 0.0)
            qy = rot.get("Y", 0.0)
            qz = rot.get("Z", 0.0)
            qw = rot.get("W", 1.0)
            orientation = _quat_to_euler(qx, qy, qz, qw)
        except UnrealRemoteError:
            position = {"x": 0, "y": 0, "z": 0}
            orientation = {"roll": 0, "pitch": 0, "yaw": 0}
    else:
        # For actors: use K2_GetActorLocation/K2_GetActorRotation (UFUNCTIONs)
        # For components: use properties RelativeLocation/RelativeRotation

        if is_component:
            try:
                loc = ue._get_property(obj_path, "RelativeLocation")
                if isinstance(loc, dict):
                    position = {
                        "x": loc.get("X", 0.0),
                        "y": loc.get("Y", 0.0),
                        "z": loc.get("Z", 0.0),
                    }
                else:
                    position = {"x": 0, "y": 0, "z": 0}
            except UnrealRemoteError:
                position = {"x": 0, "y": 0, "z": 0}

            try:
                rot = ue._get_property(obj_path, "RelativeRotation")
                if isinstance(rot, dict):
                    orientation = {
                        "roll": rot.get("Roll", 0.0),
                        "pitch": rot.get("Pitch", 0.0),
                        "yaw": rot.get("Yaw", 0.0),
                    }
                else:
                    orientation = {"roll": 0, "pitch": 0, "yaw": 0}
            except UnrealRemoteError:
                orientation = {"roll": 0, "pitch": 0, "yaw": 0}
        else:
            # Actor-level functions
            try:
                loc = ue._call_function(obj_path, "K2_GetActorLocation")
                if isinstance(loc, dict):
                    position = {
                        "x": loc.get("X", 0.0),
                        "y": loc.get("Y", 0.0),
                        "z": loc.get("Z", 0.0),
                    }
                else:
                    position = {"x": 0, "y": 0, "z": 0}
            except UnrealRemoteError:
                position = {"x": 0, "y": 0, "z": 0}

            try:
                rot = ue._call_function(obj_path, "K2_GetActorRotation")
                if isinstance(rot, dict):
                    orientation = {
                        "roll": rot.get("Roll", 0.0),
                        "pitch": rot.get("Pitch", 0.0),
                        "yaw": rot.get("Yaw", 0.0),
                    }
                else:
                    orientation = {"roll": 0, "pitch": 0, "yaw": 0}
            except UnrealRemoteError:
                orientation = {"roll": 0, "pitch": 0, "yaw": 0}

    # Try to read velocity (GetVelocity is on AActor)
    linear_velocity = {"x": 0, "y": 0, "z": 0}
    if not is_component:
        try:
            vel = ue._call_function(obj_path, "GetVelocity")
            if isinstance(vel, dict):
                linear_velocity = {
                    "x": vel.get("X", 0.0),
                    "y": vel.get("Y", 0.0),
                    "z": vel.get("Z", 0.0),
                }
        except UnrealRemoteError:
            pass

    # Estimate linear acceleration from position delta
    linear_acceleration = {"x": 0, "y": 0, "z": 0}
    angular_velocity = {"x": 0, "y": 0, "z": 0}

    if prev_sample and dt > 0:
        prev_pos = prev_sample["position"]
        prev_vel = prev_sample.get("_velocity", {"x": 0, "y": 0, "z": 0})
        cur_vel = {
            "x": (position["x"] - prev_pos["x"]) / dt,
            "y": (position["y"] - prev_pos["y"]) / dt,
            "z": (position["z"] - prev_pos["z"]) / dt,
        }
        linear_acceleration = {
            "x": (cur_vel["x"] - prev_vel["x"]) / dt,
            "y": (cur_vel["y"] - prev_vel["y"]) / dt,
            "z": (cur_vel["z"] - prev_vel["z"]) / dt,
        }

        prev_ori = prev_sample["orientation"]
        angular_velocity = {
            "x": _angle_diff(orientation["roll"], prev_ori["roll"]) / dt,
            "y": _angle_diff(orientation["pitch"], prev_ori["pitch"]) / dt,
            "z": _angle_diff(orientation["yaw"], prev_ori["yaw"]) / dt,
        }
        velocity_for_next = cur_vel
    else:
        velocity_for_next = {"x": 0, "y": 0, "z": 0}

    # Apply coordinate frame transforms
    want_local = frame in ("local", "both")
    want_world = frame in ("world", "both")

    if want_local:
        lin_vel_local = _world_to_local(linear_velocity, orientation)
        lin_accel_local = _world_to_local(linear_acceleration, orientation)
        ang_vel_local = _world_to_local(angular_velocity, orientation)

    result = {
        "timestamp": now,
        "orientation": orientation,  # always world-referenced
        "position": position,        # always world frame
        "_velocity": velocity_for_next,
    }

    if frame == "world":
        result["linear_velocity"] = linear_velocity
        result["linear_acceleration"] = linear_acceleration
        result["angular_velocity"] = angular_velocity
    elif frame == "local":
        result["linear_velocity"] = lin_vel_local
        result["linear_acceleration"] = lin_accel_local
        result["angular_velocity"] = ang_vel_local
    else:  # both
        result["linear_velocity"] = linear_velocity
        result["linear_acceleration"] = linear_acceleration
        result["angular_velocity"] = angular_velocity
        result["linear_velocity_local"] = lin_vel_local
        result["linear_acceleration_local"] = lin_accel_local
        result["angular_velocity_local"] = ang_vel_local

    return result


def _quat_to_euler(x: float, y: float, z: float, w: float) -> dict:
    """Delegates to transforms.quat_to_euler."""
    return quat_to_euler(x, y, z, w)


def _angle_diff(a: float, b: float) -> float:
    """Shortest angular difference in degrees — delegates to transforms."""
    return angle_diff(a, b)


def _world_to_local(vec_xyz: dict, orientation: dict) -> dict:
    """Delegates to transforms.world_to_local."""
    return world_to_local(vec_xyz, orientation)


def format_sample_human(sample: dict, label: str, frame: str = "world") -> str:
    """Format a sample for human-readable terminal output."""
    o = sample["orientation"]
    p = sample["position"]
    a = sample["linear_acceleration"]
    w = sample["angular_velocity"]
    line = (
        f"\r[{label}] "
        f"pos=({p['x']:8.1f}, {p['y']:8.1f}, {p['z']:8.1f})cm  "
        f"ori=({o['roll']:7.2f}, {o['pitch']:7.2f}, {o['yaw']:7.2f})°  "
    )
    if frame == "local":
        line += (
            f"ω_L=({w['x']:7.1f}, {w['y']:7.1f}, {w['z']:7.1f})°/s  "
            f"a_L=({a['x']:7.0f}, {a['y']:7.0f}, {a['z']:7.0f})cm/s²"
        )
    elif frame == "both":
        al = sample["linear_acceleration_local"]
        wl = sample["angular_velocity_local"]
        line += (
            f"ω=({w['x']:7.1f}, {w['y']:7.1f}, {w['z']:7.1f})°/s  "
            f"a=({a['x']:7.0f}, {a['y']:7.0f}, {a['z']:7.0f})cm/s²  "
            f"ω_L=({wl['x']:7.1f}, {wl['y']:7.1f}, {wl['z']:7.1f})°/s  "
            f"a_L=({al['x']:7.0f}, {al['y']:7.0f}, {al['z']:7.0f})cm/s²"
        )
    else:
        line += (
            f"ω=({w['x']:7.1f}, {w['y']:7.1f}, {w['z']:7.1f})°/s  "
            f"a=({a['x']:7.0f}, {a['y']:7.0f}, {a['z']:7.0f})cm/s²"
        )
    return line


def format_sample_csv(sample: dict, header: bool = False,
                      frame: str = "world") -> str:
    """Format a sample as CSV."""
    if header:
        cols = (
            "timestamp,"
            "pos_x,pos_y,pos_z,"
            "roll,pitch,yaw,"
            "accel_x,accel_y,accel_z,"
            "gyro_x,gyro_y,gyro_z,"
            "vel_x,vel_y,vel_z"
        )
        if frame == "both":
            cols += (",accel_local_x,accel_local_y,accel_local_z,"
                     "gyro_local_x,gyro_local_y,gyro_local_z,"
                     "vel_local_x,vel_local_y,vel_local_z")
        return cols
    o = sample["orientation"]
    a = sample["linear_acceleration"]
    w = sample["angular_velocity"]
    p = sample["position"]
    v = sample["linear_velocity"]
    row = (
        f"{sample['timestamp']:.6f},"
        f"{p['x']:.4f},{p['y']:.4f},{p['z']:.4f},"
        f"{o['roll']:.4f},{o['pitch']:.4f},{o['yaw']:.4f},"
        f"{a['x']:.4f},{a['y']:.4f},{a['z']:.4f},"
        f"{w['x']:.4f},{w['y']:.4f},{w['z']:.4f},"
        f"{v['x']:.4f},{v['y']:.4f},{v['z']:.4f}"
    )
    if frame == "both":
        al = sample["linear_acceleration_local"]
        wl = sample["angular_velocity_local"]
        vl = sample["linear_velocity_local"]
        row += (
            f",{al['x']:.4f},{al['y']:.4f},{al['z']:.4f},"
            f"{wl['x']:.4f},{wl['y']:.4f},{wl['z']:.4f},"
            f"{vl['x']:.4f},{vl['y']:.4f},{vl['z']:.4f}"
        )
    return row


def format_sample_json(sample: dict) -> str:
    """Format a sample as JSON line (excludes internal fields)."""
    out = {k: v for k, v in sample.items() if not k.startswith("_")}
    return json.dumps(out)


def main():
    parser = argparse.ArgumentParser(
        description="Stream IMU data from UE actors/components/bones via Remote Control API")
    parser.add_argument("--actor", required=True,
                        help="Actor path or name substring")
    parser.add_argument("--component", default="",
                        help="Component name/class filter on the actor")
    parser.add_argument("--bone", default="",
                        help="Bone name for skeletal mesh components")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Sample rate in Hz (default: 10)")
    parser.add_argument("--duration", type=float, default=0,
                        help="Duration in seconds (0 = indefinite, default: 0)")
    parser.add_argument("--format", "-f", default="human",
                        choices=["human", "csv", "json"],
                        help="Output format (default: human)")
    parser.add_argument("--frame", default="world",
                        choices=["world", "local", "both"],
                        help="Coordinate frame for accel/velocity/angular-vel: "
                             "'world' (default), 'local' (body frame, like a "
                             "real IMU), or 'both'")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=30010)
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    ue = UnrealRemote(host=args.host, http_port=args.port)
    if not ue.ping():
        print("Connection failed!", file=sys.stderr)
        sys.exit(1)

    try:
        target = resolve_target(ue, args.actor, args.component, args.bone)
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    interval = 1.0 / args.rate
    label = target["label"]

    if args.format == "human":
        frame_label = {"world": "world frame", "local": "local/body frame",
                       "both": "world + local frames"}[args.frame]
        print(f"Streaming IMU from: {label} @ {args.rate} Hz ({frame_label})")
        print(f"Object path: {target['object_path']}")
        if target["bone_name"]:
            print(f"Bone: {target['bone_name']}")
        print("Press Ctrl+C to stop.\n")
    elif args.format == "csv":
        print(format_sample_csv({}, header=True, frame=args.frame))

    prev_sample = None
    prev_time = time.time()
    start_time = prev_time
    sample_count = 0

    try:
        while True:
            now = time.time()
            dt = now - prev_time

            sample = read_imu_sample(ue, target, prev_sample, dt,
                                     frame=args.frame)
            sample_count += 1

            if args.format == "human":
                print(format_sample_human(sample, label, frame=args.frame),
                      end="", flush=True)
            elif args.format == "csv":
                print(format_sample_csv(sample, frame=args.frame))
            elif args.format == "json":
                print(format_sample_json(sample))

            prev_sample = sample
            prev_time = now

            if args.duration > 0 and (now - start_time) >= args.duration:
                break

            # Sleep for remainder of interval
            elapsed = time.time() - now
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    if args.format == "human":
        elapsed = time.time() - start_time
        print(f"\n\nStreamed {sample_count} samples in {elapsed:.1f}s "
              f"({sample_count / elapsed:.1f} Hz actual)")


if __name__ == "__main__":
    main()
