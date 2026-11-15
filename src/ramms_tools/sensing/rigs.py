"""
Sensor rig presets: named lists of SensorMounts to put on a robot at runtime.

    from ramms_tools.sensing import apply_rig
    apply_rig(robot, "rammp_gen1")              # or robot.sensors.apply_rig("rammp_gen1")
    apply_rig(robot, "my_rig.json")             # a JSON list of mounts (or {"mounts": [...]})

Every sensor traces / looks along its local +X (forward on these robots);
location is cm from the parent component, rotation (pitch, yaw, roll) degrees,
pitch negative = looking down. A sonar's sonar_cone_deg is the beam's half-angle.

The RAMMP Gen1 and Gen2 curb pawns ship with a rig baked into their Blueprints (a
URammsSensorRigComponent "SensorRig": imu, front_tof, front_sonar, front_camera);
"rammp_gen1" is that same rig, for a Gen1 pawn without it. "chair" puts the same
sensors on the Chaos chair the generated levels spawn. The mounts sit at the front
of each robot, clear of its own geometry (checked live: the ToF and sonar see the
ground and the curb ahead, not the robot). They are plausible positions, not
measured from the hardware -- replace them once the real mounts are known.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, Union

from .bridge import SensingError, SensorMount, as_mount

# name -> list of mount dicts (the AddSensorJson mount JSON); see the module docstring for where they come from.
RIG_PRESETS: dict[str, list[dict]] = {
    # RAMMP Gen1 MuJoCo pawn (BP_RammpGen1PoweredFront_Ramms): the rig baked into it, on its "chassis" body (whose
    # origin is at the rear of the chair, ~9 cm above the ground; the front of the chair is ~87 cm ahead of it).
    "rammp_gen1": [
        {"name": "imu", "kind": "IMU", "parent": "chassis", "location": [35.0, 0.0, 15.0], "rotation": [0.0, 0.0, 0.0]},
        {"name": "front_tof", "kind": "ToF", "parent": "chassis", "location": [88.0, 0.0, 11.0], "rotation": [-35.0, 0.0, 0.0],
         "rows": 8, "columns": 8, "h_fov_deg": 45.0, "v_fov_deg": 45.0, "max_range_cm": 400.0},
        {"name": "front_sonar", "kind": "Sonar", "parent": "chassis", "location": [88.0, 0.0, 6.0], "rotation": [-5.0, 0.0, 0.0],
         "sonar_max_range_cm": 500.0, "sonar_cone_deg": 15.0},
        {"name": "front_camera", "kind": "Camera", "parent": "chassis", "location": [86.0, 0.0, 30.0], "rotation": [-20.0, 0.0, 0.0],
         "width": 640, "height": 480, "fov_deg": 87.0, "color": True, "depth": True, "motion": True, "segmentation": True,
         "channel": -1},
    ],
    # The Chaos chair (BP_RammpGen1_Ramms): mounts on the actor's root component.
    "chair": [
        {"name": "front_tof", "kind": "ToF", "parent": "", "location": [55.0, 0.0, 30.0], "rotation": [-20.0, 0.0, 0.0],
         "rows": 8, "columns": 8, "h_fov_deg": 45.0, "v_fov_deg": 45.0, "max_range_cm": 400.0},
        {"name": "front_sonar", "kind": "Sonar", "parent": "", "location": [55.0, 0.0, 35.0], "rotation": [0.0, 0.0, 0.0],
         "sonar_max_range_cm": 500.0, "sonar_cone_deg": 15.0},
        {"name": "imu", "kind": "IMU", "parent": "", "location": [0.0, 0.0, 30.0], "rotation": [0.0, 0.0, 0.0]},
        {"name": "front_camera", "kind": "Camera", "parent": "", "location": [45.0, 0.0, 100.0], "rotation": [-25.0, 0.0, 0.0],
         "width": 640, "height": 480, "fov_deg": 87.0, "color": True, "depth": True, "motion": True, "segmentation": True,
         "channel": -1},
    ],
}

RIG_DESCRIPTIONS: dict[str, str] = {
    "rammp_gen1": "RAMMP Gen1 MuJoCo pawn: the rig its Blueprint ships with (front ToF, front sonar, IMU, RGB-D camera on the chassis)",
    "chair": "Chaos chair (BP_RammpGen1_Ramms): front ToF, front sonar, IMU and an RGB-D camera on the root",
}


def rig_names() -> list[str]:
    return sorted(RIG_PRESETS)


def rig_mounts(preset: Union[str, Path]) -> list[SensorMount]:
    """The mounts of a preset by name, or of a JSON file (a list of mounts, or {"mounts": [...]})."""
    key = str(preset)
    if key in RIG_PRESETS:
        return [SensorMount.from_dict(dict(m)) for m in RIG_PRESETS[key]]
    p = Path(key)
    if p.suffix.lower() == ".json" and p.is_file():
        data = json.loads(p.read_text(encoding="utf-8"))
        items = data.get("mounts", []) if isinstance(data, dict) else data
        if not isinstance(items, list):
            raise SensingError(f"{p}: expected a list of mounts or {{\"mounts\": [...]}}")
        return [SensorMount.from_dict(dict(m)) for m in items]
    raise SensingError(f"no rig preset {key!r} (presets: {', '.join(rig_names())}; or a .json file of mounts)")


def apply_rig(robot_or_actor, preset: Union[str, Path, Iterable[Union[SensorMount, dict]]], replace: bool = False,
              sensing=None) -> list[dict]:
    """
    Put a rig's sensors on a robot (a ramms_tools.robot.Robot, or an actor path with `sensing` or a default
    Sensing on localhost). replace=True also removes the rig sensors the preset does not have.
    Returns the AddSensorJson results.
    """
    from .bridge import Sensing
    mounts = rig_mounts(preset) if isinstance(preset, (str, Path)) else [as_mount(m) for m in preset]
    if hasattr(robot_or_actor, "actor_path"):
        actor = robot_or_actor.actor_path
        sensing = sensing or Sensing(robot_or_actor.ue)
    else:
        actor = str(robot_or_actor)
        sensing = sensing or Sensing()
    return sensing.apply_rig(actor, mounts, replace=replace)
