# ramms-tools

Python tools for interacting with RAMMS-based Unreal Engine projects via the
[Remote Control API](https://dev.epicgames.com/documentation/en-us/unreal-engine/remote-control-for-unreal-engine).

## Installation

```bash
# From the repo root (editable / development install)
pip install -e .

# Or install directly from GitHub
pip install git+https://github.com/ATDev-Inc/ramms-tools.git
```

## Requirements

- Python ≥ 3.10
- Unreal Engine running with the **Remote Control API** plugin enabled (default port 30010)
- For robot control: the **RammsCore** plugin with `URammsCoreBridge`
- For UI features: the **RammsUI** plugin with `URammsRemoteBridge`
- The **Textual** TUI library (installed automatically with the package)

## Library Usage

```python
from ramms_tools.unreal_remote import UnrealRemote

ue = UnrealRemote()  # localhost:30010

# Discover actors
actors = ue.find_actors(class_filter="StaticMeshActor")

# Find components by class name or instance name (single server-side call)
results = ue.find_actors_by_component("KinovaGen3")
results = ue.find_actors_by_component("ArmSkMesh")  # match by instance name

# Call functions on any UObject
comp = ue.actor(results[0]["component_path"])
comp.call("SetJointTarget", JointIndex=0, TargetAngle=45.0)

# Read properties
angles = comp.call("GetAllJointAngles")

# UI features (requires RammsUI plugin)
ue = UnrealRemote(ui_bridge="/Script/RammsUI.Default__RammsRemoteBridge")
ue.ui_bridge.ShowNotification(Message="Hello!", Level="ERammsNotificationLevel::Info")
```

### Coordinate Frame Transforms

The `ramms_tools.transforms` module provides utilities for working with UE's
left-handed coordinate system:

```python
from ramms_tools.transforms import world_to_local, quat_to_euler, angle_diff

# Convert UE quaternion (from GetSocketTransform) to Euler angles
euler = quat_to_euler(qx, qy, qz, qw)  # → {roll, pitch, yaw} in degrees

# Transform a world-frame vector into a body's local frame
local_vel = world_to_local({"x": vx, "y": vy, "z": vz}, euler)

# Shortest angular difference (handles ±180° wrapping)
delta = angle_diff(170, -170)  # → -20
```

## CLI Tools

All tools are installed as console scripts when you `pip install` the package.

### `ramms-tui` — Interactive Dashboard

A rich terminal UI (built with [Textual](https://textual.textualize.io/)) for
monitoring and controlling the Mebot, Kinova arm, gripper, and IMU data in
real-time.

```bash
ramms-tui                          # Launch the TUI (auto-connects to UE)
ramms-tui --host 192.168.1.10      # Connect to a remote UE instance
```

**Pages:**

| Tab | Description |
|-----|-------------|
| Dashboard | Overview cards for arm, mebot, system status, and IMU |
| Arm | 7 joint controls with ±buttons, input fields, Home/Refresh |
| Mebot | Dynamic motor controls (angular & linear) |
| IMU | Real-time streaming with target selection and coordinate frame toggle |

The IMU page supports **World**, **Local**, or **Both** coordinate frames.
In Local mode, linear velocity, angular velocity, and linear acceleration are
transformed into the body frame of the target object using the current
orientation.

### `ramms-kinova` — Kinova Gen3 Arm Control

```bash
ramms-kinova --describe                          # Show joints and current angles
ramms-kinova --set-joint 0 45.0                  # Set joint 0 to 45°
ramms-kinova --set-all 0 15 180 -150 0 -10 90   # Set all 7 joints
ramms-kinova --home                              # All joints to 0°
ramms-kinova --interactive                       # REPL mode
```

### `ramms-mebot` — Mebot Motor Control

```bash
ramms-mebot --describe                           # Show motors and positions
ramms-mebot --set-angular YawMotor 45.0          # Set angular motor target
ramms-mebot --set-linear LiftMotor 50.0          # Set linear motor target (cm)
ramms-mebot --home                               # All motors to 0
ramms-mebot --interactive                        # REPL mode
```

### `ramms-imu` — IMU Data Streaming

```bash
ramms-imu --actor BP_Mebot_Ramms_C_0                             # Stream from actor
ramms-imu --actor BP_Mebot_Ramms_C_0 --component ArmSkMesh       # From a named component
ramms-imu --actor BP_Mebot_Ramms_C_0 --component ArmSkMesh --bone end_effector  # From a bone/socket
ramms-imu --actor BP_Mebot_Ramms_C_0 --rate 30 --format csv      # 30 Hz CSV output
ramms-imu --actor BP_Mebot_Ramms_C_0 --format json --duration 10 # 10s of JSON lines
ramms-imu --actor BP_Mebot_Ramms_C_0 --frame local               # Body-frame output
ramms-imu --actor BP_Mebot_Ramms_C_0 --frame both                # World + local frames
```

The `--component` flag matches against both the component **class name** and the
component **instance name**, so you can target a specific skeletal mesh by its
name (e.g. `ArmSkMesh`) when an actor has multiple skeletal meshes.

The `--frame` flag controls the coordinate frame for velocity and acceleration:

| Value | Description |
|-------|-------------|
| `world` | (default) All values in UE world coordinates |
| `local` | Velocity & acceleration in the body's local frame |
| `both` | Both world and local values side by side |

Output includes: position, orientation (roll/pitch/yaw), linear velocity,
estimated linear acceleration, and estimated angular velocity.

### `ramms-notify` — UI Notifications

```bash
ramms-notify "Task complete!"
ramms-notify "Low battery" --level warning --duration 8
ramms-notify --demo                              # Demo sequence
ramms-notify --interactive                       # REPL mode
```

### `ramms-status` — Status Panel Control

```bash
ramms-status --battery 0.85
ramms-status --speed 1.5 --mode Autonomous
ramms-status --estop
ramms-status --demo                              # Cycle through states
ramms-status --discover                          # List widgets and actors
```

## Common Options

All CLI tools support:

| Flag | Description |
|------|-------------|
| `--host` | UE Remote Control host (default: `127.0.0.1`) |
| `--port` | UE Remote Control port (default: `30010`) |
| `--verbose` / `-v` | Show raw HTTP request/response traffic |

## License

MIT