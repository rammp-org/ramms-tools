# ramms-tools

Python tools for interacting with RAMMS-based Unreal Engine projects via the
[Remote Control API](https://dev.epicgames.com/documentation/en-us/unreal-engine/remote-control-for-unreal-engine).

## Installation

```bash
# From the repo root (editable / development install)
pip install -e .

# Or install directly from GitHub
pip install git+https://github.com/ATDev-Inc/ramms-tools.git

# With EXR support (for camera capture replay)
pip install -e '.[exr]'
```

## Requirements

- Python ≥ 3.10
- Unreal Engine running with the **Remote Control API** plugin enabled (default port 30010)
- For robot control: the **RammsCore** plugin with `URammsCoreBridge`
- For UI features: the **RammsUI** plugin with `URammsRemoteBridge`
- For binary streaming: the **RammsStreaming** plugin (TCP port 30030)
- The **Textual** TUI library (`pip install 'ramms-tools[tui]'` or `pip install textual`)

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
| Dashboard | Overview cards for arm, gripper, mebot, system status, and IMU |
| Arm | 7 joint controls with ±buttons, input fields, Home/Refresh; gripper state, Open/Close/Toggle, finger angle controls |
| Mebot | Dynamic motor controls (angular & linear) |
| IMU | Real-time streaming with target selection and coordinate frame toggle |
| Stream | RMSS stream monitor — connect to streaming server, subscribe to channels, view per-channel FPS/bandwidth/dropped frames |

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

### `ramms-gripper` — Gripper Control

```bash
ramms-gripper --describe                         # Show gripper state and finger angles
ramms-gripper --open                             # Open the gripper
ramms-gripper --close                            # Close the gripper
ramms-gripper --toggle                           # Toggle open/closed
ramms-gripper --set-fingers 45.0 45.0            # Set individual finger angles (degrees)
ramms-gripper --speed 2.0                        # Set motor speed multiplier
ramms-gripper --interactive                      # REPL mode
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

Output includes: position (m), orientation (roll/pitch/yaw in degrees),
linear velocity (m/s), estimated linear acceleration (m/s²), and estimated
angular velocity (deg/s).

When available, UE physics APIs (`GetVelocity`, `GetPhysicsLinearVelocity`,
`GetPhysicsAngularVelocityInDegrees`) are used for velocity and angular
velocity; otherwise, values are derived from position/orientation deltas.

Signal filtering options:

| Flag | Description |
|------|-------------|
| `--deadzone` | Position change deadzone in cm (default: 0.5, 0 = off) |
| `--ori-deadzone` | Orientation change deadzone in deg (default: 0.5, 0 = off) |
| `--lpf` | Low-pass filter alpha in (0,1], smaller = smoother (default: 0 = off) |

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

### `ramms-stream` — Binary Data Streaming

Connects to the RMSS streaming server (default port 30030) to receive and
monitor camera frames and other binary data.

```bash
# Receive and display stats
ramms-stream                                     # Subscribe to all channels
ramms-stream -c 0,1                              # Subscribe to channels 0 and 1
ramms-stream --save ./captures                   # Save frames to disk

# Test connectivity
ramms-stream --ping
```

For replaying captured frames or sending test data, see
[`ramms-stream-test`](#ramms-stream-test--stream-test--capture-replay).

The streaming system uses a custom TCP binary protocol (port 30030) and requires
the **RammsStreaming** UE plugin.  Add `URammsStreamSourceComponent` to actors
with cameras to stream their frames, and `URammsStreamSinkComponent` to receive
images from external sources.  Use `URammsStreamCameraBridge` (RammsUI) to
automatically pipe received stream data into the UI camera widget pipeline.

#### Compression

The server supports optional **JPEG** compression for RGB frames and **LZ4**
for depth data.  Enable on the UE side by setting
`URammsStreamingSubsystem::bEnableCompression = true` and adjusting
`JpegQuality` (1–100, default 85).

On the Python side, install the optional dependencies for transparent
decompression:

```bash
pip install 'ramms-tools[compression]'   # Pillow + lz4
pip install 'ramms-tools[all]'           # Pillow + lz4 + textual
```

The `StreamClient` auto-decompresses by default (pass `auto_decompress=False`
to disable).  Compression availability can be checked at runtime:

```python
from ramms_tools.streaming import has_jpeg, has_lz4
print(f"JPEG: {has_jpeg()}, LZ4: {has_lz4()}")
```

#### Python streaming library

```python
from ramms_tools.streaming import StreamClient, StreamSender

# Receive frames from UE (auto-decompresses JPEG/LZ4)
with StreamClient("127.0.0.1", 30030) as client:
    client.subscribe(channels=[0])
    for msg in client.iter_messages():
        meta = msg.get_metadata_json()
        # msg.payload contains raw pixel data (BGRA8 / float32)

        # Per-channel stats
        stats = client.get_channel_stats()
        for ch, cs in stats.items():
            print(f"  Ch {ch}: {cs.fps:.1f} fps, {cs.frames} frames")

# Send images to UE
with StreamSender("127.0.0.1", 30030) as sender:
    import numpy as np
    img = np.zeros((720, 1280, 4), dtype=np.uint8)
    sender.send_numpy_image(channel=0, array=img)
```

### `ramms-stream-test` — Stream Test & Capture Replay

Sends test frames or replays captured camera data to UE via the RMSS streaming
server. Useful for testing the streaming pipeline without a live camera source.

```bash
# Send synthetic color-bar test frames
ramms-stream-test --synthetic -n 100

# Replay a single camera's captured data
ramms-stream-test --capture-dir ./Saved/CameraCaptures/Robot/HeadCam

# Replay all cameras from an actor (auto-discovers camera subdirectories)
ramms-stream-test --capture-dir ./Saved/CameraCaptures/BP_Mebot_Ramms_C_0

# Include depth data and filter to specific camera
ramms-stream-test --send-depth --capture-dir ./Saved/CameraCaptures/BP_Mebot_Ramms_C_0 --camera FR_GeminiCamera

# Control chunk size and frame rate
ramms-stream-test --capture-dir ./captures --prefetch 30 --fps 30 -n 0
```

| Flag | Description |
|------|-------------|
| `--synthetic` | Send animated color-bar test frames |
| `--capture-dir` | Replay EXR+JSON data from CameraCapture plugin output |
| `--send-depth` | Include depth channel when replaying captures |
| `--camera` | Filter to a specific camera name |
| `--prefetch` | Chunk size for double-buffered loading (default: 30, 0=all) |
| `--channel` | Base RGB channel ID (default: 0) |
| `--depth-channel` | Base depth channel ID (default: channel+100) |
| `-n` / `--num-frames` | Max frames to send (default: 300, 0=all) |
| `--loop` | Loop playback continuously |

Requires the `exr` optional dependency for capture replay:
```bash
pip install 'ramms-tools[exr]'   # OpenEXR + Imath
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