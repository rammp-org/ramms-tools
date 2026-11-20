# ramms-tools

Python tools for interacting with RAMMS-based Unreal Engine projects via the
[Remote Control API](https://dev.epicgames.com/documentation/en-us/unreal-engine/remote-control-for-unreal-engine).

## Installation

```bash
# From the repo root (editable / development install)
pip install -e .

# Or install directly from GitHub (default branch -- moves under you)
pip install git+https://github.com/rammp-org/ramms-tools.git

# Pin to a release tag instead, for robots and other projects that depend on
# this one. Tags are what lets a consumer upgrade deliberately.
pip install 'git+https://github.com/rammp-org/ramms-tools.git@v0.2.1'

# With EXR support (for camera capture replay)
pip install -e '.[exr]'
```

## Requirements

- Python ≥ 3.10
- NumPy ≥ 1.21. The floor deliberately matches what Ubuntu 22.04 / ROS 2 Humble
  ships (`python3-numpy` 1.21.5), so installing this package onto a robot does
  **not** upgrade the system NumPy -- doing so breaks the system `python3-scipy`,
  which is built against NumPy < 1.25. Newer NumPy (including 2.x) is supported
  and used on development machines; nothing here requires it.
- Unreal Engine running with the **Remote Control API** plugin enabled (default port 30010)
  - UE 5.6+ refuses remote function calls that are not allow-listed; the RAMMS project ships
    `Config/DefaultRemoteControl.ini` listing the bridges, robot-component functions and engine
    getters these tools call (see [Remote Control allow-list](#remote-control-allow-list)). Add an
    entry there when you call something new.
- For robot control: the **RammsCore** plugin with `URammsCoreBridge`
- For any robot's controls, drive modes and curb crossing (`ramms_tools.control`, `ramms_tools.robot`,
  `ramms-control`, `ramms-curb`): a pawn with a `URammsRobotControlSurfaceComponent` (RammsCore)
- For robot sensing (`ramms_tools.sensing`, `ramms-sensors`): RammsCore's `URammsSensingBridge`, and
  **RammsStreaming** for the camera frames
- For moving traffic (`ramms-worldgen traffic`): the **RammsCrowd** plugin on a traffic level
- For skeletal pose control: the **RammsCore** plugin with `URammsSkeletalPoseComponent`
- For UI features: the **RammsUI** plugin with `URammsRemoteBridge`
- For binary streaming: the **RammsStreaming** plugin (TCP port 30030)
- For world generation: the **RammsWorldGen** plugin with `URammsWorldGenBridge`
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

### Robots: control surface, drive modes, curb crossing, seat

Every robot pawn carries a `URammsRobotControlSurfaceComponent` that lists all
its controls under dotted ids (`drive.forward`, `drive.mode`, `curb.assist`,
`curb.status`, `seat.height`, `camera.next` ...). `ramms_tools.control` drives
any of them without robot-specific code; `ramms_tools.robot` adds the drive
modes, the curb crossing and the seat on top.

```python
from ramms_tools.robot import Robot

robot = Robot.find()                          # first robot in the world (or Robot.find("PoweredFront"): a unique part of its name)
cs = robot.surface                            # ramms_tools.control.ControlSurface
for a in cs.axes("Curb"):                     # ControlAxis: kind, units, range, enum_labels, read_only, paired_axis
    print(a.id, a.kind, a.enum_labels)
cs.set_enum("curb.assist", "On")              # Enum by label or index
print(cs.status("curb.status"))               # StatusText(text='Ready', severity='Info')
cs.trigger("camera.next")                     # Action
robot.drive(forward=0.3, turn=0.0, seconds=2) # re-sent at 10 Hz for 2 s, then released (see "Held commands")
with cs.keep_alive({"drive.forward": 0.2}) as ka:   # or keep commands alive in the background; released on exit
    ...

print(robot.modes(), robot.mode)              # [DriveMode(id='curb', ...), ...], 'differential'
robot.set_mode("holonomic")                   # waits for the hand-over; ModeError (with the reason) if refused
robot.drive(forward=0.2, strafe=0.3, seconds=2)     # drive.strafe exists in holonomic modes

curb = robot.curb                             # Gen1Curb or Gen2Curb, chosen by component class
print(curb.phase(), curb.status_line())       # 'Idle', StatusText('Ready', 'Info')
run = curb.cross(on_event=print)              # start, hold the stick (full) while it runs; run.succeeded, run.reason, run.stats
if robot.generation == "gen2":
    d = curb.detect_step_ahead()              # Detection(found, reason, profile=CurbProfile(step_height_cm, ...))
    print(curb.plan(), curb.measured_tilt(), curb.stability_margin())
    robot.seat.set_pivot(0.0, 45.0)           # seat pivot (fore/aft, height cm); set_tilt, reset, outline
```

A crossing moves only while the rider's stick (`drive.forward`) is held
forward; let go, it pauses. `cross()` starts it -- the controller selects the
`curb` drive mode itself; do not select that mode by hand first, which starts
(or, with nothing to cross, refuses and hands back) a crossing by itself -- and
holds the stick at `forward` (default 1.0, full; 0.25..1.0, anything else is
refused) only while the crossing is under way and the robot is in the `curb`
drive mode. The stick's sender reads the phase and the drive mode in the same
request as each send, so it lets go in the round that sees the crossing Done,
Aborted or Faulted -- at most one send lands after the end, and it is released
in the next request -- however long your `on_event` / `should_stop` callbacks
take, and it does not drive the previous mode while the robot is still being
handed over to the curb mode. It holds again if a Gen2 crossing retries a fault
by itself (waited for while its status line says "Faulted, trying again"). A
Gen2 crossing goes at the stick's share of its planned speed, a Gen1 one at its
planned speeds once the stick is past the engage threshold (0.25).

`cross()` does not take over a crossing it did not start -- the rider's, curb
assist's or another client's: a Remote stick outranks the rider's own, who
could then no longer pause or cancel it by letting go -- it returns a `CurbRun`
with `already_running` set, unless `join=True`. `wait_until_done(hold_forward=...)`
does the waiting part on a crossing started otherwise. `CurbProfile` /
`CurbTerrain` edits are sent as edited (`begin`, `check_feasibility`,
`begin_on_terrain`).

#### Held commands: the deadman, and older sims that latch

A `Remote` / `Autonomy` command outranks local input (keyboard, gamepad,
touch) for 0.3 s after each send. A **Continuous** control (`drive.forward`,
`drive.turn`, `drive.strafe`, jog axes ...) that is not sent again:

| Sim | What the robot does |
|---|---|
| with the control-surface deadman (RammsCore from 2026-11-15) | springs back to its default 0.3 s after the last Remote / Autonomy send |
| older | **keeps doing the last command** until it is released, set again, or local input moves that control |

So re-send held commands at >= 10 Hz -- `hold()`, `keep_alive()`,
`drive(seconds=)` do -- and release them at the end. Each round goes out as one
Remote Control request (`/remote/batch`, via `UnrealRemote.call_many`): the
engine serves about one request a frame, so N controls sent one request each
would each be refreshed only every N frames, past the deadman's window at a low
frame rate (an unfocused editor runs at a few frames a second). Without a batch
route the client falls back to one request per control and says so; a round
slower than 0.3 s is warned about with its request count. A `KeepAlive` can
also read a condition with each round (`SendGate`) and let go the moment it is
false. ramms-tools releases what
it holds when a hold ends, raises or is interrupted; at interpreter exit it
stops every running `KeepAlive` and releases every Continuous control set once
(`ControlSurface.set`, `drive(once=True)`, `hold(release=False)`,
`KeepAlive.stop(release=False)`) that was not released
(`ramms_tools.control.release_all()`); the CLIs also do it on SIGINT, SIGTERM
and SIGHUP. `drive()` without `seconds` raises: `once=True` sends a single
command explicitly (an older sim keeps it until `stop()`). Nothing can release
after a hard kill (SIGKILL, power or network loss): on an older sim the last
command then stays. Position / Velocity / Enum controls hold their target until
released on every sim. `ramms-control set` takes a Continuous control only with
`--hold SECONDS`.

Out parameters come back beside `ReturnValue` in Remote Control's responses;
`UnrealRemote.call_full()` / `RemoteObjectProxy.call_full()` return the whole
response (`{"OutSeverity": "Info", "ReturnValue": "Ready"}`) where `call()`
keeps only the return value.

RAMMP Gen1 (the curb pawn and the Chaos chair) is in the public project. The
**RAMMP Gen2 (lift-drive)** robot is only in levels built with the private
assets; everything here works on whichever robot
is in the world, and Gen2-only calls say so on a Gen1.

### Sensing: ToF, sonar, IMU and cameras

`ramms_tools.sensing` adds sensors to a robot at runtime through the
`URammsSensingBridge`, reads them, and decodes its cameras' RMSS streams:

```python
from ramms_tools.sensing import CameraStreams, Sensing, SensorMount

robot.sensors.apply_rig("rammp_gen1")         # preset: front ToF, sonar, IMU, RGB-D camera ("chair" for the Chaos chair)
robot.sensors.add(SensorMount("left_sonar", "Sonar", parent="chassis", location=(30, 30, 25), rotation=(0, 60, 0)))
tof = robot.sensors.read("front_tof", measure=True)
print(tof.min_distance_cm, tof.grid())        # (rows, columns) cm, NaN = no hit
imu = robot.sensors.read("imu")
print(imu.linear_acceleration, imu.units, imu.linear_acceleration_ms2())   # raw cm/s^2, converted

sensing = Sensing()
status = sensing.start_streams()              # RMSS server + capture (needs PIE or a game)
cam = next(s for s in robot.sensors.list() if s.name == "front_camera")
with CameraStreams.from_sensor(cam, port=status.port) as streams:   # this robot's camera: its actor and channel
    f = streams.grab()                        # one capture: {"color": (H,W,3) RGB, "depth": (h,w) float32 cm,
                                              #  "motion": (h,w,2) float32 px/frame, "mask": (H,W) uint8}
    print(f["mask"].classes)                  # {3: "Car", ...}; sensing.segmentation_classes() for the table
```

Stream ids carry only the camera's name, and the Gen1 / Gen2 rigs and the
`chair` preset all name theirs `front_camera`: pick a robot's camera by its
actor (and channel), never by name alone -- `CameraStreams.from_sensor`, or
`actor=` on `CameraStreams` / `CameraStreams.from_status` (which refuses a name
several actors stream). A camera placed by hand can stream under its stream
source's name: `from_sensor` / `from_status` take the stream ids the sim
reports. Every camera frame's metadata names its `actor` and its capture
`frame` number; `grab()` returns a `FrameSet` of one capture -- the newest
complete one, or at the timeout the newest with the most roles, with
`complete` False and `missing` naming the rest (never frames of two captures
passed off as one set). `StreamStatus.port` is None while the RMSS server is
not running. The server sends a connection no frames until it subscribes;
`CameraStreams` and `ImuStream` subscribe on connect.

Adding / removing sensors and starting streams need a play world (PIE or a
game) on sims from the fix round; older sims keep a mount added in the editor
world and build it at BeginPlay (`"built": false`). A parent component the
actor does not have is an error. Sensor names are 1..64 characters of
`A-Z a-z 0-9 _ -` (no `.`), not ending in `_depth`, `_dmv`, `_seg` or `_stream`.
`start_streams` cannot turn disk writes on (the sim refuses it over the bridge);
`stop_streams` undoes only what the streams set up.

Every sensor looks along its local +X; mount locations are cm from the parent
component (a MuJoCo body such as `chassis`, or `""` for the root), rotations
(pitch, yaw, roll) degrees; a sonar's `sonar_cone_deg` is a half-angle. A
camera with base channel C streams colour on C (`camera/<name>/color`, FrameRGB
bgra8), depth on C+100 (`camera/<name>/depth`, float32 cm), motion on C+200
(`motion/<name>`, FrameMotion rg32f) and its mask on C+300 (`mask/<name>`,
FrameData mono8, with the class table in the metadata);
`ramms_tools.streaming.frames` decodes any of them to numpy. The RAMMP Gen1 and
Gen2 curb pawns ship with a rig baked in (`imu`, `front_tof`, `front_sonar`,
`front_camera`); the presets (`ramms_tools/sensing/rigs.py`) put the same sensors
on robots without one: `rammp_gen1` (that rig) and `chair` (the Chaos chair on the
generated levels). The mounts are plausible front-of-robot positions, not measured
from the hardware. `apply_rig` also takes a JSON file of mounts.

**IMU from the physics engine, no component needed.** An IMU reading for any body
comes straight from the engine that simulates it:

```python
print(robot.sensors.imu_bodies())             # [{"name": "chassis", "engine": "mujoco", ...}, ...]
r = robot.sensors.body_imu("chassis", offset=(50, 0, 15), wait=2.0)   # a point on the chassis, cm
print(r.source, r.linear_acceleration, r.angular_velocity, r.linear_velocity, r.sim_time)
```

The reading takes its body by name: a MuJoCo body, a component, or a Chaos
skeletal bone. MuJoCo and Chaos bodies are sampled every physics step at the
sensor's point, so lever-arm and centripetal terms are included. The
acceleration is what an accelerometer reads: +g up at rest, about 0 in free
fall. The IMU components use the same engines: an IMU sensor reports
`source` `mujoco`, `chaos` or `kinematic`, the last being transform differencing
for bodies no engine simulates. A point's first read starts tracking it, so it
has no sample yet; `wait=` re-reads until one arrives. `sim_time` is None until
there is a sample.

**Every physics-step sample.** The sim keeps about 2 s of each tracked point's
samples (1024 at MuJoCo's 500 Hz). Read them all over Remote Control with a
cursor, or have the sim stream them over RMSS:

```python
from ramms_tools.sensing import ImuStream

for s in sensing.follow_imu_samples(robot.actor_path, "chassis", period=0.1, duration=5.0):   # ReadImuSamplesJson
    print(s.seq, s.sim_time, s.linear_acceleration, s.angular_velocity)    # cm/s^2 (gravity in), deg/s, sensor frame

info = sensing.start_imu_stream(robot.actor_path, "imu")       # an IMU sensor or a body; channel >= 1000
with ImuStream.from_info(info) as stream:                      # FrameData imu_f64x18: the samples of each game tick
    for s in stream.samples(duration=5.0):
        ...
sensing.stop_imu_stream(robot.actor_path, "imu")
```

`read_imu_samples(since=...)` is one call (`next` is the cursor to pass back,
`dropped` the samples lost before the oldest kept); the follower's and the
stream's `dropped` count what was lost. Each sample has seq, sim_time,
acceleration, angular velocity and angular acceleration (sensor frame), velocity
and orientation (world). An IMU stream's id is `imu/<actor name>/<target>` (the
body's name for target `""`). Subscribe to a stream promptly (`ImuStream` does on
connect): the sim stops one that no RMSS client has subscribed to for 30 s and
frees its point. Sim limits: Remote Control callers get 32 of a world's 64
tracked IMU points, a world runs at most 8 IMU streams, body / target names are
at most 256 characters, a stream's port is clamped to 1..65535, and
`ReadImuSamplesJson` returns complete samples only.

### Remote Control allow-list

UE 5.6+ refuses remote function calls that are not allow-listed. The source of
truth is the RAMMS project's `Config/DefaultRemoteControl.ini`: it allows the
static bridges as whole classes, and on components and actors **only the
functions** these tools call -- never a whole component class, which would also
expose everything it inherits (`K2_DestroyComponent` ...) to anyone on the
network -- and no Blueprint child classes. A project of its own needs the same
entries (`+CustomAllowedRemoteFunctionCalls=(ClassPath="/Script/...",FunctionName=("SetControl"),bAllowChildClasses=False)`).
`FunctionName` is optional in UE, so its value goes in parentheses: a bare
`FunctionName="SetControl"` does not parse, and the entry then allows every
function of the class.
A `/remote/batch` request is checked call by call: it needs nothing more.

| Class | Functions ramms-tools calls | Used by |
|---|---|---|
| `/Script/RammsCore.RammsCoreBridge`, `/Script/RammsCore.RammsSensingBridge` | the whole (static) bridge | discovery; `sensing`, `ramms-sensors` |
| `/Script/RammsCore.RammsRobotControlSurfaceComponent` | `GetControlSurfaceJson`, `SetControl`, `TriggerControl`, `ReleaseControl`, `GetControlValue`, `GetControlTarget`, `GetControlText` | `control`, `robot`, `ramms-control`, `ramms-curb`, TUI Robot page |
| `/Script/RammsCore.RammsDriveModeSelector` | `GetAvailableModeIds`, `GetActiveModeId`, `SetActiveMode`, `CycleMode`, `IsSwitchingMode`, `GetSwitchStatus`, `GetMeasuredMotion` | drive modes; the curb stick's gate |
| `/Script/RammsCore.RammsRammpGen1CurbController` | `StartCrossing`, `Abort`, `GetPhase`, `GetStatus`, `GetStatusLine`, `GetStats`, `GetCrossing` | RAMMP Gen1 curb crossing |
| `/Script/RammsCore.RammsCurbNegotiationController` | `CheckFeasibility`, `BeginNegotiation`, `BeginNegotiationOnTerrain`, `NegotiateStepAhead`, `DetectStepAhead`, `DetectTerrainAhead`, `Abort`, `IsNegotiating`, `GetPhase`, `GetPhaseAxle`, `GetStatusText`, `GetStatusLine`, `GetDetectionText`, `GetActiveTerrain`, `GetActiveProfile`, `GetElapsedSeconds`, `GetMeasuredTiltDeg`, `GetPlannedTiltDeg`, `GetBaseHeight`, `GetDriveSpeed`, `GetWheelProgress`, `GetLegHeights`, `GetWheelClearance`, `GetWheelStatus`, `GetStabilityMargin`, `GetPlanDescription`, `GetStepLog`, `GetScheduleLines`, `GetRunStats` | RAMMP Gen2 curb crossing |
| `/Script/RammsCore.RammsSeatMechanismController` | `SetPivotTarget`, `SetTiltDeg`, `ResetSeat`, `GetPivot`, `GetTiltDeg`, `GetReachableOutline` | RAMMP Gen2 seat |
| `/Script/RammsCrowd.RammsTrafficSpawner` | `SpawnTraffic`, `DespawnTraffic`, `GetSpawnedCount` | `ramms-worldgen traffic` |
| `/Script/RammsCrowd.RammsTrafficSignalClock` | `GetSignalState`, `GetControlledCrossingCount` | `ramms-worldgen traffic` |
| `/Script/Engine.Actor` (child classes allowed) | `K2_GetActorLocation`, `K2_GetActorRotation` | traffic car poses, `ramms-imu` |

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

### World Generation

`ramms_tools.worldgen` drives the **RammsWorldGen** plugin — seed-driven
pedestrian / wheelchair environments (street grids, curb ramps, entrances,
hazards, parks, indoor layouts) that come with a ground-truth manifest of
routes, clear widths and accessibility issues. It works in the editor and in
a running game.

```python
from ramms_tools.worldgen import WorldGen

wg = WorldGen()                              # localhost:30010
print(wg.world())                            # {'map': 'Map_Gen_Downtown', 'play': False, 'generators': 1, ...}
print([p["name"] for p in wg.describe_presets()])

gen = wg.generator()                         # first ARammsWorldGenerator in the level (or wg.spawn("PRE_Downtown", 7))
gen.set_params(HazardDensity=6.0, NonCompliance=0.5)   # overrides on top of the preset
gen.generate(seed=42)                        # blocks until the manifest is published

m = gen.manifest()                           # ramms_tools.worldgen.Manifest
print(m.summary())
for f in m.issues:                           # non-compliant features
    print(f.kind, f.subtype, f.issue, f.measure, "/", f.threshold, f.location)
route = m.routes[0]
print(route.name, route.min_clear_width, route.waypoints(spacing=100))
start, yaw = m.suggested_start, m.suggested_start_yaw   # where to put the wheelchair

# Street crossings: one Crosswalk feature per crossing; step-free ones have a curb ramp at both ends
for cw in m.crosswalks(step_free=True):
    a, b = cw.crossing_ends()                # centre line at the two curbs
    print(cw.subtype, a, b)                  # Ramped / SteepRamp (OneRamp, NoRamps with step_free=False)

# Another asset set: palettes swap the meshes, the layout stays the preset's
print(wg.palettes())                         # ['/RammsWorldGen/Palettes/PAL_Default.PAL_Default', ...]
gen.set_palette("PAL_UrbanVerse")            # "" = back to the preset's palette
gen.generate(seed=42)
m = gen.manifest()
print(m.palette, m.sources())                # ... {'': 206, 'urbanverse': 209}
for f in m.features_of("Prop"):
    print(f.subtype, f.asset, f.source_uid)  # mesh placed, and its UrbanVerse uid when it came from there

# Glass (PAL_UrbanVerse): every placement gets its own frost / blur, tint, ripple and opacity, drawn
# from the seed and its position. Cosmetic only: the layout and manifest stay the same.
gen.set_params(GlassVariation=1.0, GlassFrostChance=0.6)  # 0 = every copy looks like its asset
print(gen.vary_glass())                      # re-roll now without regenerating; returns instances written

# A dataset: one manifest per seed, written locally (drops earlier overrides first)
for seed, path in wg.batch("PRE_Park", range(1, 51), out_dir="data/park"):
    print(seed, path)
for seed, path in wg.batch("PRE_Downtown", range(20), out_dir="data/dt_uv", palette="PAL_UrbanVerse"):
    print(seed, path)                        # PRE_Downtown_PAL_UrbanVerse_<seed>.json
```

Graded streets, curb heights and rough ground (the RammsWorldGen README's
"Graded streets and rough ground"), and the traffic on them:

```python
gen.set_terrain(max_grade=0.08, crown=0.02, cross_slope=0.02, noncompliant_cross_slope=0.2,
                curb_min=10, curb_max=20, roughness=0.5, potholes=2, humps=0.3)   # range-checked
gen.generate(seed=7)
m = gen.manifest()
print(m.terrain_summary())    # max_running_grade, cross_slope_range, curb_height_range, curb ramps, roughness counts
print(m.grades_by_side(), m.curb_heights(), m.roughness_counts())
print(m.traffic_summary())    # roads, length, intersections by subtype, signals, crosswalks

from ramms_tools.worldgen import Traffic
t = Traffic()                 # a traffic level, in PIE or a game
print(t.count(), t.signal_state())
t.despawn(); t.spawn()
for car in t.cars(limit=5):
    print(car.name, car.location, car.yaw)
```

The friendly names map to `MaxStreetGrade`, `RoadCrown`, `SidewalkCrossSlope`,
`NonCompliantCrossSlopeChance`, `CurbHeight`, `CurbHeightMin`, `CurbHeightMax`,
`SurfaceRoughness`, `PanelLength`, `HeaveDensity`, `CrackDensity`,
`CoverDensity`, `PavingPatchDensity`, `PotholeDensity`, `SpeedHumpChance` and
`RootHeaveChance` (`TERRAIN_PARAMS` / `ROUGHNESS_PARAMS`); the parameter names
work too, and `flat=True` zeroes them first.

`Manifest` also loads exported files (`Manifest.load("x.json")`) and offers
`features_of(kind)`, `features_near(point, radius)`, `features_on_route(route)`,
`routes_narrower_than(width)`, `crosswalks(step_free)`, `assets(kind)`,
`sources()` and `issues_by_kind()` — no engine needed. For placed meshes,
`location`/`size`/`yaw` describe the object's box (`location` is the centre of
its base on the ground, not the mesh pivot; trees are located by their trunk); `asset` is the mesh placed and `source` where it came
from (`urbanverse:<uid>|<category>`), both empty in manifests from before they were recorded.
`set_params` takes numbers (bools as 0/1) and, for text parameters such as
`Palette` and `FacadeStyles`, strings. The parameter names are listed in the plugin's
README ([rammp-org/ramms-worldgen](https://github.com/rammp-org/ramms-worldgen),
a submodule of ramms-sim at `Plugins/RammsWorldGen`).

#### Editor, PIE and packaged builds

The same calls work in all three; the bridge targets the PIE / game world when
one is running, and the editor world otherwise.

- **Editor / PIE**: Remote Control is on by default (port 30010).
- **Packaged or `-game` builds**: Unreal turns the Remote Control web server
  off outside the editor. Launch with `-RCWebControlEnable`, e.g.
  `Ramms.exe Map_Gen_Downtown -RCWebControlEnable -windowed`.
  Cook the maps you want to drive (`-map=Map_Gen_Downtown+...`); the presets and
  graphs ship in the plugin and are cooked with them. Palettes are only reached
  through soft references, so ramms-sim cooks `/RammsWorldGen/Palettes` and
  `/Game/UrbanVerse` explicitly (`DirectoriesToAlwaysCook` in
  `Config/DefaultGame.ini`); `set_palette` in a packaged build only finds palettes cooked that way.
- Generators spawned with `spawn(label=...)` are also tagged with the label, so
  `generator("MyLabel")` works in packaged builds, which have no actor labels.
- **Headless** (cluster / CI): use `-RenderOffScreen`, not `-nullrhi` —
  Remote Control does not start when the engine cannot render.
- `open_map` loads the level in the editor and travels (`OpenLevel`) in PIE or
  a game; actor paths change afterwards, so call `generators()` again.

`examples/worldgen_smoke.py` runs discovery, generation, determinism,
parameter overrides, graded streets (skipped on an indoor scene or a plugin
without the terrain parameters; `--no-graded`), a palette switch and back (`--palette`, default
`PAL_UrbanVerse`, skipped when it is not imported) with its glass variation
(which must leave the layout unchanged), the recorded assets,
manifest export, a small batch and a map travel against
whatever is listening, and exits non-zero on the first failure:

```bash
python examples/worldgen_smoke.py                    # editor / PIE on localhost
python examples/worldgen_smoke.py --host 10.0.0.5    # a packaged build elsewhere
```

## CLI Tools

All tools are installed as console scripts when you `pip install` the package.

### `ramms-tui` — Interactive Dashboard

A rich terminal UI (built with [Textual](https://textual.textualize.io/)) for
monitoring and controlling the RAMMP Gen1, Kinova arm, gripper, IMU data and
any robot's control surface in real-time.

```bash
ramms-tui                          # Launch the TUI (auto-connects to UE)
ramms-tui --host 192.168.1.10      # Connect to a remote UE instance
```

**Pages:**

| Tab | Description |
|-----|-------------|
| Dashboard | Overview cards for arm, gripper, RAMMP Gen1, system status, and IMU |
| Arm | 7 joint controls with ±buttons, input fields, Home/Refresh; gripper state, Open/Close/Toggle, finger angle controls |
| RAMMP Gen1 | Dynamic motor controls (angular & linear) |
| IMU | Real-time streaming with target selection and coordinate frame toggle |
| Stream | RMSS stream monitor — connect to streaming server (subscribes to every channel on connect; the server sends nothing before a subscribe), view per-channel FPS/bandwidth/dropped frames |
| Robot | Any robot's control surface: robot and drive-mode selectors, curb panel (phase, status coloured by severity, assist / style, cross / stop, Gen2 detection), enum selectors and action buttons per group, live readback, sensors with their latest readings |

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

### `ramms-pose` — Skeletal Pose Control

Controls any actor with a `URammsSkeletalPoseComponent` (kinematic bone posing
via `UPoseableMeshComponent`). Works with arbitrary skeletal meshes — not limited
to a specific robot model.

```bash
ramms-pose --list                              # Find poseable actors
ramms-pose --describe                          # Show joints, meshes, and current values
ramms-pose --set-all 0 15 -90 30               # Set all joint targets
ramms-pose --set Shoulder 45.0                 # Set joint by name
ramms-pose --set-index 0 45.0                  # Set joint by index
ramms-pose --home                              # All joints to 0
ramms-pose --sweep 0 -90 90 --period 4         # Sweep joint 0 between -90°..90°
ramms-pose --interactive                       # REPL mode
```

**Interactive commands:**

| Command | Description |
|---------|-------------|
| `state` | Show current joint values and targets |
| `set <idx> <value>` | Set joint by index |
| `setname <name> <value>` | Set joint by name |
| `setall <v0> <v1> ...` | Set all joints (validates count) |
| `home` | All joints to 0 |
| `snap` | Snap to targets (bypass interpolation) |
| `sweep <idx> <lo> <hi> [period]` | Sweep joint (Ctrl+C to stop) |
| `meshes` | List poseable meshes |

Supports both revolute (rotation) and prismatic (translation) joints.
Joint names are auto-populated from physics asset constraints.

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

### `ramms-rammp-gen1` — RAMMP Gen1 Motor Control

```bash
ramms-rammp-gen1 --describe                           # Show motors and positions
ramms-rammp-gen1 --set-angular motor_swing_arm_l 5.0   # Set angular motor target (degrees)
ramms-rammp-gen1 --set-linear dw_main_plate_l 2.0     # Set linear motor target (cm)
ramms-rammp-gen1 --home                               # All motors to 0
ramms-rammp-gen1 --interactive                        # REPL mode
```

### `ramms-imu` — IMU Data Streaming

```bash
ramms-imu --actor BP_RammpGen1_Ramms_C_0                             # Stream from actor
ramms-imu --actor BP_RammpGen1_Ramms_C_0 --component ArmSkMesh       # From a named component
ramms-imu --actor BP_RammpGen1_Ramms_C_0 --component ArmSkMesh --bone end_effector  # From a bone/socket
ramms-imu --actor BP_RammpGen1_Ramms_C_0 --rate 30 --format csv      # 30 Hz CSV output
ramms-imu --actor BP_RammpGen1_Ramms_C_0 --format json --duration 10 # 10s of JSON lines
ramms-imu --actor BP_RammpGen1_Ramms_C_0 --frame local               # Body-frame output
ramms-imu --actor BP_RammpGen1_Ramms_C_0 --frame both                # World + local frames
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
monitor camera frames and other binary data. The stats show each channel's
stream id and role; `--save` writes motion vectors as `.npy` (float32,
H x W x 2), segmentation masks as `.png` (Pillow) or `.npy`, and IMU stream
messages (FrameData `imu_f64x18`) as `.npy` (float64, n x 18).

```bash
# Receive and display stats
ramms-stream                                     # Subscribe to all channels
ramms-stream -c 0,100,200,300                    # One camera's colour, depth, motion and mask
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

The server sends a connection no frames until it has subscribed:
`subscribe(channels)` for those channels, `subscribe()` for every channel.
`StreamClient` warns once when it is polled without a subscription
(`StreamSender`, which only sends, needs none).

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
            print(f"  Ch {ch}: {cs.fps():.1f} fps, {cs.frames} frames")

# Send images to UE
with StreamSender("127.0.0.1", 30030) as sender:
    import numpy as np
    img = np.zeros((720, 1280, 4), dtype=np.uint8)
    sender.send_numpy_image(channel=0, array=img,
                            stream_id="camera/front/color",
                            name="Front Color",
                            group="front")
```

#### Stream Associations & Metadata

Streams can carry association metadata so that UE automatically links related
data (e.g. color ↔ depth from the same camera):

| Metadata Key | Purpose | Example |
|--------------|---------|---------|
| `stream_id` | Unique identifier used by UE instead of numeric channel | `"camera/wrist/color"` |
| `group` | Shared label linking related streams (color ↔ depth) | `"wrist"` |
| `role` | Stream purpose: `color`, `depth`, `mask`, `infrared` | `"depth"` |
| `name` | Human-readable display name for UE UI | `"Wrist Depth"` |

All send methods (`send_image`, `send_depth`, `send_depth_uint16`, `send_motion`)
accept these as keyword arguments. When provided, they **override** any
matching keys already present in the `metadata` dict, so callers can safely
pass a base metadata dict and override specific fields:

```python
base_meta = {"intrinsics": {...}, "transform": {...}}
sender.send_depth(channel=100, depth_bytes=data, width=w, height=h,
                  metadata=dict(base_meta),  # copy to avoid mutation
                  stream_id="camera/wrist/depth",
                  group="wrist", role="depth",
                  name="Wrist Depth")
```

#### Depth Formats

The sender supports two depth encodings:

| Method | Format | Units | UE Texture | Use Case |
|--------|--------|-------|------------|----------|
| `send_depth()` | float32 | cm | PF_R32_FLOAT | UE captures, high precision |
| `send_depth_uint16()` | uint16 | mm | PF_G16 | ROS depth, bandwidth-efficient |

The UE sink auto-detects the format from the `fmt` metadata field (`"float32"`
vs `"16uc1"` / `"uint16"` / `"mono16"`) and creates the appropriate texture.
Downstream consumers (PGM shader, camera widget materials) automatically apply
the correct unnormalization and unit conversion.

```python
# Float32 depth in centimeters (from UE captures)
sender.send_depth(channel=100, depth_bytes=depth_f32.tobytes(),
                  width=w, height=h)

# Uint16 depth in millimeters (from ROS / external sensors)
sender.send_depth_uint16(channel=100, depth_bytes=depth_u16.tobytes(),
                         width=w, height=h,
                         stream_id="camera/wrist/depth",
                         group="wrist")
```

#### Color Formats

The sink accepts three color formats, auto-detected from the `fmt` metadata:

| Format | Bytes/Pixel | Notes |
|--------|-------------|-------|
| `bgra8` | 4 | Native UE format (no conversion) |
| `rgba8` | 4 | Swizzled to BGRA on receipt |
| `rgb8` | 3 | Expanded to BGRA (R↔B swap + alpha=255) |

`send_numpy_image()` auto-detects `rgb8` vs `bgra8` from the array shape
(3 vs 4 channels).

### `ramms-stream-test` — Stream Test & Capture Replay

Sends test frames or replays captured camera data to UE via the RMSS streaming
server. Useful for testing the streaming pipeline without a live camera source.

Each stream is identified by a **stream ID** rather than a raw numeric channel.
Stream IDs are generated from configurable templates that support `{name}`
(camera label) and `{index}` (0-based camera index) placeholders:

```bash
# Send synthetic color-bar test frames
ramms-stream-test --synthetic -n 100

# Replay all cameras (auto-discovers subdirectories, generates stream IDs)
ramms-stream-test --capture-dir ./Saved/CameraCaptures/BP_RammpGen1_Ramms_C_0

# Include depth data and filter to a specific camera
ramms-stream-test --send-depth --capture-dir ./Saved/CameraCaptures/BP_RammpGen1_Ramms_C_0 --camera FR_GeminiCamera

# Custom stream ID templates
ramms-stream-test --capture-dir ./captures --color-id "robot/{name}/rgb" --depth-id "robot/{name}/depth"

# Override name placeholder for synthetic mode
ramms-stream-test --synthetic --name front_cam --color-id "test/{name}/color"

# Control chunk size and frame rate
ramms-stream-test --capture-dir ./captures --prefetch 30 --fps 30 -n 0
```

| Flag | Description |
|------|-------------|
| `--synthetic` | Send animated color-bar test frames |
| `--capture-dir` | Replay EXR+JSON data from CameraCapture plugin output |
| `--send-depth` | Include depth data when replaying captures |
| `--camera` | Filter to a specific camera name |
| `--color-id` | Stream ID template for color (default: `camera/{name}/color`) |
| `--depth-id` | Stream ID template for depth (default: `camera/{name}/depth`) |
| `--mask-id` | Stream ID template for masks (default: `mask/{name}`) |
| `--motion-id` | Stream ID template for motion (default: `motion/{name}`) |
| `--name` | Default `{name}` value when no camera label exists |
| `--prefetch` | Chunk size for double-buffered loading (default: 30, 0=all) |
| `-n` / `--num-frames` | Max frames to send (default: 300, 0=all) |
| `--loop` | Loop playback continuously |

In capture replay mode, each discovered camera generates a pair of stream IDs
(e.g. `camera/FL_Capture/color` and `camera/FL_Capture/depth`) sharing the
same group for automatic association in UE. Numeric channel IDs are assigned
internally and are not exposed to the user.

Requires the `exr` optional dependency for capture replay:
```bash
pip install 'ramms-tools[exr]'   # OpenEXR + Imath
```

### `ramms-bag` — Replay a recorded RAMMP bag into RammpUI

Plays a ROS 2 bag recorded on the robot into a running RammpUI (PIE, `-game` or
packaged) through the interfaces the robot itself uses: RMSS for the cameras,
Remote Control for detected objects and the robot display's joints. No ROS
install is needed: the rosbag2 sqlite files are read directly and the messages
decoded by `ramms_tools.rosbag`.

```bash
ramms-bag info BAG                          # topics, cameras, objects, and what a replay will and will not show
ramms-bag play BAG                          # real time
ramms-bag play BAG --rate 2 --start 30 --end 60 --loop
ramms-bag play BAG --overlay segmentation   # the overlay slot: wrist (default) | segmentation | segmenter | none
ramms-bag play BAG --positions derived      # no 3D from the tracker: RammsUI derives markers from depth
ramms-bag play BAG --verify                 # afterwards, check the 3D markers landed where the bag says
ramms-bag play BAG --dry-run                # send nothing; count what would be sent
```

BAG is a rosbag2 directory (`metadata.yaml` plus `.db3` files). A zipped bag
has to be extracted first; `ramms-bag` prints the `unzip` command.

**What goes where:**

| Bag | UI |
|---|---|
| `/scene_camera/color/image_raw` | RMSS ch 0 `camera/scene/color`, with intrinsics (CameraInfo) and the camera's pose (TF) |
| `/scene_camera/depth/image_rect_raw` (16UC1 mm) | RMSS ch 1 `camera/scene/depth`, the same |
| `/perception/object_detections` (the tracker) | `UpdateDetectedObjects`: normalized 2D boxes, and 3D markers from each object's camera-frame pose, size and orientation, with `time_since_detection_s` |
| `/joint_states` | `SetAllJointTargets` on the robot display, by joint name (`arm_joint_N` to `link_{N-1}` in degrees; the gripper's knuckle to its fingers) |
| overlay slot (`--overlay`) | `wrist`: the wrist camera with its moving pose (ch 2 `engine/overlay`, ch 4 `engine/overlay/depth`). `segmentation`: the scene frame with the segmenter's masks, one value per class (ch 3 `engine/overlay/segmentation`), and the segmenter's own boxes on the overlay (2D only, ids `seg:...`); the overlay's depth channel is emptied first, so an earlier wrist replay's depth cannot place those boxes in 3D. `segmenter`: the segmenter's own rendering (needs Pillow) |

- **Camera poses** are the camera body frame in `base_link`, sent `relative`,
  so the UE stream receiver stands for the robot's base. Both camera streams
  carry them, where the UI bridge sends them on depth only; RammsUI takes either.
- **Staleness** (`time_since_detection_s`) is counted from the last time the
  5 Hz segmenter actually saw the object: a raw box overlapping the tracked one,
  whatever class the segmenter gave it. A tracker carrying an object forward is
  not a sighting, so an occluded object's age keeps growing. On the `_vo` bag
  this matches the producer's own values within 0.04 s 91% of the time.
- **On exit** (the end, Ctrl-C or an error) the UI is put back: detections are
  cleared, the overlay window closed and the joints restored.
- **`--verify`** re-pushes the newest set of 3D objects and reads back, through
  the editor's Python, where the world markers landed. Each must be within 1 cm
  of the bag's own pose, composed with the stream receiver's transform. With
  `--positions derived`, within 1 cm of where RammsUI must derive it from the
  last depth frame sent (the distance to the tracker's own 3D centre is printed
  as well). Joint targets are read back too.
- **`--loop`** repeats the window; each pass lasts the whole window (at least
  50 ms), however early its last message.
- **The Nov 19 2026 main bag's 3D is frozen.** It has no `odom->base_link` in
  `/tf`, so each tracked object keeps the position it had when the bag began
  while the camera drives on. `info` and `play` warn about this. Use the bag
  re-processed with visual odometry (`_vo`) for 3D markers, or
  `--positions derived`.

### `ramms-ui-bridge-test` — RAMMP UI Bridge Interfaces

Exercise the `RammsBridgeReceiver` surfaces the RAMMP UI bridge drives —
panels and buttons, detected-object and world-state views, the camera feed, the
RMSS camera streams, and the UI events that come back — without needing ROS 2
or the bridge itself. Useful for UI work, and for reproducing a bridge problem
in isolation.

```bash
ramms-ui-bridge-test discover                 # which preset is live, what it exposes, stream IDs
ramms-ui-bridge-test selftest                 # non-interactive transport checks, exits non-zero on failure
ramms-ui-bridge-test panel --scenario error   # idle|selection|progress|paused|error|stress|empty
ramms-ui-bridge-test objects --select cup_1   # tappable boxes, with one highlighted
ramms-ui-bridge-test camera open              # OpenCameraFeed / CloseCameraFeed
ramms-ui-bridge-test stream --seconds 10 --overlay   # synthetic scene colour + depth + overlay
ramms-ui-bridge-test events --follow          # watch button presses and object taps live
ramms-ui-bridge-test styles                   # every button style, read back and checked
ramms-ui-bridge-test pois --tap               # POIs, and a tap dimming the others
ramms-ui-bridge-test markers                  # world markers derived from 2D + depth (editor PIE)
ramms-ui-bridge-test joints                   # the robot display: joints, monitor, status bar
ramms-ui-bridge-test joints --check           # stow, activate, drive, park -- all read back
ramms-ui-bridge-test joints --set link_1=40   # set joints by name; every other joint held
ramms-ui-bridge-test walkthrough              # guided pass over every surface
```

Three things it guards, all of which fail silently against a real editor:

- **A preset entry bound to nothing** — Remote Control answers HTTP 200 for a
  call that reaches no object at all, so every push looks like it worked and
  the UI never changes. This happens whenever the level holding
  `RammsBridgeReceiver` is moved or replaced: the preset keeps a binding to the
  old path. `discover` prints what each function is bound to and exits non-zero
  when any is orphaned, `selftest` asserts it before running checks that would
  otherwise all "pass", and any call to an unbound function is refused.


- **Preset functions are called by their exposed label, not their C++ name** —
  `Poll UIEvents`, not `PollUIEvents`, which returns
  `"Unable to resolve the preset field."` The tool discovers the mapping from
  the preset rather than hardcoding it.
- **An unrecognised parameter is accepted with HTTP 200 and arrives empty** —
  UE ignores parameters it does not recognise, so the function receives an empty
  string and `UpdatePanel` clears the panel instead of updating it, with no
  error anywhere. Every call is checked against the argument names the preset
  declares before it is sent. (Case does *not* matter: `PanelJSON`, `PanelJson`
  and `paneljson` all land, so the check is case-insensitive and only a genuinely
  unknown name is refused.)

It also encodes `box`/`centroid` as `{"X":..,"Y":..,"Z":..,"W":..}` objects
rather than bare arrays, which UE's JSON struct converter rejects for
`FVector4`/`FVector2D`.

Requires the editor running with Remote Control (HTTP on 30010) and a level
containing `RammsBridgeReceiver`. The preset is **discovered**: it picks the one
that exposes the bridge API *with its bindings intact*, because presence alone
is not enough — an orphaned entry is still listed and still answers HTTP 200.
Projects disagree on the name (RammpUI binds it on `Content/RCPS`, the Ramms
project has only `RCP_RammsBridge`), so any hardcoded default is wrong
somewhere. `--preset` pins one, and a failed discovery names every candidate it
rejected and why. Layout, world state and the camera feed still need eyes:
those scenarios print what you should see, and the tap round-trip is what gets
asserted.

**Button styles and POIs** are read back from the running HUD (PIE or a game;
in the editor with nothing playing there is no HUD and they are skipped):

- `styles` pushes one button per style, then reads each live `RAMMS_Button`'s
  `PanelButton`, label text, label opacity and brush tint. It checks:
  - every pushed button is on the panel exactly once and shows its own label;
  - names RAMMS_Button doesn't know (`normal`, absent, unknown, a different case)
    are drawn like the style its case-insensitive switch falls back to;
  - its styles (`valid`, `invalid`, `unavailable`, `cancel`, `error`) look
    different from each other.

  The contract's `warning` (the Orchestrator's Cancel buttons) drawn like
  `normal` is printed as a warning, not a failure. `--scenario style-fallback`
  sends unknown, empty and differently-cased names.
- `pois` pushes objects to the POI layer and checks one live POI per object,
  none left from an earlier push, all opaque. With `--tap` it asks for a tap on
  a POI, checks the event came from the POI layer (no `stream_id`, no
  `tap_point`) and that every other POI dims to the layer's
  `UnselectedPOIOpacity`, then asks for a tap on the background and checks they
  all come back.
- `selftest` runs both (POIs without the tap). The walkthrough adds both,
  with a tap.
- `events` labels each object tap with where it came from:
  - `[camera image]`: it carries `tap_point`, or a camera stream id such as
    `camera/scene/color`;
  - `[3D view]`: a bare source such as `scene`;
  - `[POI layer]`: neither. A 3D tap on an object with no source looks the
    same, so this one is a best guess.

Live widgets are found by probing their object names (`RAMMS_Button_C_<k>`,
`RAMMS_POI_C_<k>`), one `/remote/batch` call per 1024 names.
- The number `k` only grows, and a replaced widget stays around with an empty
  `Slot` until garbage collection, so only widgets with a `Slot` count.
- Buttons are rebuilt on every push, so the probe stops shortly after the
  newest.
- POIs are kept per object, so a long-lived one can sit far below newer ones,
  and they are scanned in full.

**World markers derived from 2D** (`markers`) check the depth fallback in
`URammsDetectionWorldMarkers`. An object sent without `position_m` is placed by
unprojecting its centroid through its camera's intrinsics, at the range read from
that camera's depth stream, through the stream's pose.

- **Run it in an editor play session.** It drives the marker component through the
  editor's Python (Project Settings > Python > Remote Execution), not Remote Control:
  the project's Remote Control allow-list covers only the bridge receiver, and should.
  `--editor-project` picks the editor (default `RammpUI`).
- **It streams its own camera** over RMSS, on `test/markers/color` and
  `test/markers/depth`, with intrinsics and a pose in the metadata the stream bridge
  already reads. The values are deliberately asymmetric (fx ≠ fy, an off-centre
  principal point, a roll), so a swapped axis or a dropped angle can't pass. Depth is a
  wall at 2 m, a box at 1.2 m and a patch with no return.
- **What it checks:**
  - whether a component in the level places markers from bridge pushes (one
    `position_m` object pushed through the receiver);
  - the whole app path: a 2D object pushed through the receiver in the bridge's
    own form (`{"X", "Y"}` pixels) gets a derived marker from the level's
    component;
  - a derived position from current depth with nothing but the component itself;
  - the projection at the centre, a corner, the opposite corner and the wall,
    against positions computed independently;
  - no position where depth has no return, or for a point outside the image;
  - the bridge's `source: "scene"` alias. That needs the scene camera to
    carry intrinsics, a pose and depth: `markers --scene` streams the test
    camera on the scene camera's own stream ids (taking over the scene view
    while it runs), which decides it and adds an object exactly as the bridge
    sends one;
  - one-object payloads in every form the bridge uses: `centroid` and `box` as
    arrays and as `{"X", ...}` objects, normalized and in pixels, `source` on the
    object or only on the push;
  - sent `position_m` in both frames, as controls.
- **It uses a temporary component of its own** for the derive and payload checks,
  and holds a raw-depth request while it needs one. It destroys the one and releases
  the other afterwards, also after a run that died half way. The level's projection
  manager gives the test camera a projector of its own; that is removed too (not
  under `--scene`, where the projector is the real scene camera's).
- **Its own stream ids by default.** Before RammsUI's world-marker fixes, the stream
  bridge read a stream's intrinsics and size only the first time it saw it, so a
  scene stream first sent without intrinsics (the synthetic video in `walkthrough`
  and `stream` sends none) never gained them. It now re-registers a stream when
  either changes.

**The robot display's joints** are checked automatically. They go through
`SetAllJointTargets` (preset label `Set All Joint Targets`) on the display's
`RammsSkeletalPose` component. Each send is a full positional vector, built by joint
name from the component's own list: a short array is accepted but writes only the
first few joints, which on the RAMMP display are the base linkages. `joints --check`,
and the joints section of `selftest`, do this:

1. stow the arm, then make it active, then stow it again;
2. turn the drive wheels, then stop them.

After each step it reads back the joint targets, the skeleton's applied values, the
joint-motion monitor's verdict (`ArmPoseState`, `bWheelsMoving`) and the status bar's
arm and chair text (`Stowed`/`Active`, `DRIVING`/`PARKED`). A status bar it cannot
find counts as one failure, and its indicator checks are skipped rather than passed.
At the end it puts the targets back, except the wheels, which stay where they
stopped: spinning them back would make the chair read `DRIVING` again. A restore
that does not land is a failure too.

Where the joints are sent depends on what is running. The tool reads the editor's
`UnrealEdEngine_0.PlayWorld` (the live play world, or empty when nothing plays), or
finds `GameEngine_0` (a game). If neither answers, it sends nothing.

- **Editor, playing (PIE):** sent straight to the play session's component
  (`/Game/Maps/UEDPIE_<n>_<map>...`, as `PlayWorld` names it), not through the
  preset. A preset entry names the
  editor's level and resolves there even during play. Measured: a preset call wrote
  the editor's copy of the actor and the running robot stayed where it was. The same
  applies to any other sender driving the preset during PIE.
- **Editor, not playing:** refused, because the editor's own level answers on
  those paths and the targets would be written into the actor that the map saves.
  `joints` exits non-zero; `selftest` skips the joint checks and names the reason
  on its last line.
- **A game (packaged, or `UnrealEditor -game`):** sent through the preset, because
  that is the route an external sender uses.

Two Remote Control quirks are handled:

- The preset reports **no arguments** for `SetAllJointTargets` (it leaves out
  `const TArray<double>&` parameters), so the tool pins the argument name, `Values`.
- If the joint entry is left dangling after the display actor is swapped,
  `discover` says what to rebind it to.

Don't run the joint checks while another sender is streaming joints: its next frame
overwrites the test's targets.

### `ramms-control` — Any Robot's Controls

```bash
ramms-control robots                                # robots in the world, generation, drive mode
ramms-control describe --group Curb                 # controls: kind, units, range, choices
ramms-control get drive.mode curb.status --target   # Enum labels, Text with severity, numbers
ramms-control set curb.assist On                    # Enum by label or index
ramms-control set seat.height 45                    # Position: holds until `release`
ramms-control set drive.forward 0.3 --hold 2        # Continuous: only with --hold (re-sent at 10 Hz, then released)
ramms-control trigger camera.next
ramms-control release seat.height
ramms-control watch curb.phase curb.status --rate 5 # or --group Seat; --json for JSON lines
ramms-control drive --forward 0.4 --turn 0.1 --seconds 3   # --strafe in holonomic modes
ramms-control mode                                  # list; `mode holonomic` switches (waits); `mode --cycle`
```

`--source` sets the `ERammsControlSource` (default `Remote`; a Remote command
outranks local input for 0.3 s after each send). Ctrl+C, SIGTERM and SIGHUP
release whatever is held before exiting (exit code 130); see "Held commands"
for what the sim does with a command that is not re-sent.

### `ramms-curb` — Curb Crossing

Works on whichever robot is in the world: the RAMMP Gen1 curb controller or
the RAMMP Gen2 (lift-drive) curb negotiation (private assets only).

```bash
ramms-curb status                          # phase, status line + severity, assist, drive mode, key sensing
ramms-curb watch --rate 5 --changes        # tilt, base height, speed, margin (Gen2); step (Gen1)
ramms-curb assist on
ramms-curb style rolling                   # Gen1: "Stop and step" or "Rolling"
ramms-curb detect                          # Gen2: step and terrain ahead (Gen1 has no look-ahead)
ramms-curb drive --speed 0.3 --seconds 4   # approach
ramms-curb go --wait                       # start a crossing, hold the stick while it runs, phase log + stats; exit 1 unless Done
ramms-curb go --wait --forward 0.8 --on-interrupt pause    # a Gen2 crossing at 80 % of its planned speed
ramms-curb go --wait --join                # hold the stick for a crossing already under way (the rider's ...)
ramms-curb abort
ramms-curb stats
ramms-curb plan                            # Gen2: plan, schedule, step log, planned tilt
ramms-curb seat --pivot 0,45 --tilt -5     # Gen2 seat (--pivot=-10,45 for a negative fore/aft), --reset, --outline
```

`go` starts the crossing; the controller selects the curb drive mode itself.
A crossing moves only while the stick (`drive.forward`) is held forward:
`go --wait` holds it at `--forward` (default 1.0, full; 0.25..1.0 -- below the
engage threshold a crossing never moves, and anything else is refused) only
while the crossing is under way in the curb drive mode. The sender reads both
with each send and lets go in the round that sees the crossing done, stopped or
faulted (at most one send after it, released at once), holding it again if a
Gen2 crossing retries. Without `--wait`, the crossing waits, paused, for a
stick (`go --wait --join` holds it). A crossing already under way that `go` did not start (the rider's, curb
assist's, another client's) is not taken over unless `--join`; `--no-hold`
only watches it. Ctrl+C / SIGTERM / SIGHUP during `go --wait` release the stick
and abort the crossing (`--on-interrupt pause` leaves it paused:
`go --wait --join` goes on).

### `ramms-sensors` — Robot Sensing

```bash
ramms-sensors rig list                                 # presets (rammp_gen1, chair)
ramms-sensors rig apply rammp_gen1 [--replace]         # or a .json file of mounts
ramms-sensors add left_sonar --kind sonar --parent chassis --at 30,30,25 --rot 0,60,0 --cone 10
ramms-sensors add cam2 --json '{"kind": "camera", "width": 320, "height": 240}' --no-motion
ramms-sensors list                                     # --all: every actor
ramms-sensors read front_tof --measure --grid
ramms-sensors watch --rate 5 --format csv              # table | json | csv
ramms-sensors bodies                                   # bodies an IMU can be read from, and their engine
ramms-sensors imu --body chassis --offset 50,0,15 --rate 20 --format csv   # physics-engine IMU, no component
ramms-sensors imu --body chassis --samples --format csv > imu.csv   # every physics-step sample (Remote Control)
ramms-sensors imu --sensor imu --stream --duration 10              # every sample over RMSS (an IMU stream)
ramms-sensors remove left_sonar                        # rig sensors only
ramms-sensors streams start --every 2 --no-motion      # needs PIE or a game
ramms-sensors streams status | stop
ramms-sensors classes                                  # segmentation classes (mask values)
ramms-sensors camera front_camera --save shots --frames 10      # --robot / --actor: whose front_camera
```

`camera` takes the camera on the robot `--robot` / `--actor` names (the first
robot by default): several robots can have a camera of the same name. It
writes colour `.png` (else `.npy`), depth (float32 cm) and motion `.npy`, masks
`.png` (else `.npy`), a `.json` of each frame's metadata and `classes.json`;
each set is one capture. `imu --samples` / `--stream` print every physics-step
sample (table, `--format json` lines or CSV; the summary goes to stderr);
`--stream` stops the IMU stream at the end unless `--keep-stream`. A negative first value in `--at` / `--rot` needs the `=` form
(`--rot=-20,0,0`). Sonar cones are half-angles; IMU readings are in the
component's units (cm/s^2, deg/s).

### `ramms-worldgen` — World Generation

Drive RammsWorldGen from the shell: list presets and example maps, open a map,
spawn / regenerate / randomize a generator, tweak parameters, fetch or export
the ground-truth manifest, and build datasets of many seeds.

```bash
ramms-worldgen world                               # current map, generator count
ramms-worldgen presets                             # PRE_Downtown, PRE_Suburban, PRE_Park, PRE_Indoor_Home, PRE_Indoor_Office
ramms-worldgen palettes                            # PAL_Default, PAL_UrbanVerse (once imported), ...
ramms-worldgen maps                                # /Game/Maps/Gen/Map_Gen_*
ramms-worldgen open Map_Gen_Suburban               # editor: load the level; game: travel to it
ramms-worldgen spawn --preset PRE_Downtown --seed 7 --generate
ramms-worldgen generate --seed 42                  # regenerate, wait, print a summary
ramms-worldgen generate --preset PRE_Park --set FoliageDensity=2 --player-start
ramms-worldgen generate --seed 42 --palette PAL_UrbanVerse   # UrbanVerse street furniture and cars
ramms-worldgen generate --palette ""               # back to the preset's palette
ramms-worldgen randomize
ramms-worldgen status
ramms-worldgen params                              # resolved parameter block
ramms-worldgen params --set HazardDensity=6 --set NonCompliance=0.5
ramms-worldgen params --set FacadeStyles=dt01,dt03  # text values are passed as strings
ramms-worldgen params --clear                      # back to the preset's values
ramms-worldgen glass --set GlassFrostChance=0.6    # re-roll the per-placement glass looks only
ramms-worldgen manifest --issues --routes          # summary + non-compliant features + routes
ramms-worldgen manifest --out downtown_42.json     # save locally
ramms-worldgen manifest --export                   # the engine writes it under Saved/WorldGen too
ramms-worldgen batch --preset PRE_Park --seeds 1-50 --out data/park
ramms-worldgen batch --preset PRE_Downtown --seeds 1-20 --palette PAL_UrbanVerse --out data/dt_uv
ramms-worldgen inspect data/park/PRE_Park_3.json --issues   # offline
ramms-worldgen cleanup
ramms-worldgen terrain                             # graded-street / roughness parameters now
ramms-worldgen terrain --max-grade 0.08 --crown 0.02 --cross-slope 0.02 --noncompliant-cross-slope 0.2 \
                       --curb-min 10 --curb-max 20 --generate --seed 7
ramms-worldgen terrain --flat --roughness 0.5 --heaves 2 --cracks 3 --covers 1 --paving 1 --potholes 2 \
                       --humps 0.3 --root-heaves 0.5
ramms-worldgen manifest --terrain --traffic        # grades, cross slopes, curb heights, ramps, roughness; roads, signals
ramms-worldgen inspect data/x.json --terrain       # the same offline
ramms-worldgen traffic status                      # spawners, cars, signal phase (traffic level, PIE or game)
ramms-worldgen traffic spawn | despawn | cars --limit 10 | signals
```

`--generator NAME` picks a generator by actor path, name or label when a level
has several; `--json` switches `world`, `presets`, `palettes`, `status`, `params`, `glass`,
`manifest` and `inspect` to machine-readable output. `batch` starts from the
preset (earlier overrides are dropped unless `--keep-overrides`) and stops on an
unknown `--set` name; `generate` and `params` warn and carry on.

## Common Options

All CLI tools support:

| Flag | Description |
|------|-------------|
| `--host` | UE Remote Control host (default: `127.0.0.1`) |
| `--port` | UE Remote Control port (default: `30010`) |
| `--verbose` / `-v` | Show raw HTTP request/response traffic |

## License

MIT
