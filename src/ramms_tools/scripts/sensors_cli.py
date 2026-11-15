#!/usr/bin/env python3
"""
ramms-sensors -- put ToF, sonar, IMU and camera sensors on a robot, read them,
and grab its cameras' colour / depth / motion / mask frames over RMSS.

    ramms-sensors list                                     # sensors on the robot (--all: every actor)
    ramms-sensors rig list                                 # rig presets (rammp_gen1, chair)
    ramms-sensors rig apply rammp_gen1                     # front ToF, sonar, IMU, RGB-D camera
    ramms-sensors rig apply my_rig.json --replace          # mounts from a file; drop rig sensors not in it
    ramms-sensors add left_sonar --kind sonar --parent chassis --at 30,30,25 --rot 0,60,0
    ramms-sensors add tof2 --json '{"kind": "tof", "rows": 4, "columns": 4, "location": [40, 0, 20]}'
    ramms-sensors remove left_sonar
    ramms-sensors read front_tof --measure --grid          # trace now; print the 8x8 zones
    ramms-sensors watch --rate 5 --format table            # every ToF / sonar / IMU (json | csv | table)
    ramms-sensors imu --body chassis --rate 10             # IMU from the physics engine (MuJoCo / Chaos), no component
    ramms-sensors imu --body chassis --samples --format csv > imu.csv   # every physics-step sample, over Remote Control
    ramms-sensors imu --sensor imu --stream --duration 10   # every sample over RMSS (an IMU stream, channel >= 1000)
    ramms-sensors bodies                                   # bodies an IMU can be read from, and their engine
    ramms-sensors streams start --no-motion                # RMSS server + capture (port 30030)
    ramms-sensors streams status
    ramms-sensors streams stop
    ramms-sensors classes                                  # segmentation classes (mask values)
    ramms-sensors camera front_camera --save shots --frames 10      # colour .png, depth / motion .npy, mask .png

Adding / removing sensors and starting streams need a play world (PIE or a
game) on sims from the fix round; the segmentation classes always do. Only rig
sensors (the ones added here) can be removed. Sonar cones are half-angles; IMU
readings are in the component's units (cm/s^2, deg/s).

--robot NAME (or --actor PATH) picks the actor; default the first robot. That
includes `camera`: two robots can both have a "front_camera" (stream ids carry
only the name), so the camera is the one on that actor. Mount locations are cm
relative to the parent component, rotations pitch,yaw,roll degrees; every
sensor looks along its local +X.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

from ramms_tools.control import ControlError
from ramms_tools.robot import Robot, RobotNotFound
from ramms_tools.sensing import (RIG_DESCRIPTIONS, CameraStreams, ImuStream, Sensing, SensingError, SensorMount,
                                 rig_mounts, rig_names, save_frame)
from ramms_tools.streaming.frames import ROLES
from ramms_tools.unreal_remote import UnrealRemoteError

from ._common import (Interrupted, Ticker, add_common, connect, dump_json, exit_signals, finish_parser, fmt_num,
                      interrupted_message, setup_logging)


def _triple(text: str, what: str) -> tuple[float, float, float]:
    try:
        x, y, z = (float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"{what} expects three comma-separated numbers, got {text!r}") from None
    return (x, y, z)


def mount_from_args(a: argparse.Namespace) -> SensorMount:
    """The mount `add` builds: --json (inline or @file) as the base, the flags on top."""
    base: dict = {}
    if a.mount_json:
        text = a.mount_json
        if text.startswith("@"):
            text = Path(text[1:]).read_text(encoding="utf-8")
        base = json.loads(text)
        if not isinstance(base, dict):
            raise SensingError("--json expects one mount object")
    flags = {
        "kind": a.kind, "parent": a.parent, "socket": a.socket,
        "location": _triple(a.at, "--at") if a.at else None, "rotation": _triple(a.rot, "--rot") if a.rot else None,
        "rows": a.rows, "columns": a.columns, "h_fov_deg": a.hfov, "v_fov_deg": a.vfov, "max_range_cm": a.range,
        "sonar_max_range_cm": a.sonar_range, "sonar_cone_deg": a.cone,
        "width": a.width, "height": a.height, "fov_deg": a.fov, "channel": a.channel,
        "color": False if a.no_color else None, "depth": False if a.no_depth else None,
        "motion": False if a.no_motion else None, "segmentation": False if a.no_segmentation else None,
    }
    base.update({k: v for k, v in flags.items() if v is not None})
    base["name"] = a.name
    base.setdefault("kind", "ToF")
    return SensorMount.from_dict(base)


def resolve_actor(ue, a: argparse.Namespace) -> str:
    """--actor PATH / name, else --robot NAME, else the first robot."""
    if a.actor:
        if "/" in a.actor or ":" in a.actor:
            return a.actor
        try:
            return Robot.find(a.actor, ue=ue).actor_path
        except RobotNotFound:
            hits = ue.find_actors(name_filter=a.actor)
            if not hits:
                raise SensingError(f"no actor matching {a.actor!r}") from None
            return hits[0].object_path
    return Robot.find(a.robot, ue=ue).actor_path


def print_reading_table(readings) -> None:
    for r in readings:
        print(f"  {r.name:20s} {r.kind:6s} t={r.timestamp:8.2f}  {r.summary()}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ramms-sensors", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common(p, True)
    p.add_argument("--actor", "-a", default="", help="actor path or name (default: --robot, else the first robot)")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("list", help="sensors on the robot")
    s.add_argument("--all", action="store_true", help="every actor's sensors")
    s = sub.add_parser("add", help="add (or replace) a sensor")
    s.add_argument("name")
    s.add_argument("--json", dest="mount_json", default=None, metavar="JSON|@FILE", help="the mount as JSON (flags override it)")
    s.add_argument("--kind", "-k", default=None, help="tof, sonar, imu or camera (default tof)")
    s.add_argument("--parent", default=None, help="component to attach to (a MuJoCo body, e.g. chassis); '' = root")
    s.add_argument("--socket", default=None)
    s.add_argument("--at", default=None, metavar="X,Y,Z", help="location, cm relative to the parent (--at=-10,0,20 for a negative x)")
    s.add_argument("--rot", default=None, metavar="P,Y,R", help="rotation pitch,yaw,roll degrees (looks along +X); a negative first value needs --rot=-20,0,0")
    g = s.add_argument_group("ToF")
    g.add_argument("--rows", type=int, default=None)
    g.add_argument("--columns", type=int, default=None)
    g.add_argument("--hfov", type=float, default=None, help="horizontal field of view, deg")
    g.add_argument("--vfov", type=float, default=None)
    g.add_argument("--range", type=float, default=None, help="max range, cm")
    g = s.add_argument_group("Sonar")
    g.add_argument("--sonar-range", type=float, default=None, help="max range, cm")
    g.add_argument("--cone", type=float, default=None, help="beam HALF-angle, deg (15 = a 30 deg beam)")
    g = s.add_argument_group("Camera")
    g.add_argument("--width", type=int, default=None)
    g.add_argument("--height", type=int, default=None)
    g.add_argument("--fov", type=float, default=None, help="horizontal field of view, deg")
    g.add_argument("--channel", type=int, default=None, help="base RMSS channel (-1 = next free)")
    g.add_argument("--no-color", action="store_true")
    g.add_argument("--no-depth", action="store_true")
    g.add_argument("--no-motion", action="store_true")
    g.add_argument("--no-segmentation", action="store_true")
    s = sub.add_parser("remove", help="remove a sensor")
    s.add_argument("name")
    s = sub.add_parser("read", help="one sensor's latest sample")
    s.add_argument("name")
    s.add_argument("--measure", action="store_true", help="trace now instead of the last sample")
    s.add_argument("--grid", action="store_true", help="ToF: print the zones")
    s = sub.add_parser("watch", help="every ToF / sonar / IMU at a rate")
    s.add_argument("--rate", type=float, default=2.0)
    s.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl+C)")
    s.add_argument("--format", choices=["table", "json", "csv"], default="table")
    s.add_argument("--names", default="", help="only these sensors (comma-separated)")
    s = sub.add_parser("imu", help="an IMU on any body, straight from its physics engine (no IMU component needed); "
                                   "--samples / --stream: every physics-step sample")
    s.add_argument("--body", default="", help="MuJoCo body, component or bone (default: the robot's main simulated body)")
    s.add_argument("--sensor", default="", help="an IMU sensor (by its name in `list`) instead of a body")
    s.add_argument("--samples", action="store_true",
                   help="every physics-step sample, over Remote Control (ReadImuSamplesJson, polled every --period s)")
    s.add_argument("--stream", action="store_true", help="every physics-step sample, over RMSS (an IMU stream)")
    s.add_argument("--period", type=float, default=0.1, help="--samples: seconds between polls (the sim keeps ~2 s)")
    s.add_argument("--channel", type=int, default=-1, help="--stream: RMSS channel (-1 = the next free one >= 1000)")
    s.add_argument("--stream-host", default=None, help="--stream: RMSS host (default --host)")
    s.add_argument("--stream-port", type=int, default=None, help="--stream: RMSS port (default the sim's)")
    s.add_argument("--keep-stream", action="store_true", help="--stream: leave the IMU stream running at the end")
    s.add_argument("--offset", default=None, metavar="X,Y,Z", help="sensor position in the body frame, cm (--offset=-10,0,5)")
    s.add_argument("--rot", default=None, metavar="P,Y,R", help="sensor rotation pitch,yaw,roll, deg (--rot=-20,0,0)")
    s.add_argument("--source", default="auto", choices=["auto", "mujoco", "chaos", "kinematic"])
    s.add_argument("--rate", type=float, default=0.0, help="keep reading at this rate (0 = once)")
    s.add_argument("--duration", type=float, default=0.0, help="seconds with --rate / --samples / --stream (0 = until Ctrl+C)")
    s.add_argument("--format", choices=["table", "json", "csv"], default="table")
    sub.add_parser("bodies", help="the bodies an IMU can be read from, and which engine simulates each")
    s = sub.add_parser("rig", help="rig presets: list, show, apply")
    s.add_argument("action", choices=["list", "show", "apply"])
    s.add_argument("preset", nargs="?", default=None, help="preset name or a .json file of mounts")
    s.add_argument("--replace", action="store_true", help="apply: also remove rig sensors the preset does not have")
    s = sub.add_parser("streams", help="camera streams over RMSS: start, stop, status")
    s.add_argument("action", choices=["start", "stop", "status"])
    s.add_argument("--stream-port", type=int, default=None, help="RMSS port (default the sim's, 30030)")
    s.add_argument("--every", type=int, default=None, metavar="N", help="capture every N frames")
    s.add_argument("--no-color", action="store_true")
    s.add_argument("--no-depth", action="store_true")
    s.add_argument("--no-motion", action="store_true")
    s.add_argument("--no-segmentation", action="store_true")
    s.add_argument("--save-to-disk", action="store_true", help="also write the captures on the engine's disk")
    sub.add_parser("classes", help="segmentation classes (mask values)")
    s = sub.add_parser("camera", help="grab a camera's colour / depth / motion / mask frames over RMSS")
    s.add_argument("name")
    s.add_argument("--save", default="", metavar="DIR", help="write the frames here")
    s.add_argument("--frames", type=int, default=1, help="how many frame sets")
    s.add_argument("--roles", default="", help="comma-separated: color,depth,motion,mask (default: what the camera streams)")
    s.add_argument("--stream-host", default=None, help="RMSS host (default --host)")
    s.add_argument("--stream-port", type=int, default=None, help="RMSS port (default the sim's)")
    s.add_argument("--channel", type=int, default=None, help="the camera's base channel (default: from the sim)")
    s.add_argument("--start", action="store_true", help="start the camera streams first")
    s.add_argument("--wait", type=float, default=5.0, help="seconds to wait for each frame set")
    finish_parser(p, sub, no_json=("add",))
    for sp in sub.choices.values():
        sp.add_argument("--actor", "-a", default=argparse.SUPPRESS)
    return p


def main(argv: list[str] | None = None) -> int:
    p = build_parser()
    a = p.parse_args(argv)
    setup_logging(a.verbose)
    if a.cmd == "rig" and a.action in ("list", "show"):
        return _rig_offline(a)
    ue = connect(a)
    if ue is None:
        return 2
    sensing = Sensing(ue)
    try:
        with exit_signals():
            return _run(p, a, ue, sensing)
    except Interrupted as e:
        print(interrupted_message(e), file=sys.stderr)
        return 130
    except (RobotNotFound, SensingError, ControlError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except UnrealRemoteError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except OSError as e:
        print(f"error: RMSS: {e}", file=sys.stderr)
        return 1
    except (ValueError, argparse.ArgumentTypeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


def _run(p: argparse.ArgumentParser, a: argparse.Namespace, ue, sensing: Sensing) -> int:
    if a.cmd == "list":
        actor = "" if a.all else resolve_actor(ue, a)
        sensors = sensing.list(actor)
        if a.json:
            print(json.dumps([s.raw for s in sensors], indent=1))
            return 0
        if not sensors:
            print("no sensors" + ("" if a.all else f" on {actor.rsplit('.', 1)[-1]}"))
        for s in sensors:
            x, y, z = s.location
            origin = "rig" if s.rig else "placed"
            print(f"  {s.name:20s} {s.kind:6s} {origin:6s} on {s.parent or 'root'}{('/' + s.socket) if s.socket else ''}"
                  f"  at ({x:.0f}, {y:.0f}, {z:.0f})  {s.summary()}" + (f"   [{s.actor_name}]" if a.all else ""))
            for st in s.streams:
                print(f"      {st.role:6s} ch {st.channel:4d}  {st.stream_id:28s} {st.message} {st.fmt}")
        return 0

    if a.cmd == "add":
        actor = resolve_actor(ue, a)
        m = mount_from_args(a)
        res = sensing.add(actor, m)
        print(f"added {res.get('name', m.name)} ({res.get('kind', m.kind)}) -> {res.get('component_path', '')}")
        if res.get("built") is False:
            print("warning: kept on the rig, not built yet: the actor has not begun play (the editor world); it is "
                  "built at BeginPlay" + (f" ({res['note']})" if res.get("note") else ""), file=sys.stderr)
        if "channel" in res:
            print(f"  base channel {res['channel']}")
        streams = res.get("streams")
        if streams:
            for st in streams:
                print(f"  streams {st.get('role', ''):6s} ch {st.get('channel', '?'):>4}  {st.get('stream_id', '')}")
        else:
            for role, sid in m.stream_ids().items():
                print(f"  streams {role}: {sid}")
        return 0

    if a.cmd == "remove":
        ok = sensing.remove(resolve_actor(ue, a), a.name)
        print(f"removed {a.name}" if ok else f"not removed: no rig sensor {a.name} on the actor (sensors placed by hand "
                                             "stay), or not a play world (sensing changes need PIE or a game)")
        return 0 if ok else 1

    if a.cmd == "read":
        r = sensing.read(resolve_actor(ue, a), a.name, measure=a.measure)
        if a.json:
            print(json.dumps(r.raw, indent=1))
            return 0
        print(f"{r.name} ({r.kind}) t={r.timestamp:.2f}: {r.summary()}")
        if a.grid and r.kind == "ToF":
            g = r.grid()
            for row in g:
                print("  " + " ".join("   -  " if v != v else f"{v:6.1f}" for v in row))
        return 0

    if a.cmd == "watch":
        actor = resolve_actor(ue, a)
        names = {n.strip() for n in a.names.split(",") if n.strip()}
        writer = {"w": None}

        def tick(t: float) -> None:
            readings = [r for r in sensing.read_all(actor) if not names or r.name in names]
            if a.format == "json" or a.json:
                print(json.dumps({"t": round(t, 3), "readings": [r.raw for r in readings]}), flush=True)
            elif a.format == "csv":
                for r in readings:
                    row = {"t": round(t, 3), **r.flat()}
                    if writer["w"] is None:
                        writer["w"] = csv.DictWriter(sys.stdout, fieldnames=list(row), extrasaction="ignore")
                        writer["w"].writeheader()
                    writer["w"].writerow(row)
                sys.stdout.flush()
            else:
                print(f"-- {t:7.2f} s")
                print_reading_table(readings)
        Ticker(a.rate, a.duration).run(tick)
        return 0

    if a.cmd == "bodies":
        actor = resolve_actor(ue, a)
        bodies = sensing.imu_bodies(actor)
        if a.json:
            print(json.dumps(bodies, indent=1))
            return 0
        for b in bodies:
            star = "*" if b.get("default") else " "
            bone = f"  bone {b['bone']}" if b.get("bone") else ""
            print(f"  {star} {b.get('name', ''):30s} {b.get('engine', ''):9s} {b.get('component', '')}{bone}")
        if not bodies:
            print("no bodies (nothing on this actor is simulated; an IMU would be kinematic)")
        return 0

    if a.cmd == "imu":
        actor = resolve_actor(ue, a)
        offset = _triple(a.offset, "--offset") if a.offset else None
        rot = _triple(a.rot, "--rot") if a.rot else None
        if a.samples and a.stream:
            p.error("--samples or --stream, not both")
        if a.samples or a.stream:
            return _imu_samples(a, sensing, actor, a.sensor or a.body, offset, rot)
        writer = {"w": None}
        first = {"read": True}

        def read_once(t: float) -> None:
            if a.sensor:
                r = sensing.read(actor, a.sensor)
            else:
                # The first read of a body point starts tracking it: wait for its first sample.
                r = sensing.body_imu(actor, a.body, offset, rot, a.source, wait=2.0 if first["read"] else 0.0)
            first["read"] = False
            if a.format == "json" or a.json:
                print(json.dumps({"t": round(t, 3), **r.raw}), flush=True)
            elif a.format == "csv":
                row = {"t": round(t, 3), "body": r.body, **r.flat()}
                if writer["w"] is None:
                    writer["w"] = csv.DictWriter(sys.stdout, fieldnames=list(row), extrasaction="ignore")
                    writer["w"].writeheader()
                writer["w"].writerow(row)
                sys.stdout.flush()
            else:
                v = r.linear_velocity
                vel = f"  v ({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f}) cm/s" if v else ""
                sim = f" sim {r.sim_time:.3f}" if r.sim_time is not None else ""
                print(f"{r.body or 'body'} t={r.timestamp:.2f}{sim}: {r.summary()}{vel}", flush=True)

        if a.rate > 0:
            Ticker(a.rate, a.duration).run(read_once)
        else:
            read_once(0.0)
        return 0

    if a.cmd == "rig":                     # apply
        if not a.preset:
            p.error("rig apply needs a preset (ramms-sensors rig list)")
        actor = resolve_actor(ue, a)
        mounts = rig_mounts(a.preset)
        res = sensing.apply_rig(actor, mounts, replace=a.replace)
        for m, r in zip(mounts, res):
            print(f"  {m.name:20s} {m.kind:6s} -> {r.get('component_path', '')}")
        print(f"applied {a.preset} to {actor.rsplit('.', 1)[-1]} (see ramms_tools.sensing.rigs for where its mounts come from)")
        return 0

    if a.cmd == "streams":
        if a.action == "start":
            st = sensing.start_streams(port=a.stream_port, every_n_frames=a.every,
                                       color=False if a.no_color else None, depth=False if a.no_depth else None,
                                       motion=False if a.no_motion else None,
                                       segmentation=False if a.no_segmentation else None,
                                       save_to_disk=True if a.save_to_disk else None)
        elif a.action == "stop":
            ok = sensing.stop_streams()
            print("capture stopped (the RMSS server stays up)" if ok else
                  "not stopped: no capture in this world, or not a play world (PIE or a game)")
            st = sensing.status()
        else:
            st = sensing.status()
        if a.json:
            print(json.dumps(st.raw, indent=1))
            return 0
        srv, cap = st.server, st.capture
        server = f"running on {st.port}" if st.port is not None else "not running"
        print(f"RMSS server {server}, {srv.get('connections', 0)} connection(s); "
              f"capture {'on' if st.capturing else 'off'} (every {cap.get('every_n_frames', '?')} frames, "
              f"{cap.get('cameras', 0)} cameras, disk {'on' if cap.get('save_to_disk') else 'off'})")
        for s in st.streams:
            who = s.camera or s.target
            print(f"  {who:16s} {s.role:6s} ch {s.channel:4d}  {s.stream_id:28s} {s.message} {s.fmt}   "
                  f"[{s.actor.rsplit('.', 1)[-1]}]")
        return 0

    if a.cmd == "classes":
        t = sensing.segmentation_classes()
        if a.json:
            print(json.dumps(t.raw, indent=1))
            return 0
        for c in t.classes:
            print(f"  {c.value:4d}  {c.name:24s} {c.coarse} ({c.coarse_id})")
        return 0

    if a.cmd == "camera":
        return _camera(a, ue, sensing)
    return 0


def _rig_offline(a: argparse.Namespace) -> int:
    if a.action == "list":
        for n in rig_names():
            print(f"  {n:14s} {RIG_DESCRIPTIONS.get(n, '')}")
        return 0
    if not a.preset:
        print("error: rig show needs a preset", file=sys.stderr)
        return 1
    try:
        mounts = rig_mounts(a.preset)
    except SensingError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(json.dumps([m.to_dict() for m in mounts], indent=1))
    return 0


def _imu_samples(a: argparse.Namespace, sensing: Sensing, actor: str, target: str, offset, rot) -> int:
    """`imu --samples` / `imu --stream`: every physics-step sample, as a table, JSON lines or CSV (the summary goes to
    stderr, so stdout stays data). Ctrl+C ends it normally."""
    writer = {"w": None}
    count = {"n": 0}

    def emit(sample) -> None:
        count["n"] += 1
        row = sample.flat()
        if a.format == "json" or a.json:
            print(json.dumps({"target": target, **row}), flush=True)
        elif a.format == "csv":
            if writer["w"] is None:
                writer["w"] = csv.DictWriter(sys.stdout, fieldnames=list(row))
                writer["w"].writeheader()
            writer["w"].writerow(row)
        else:
            acc, gyro = sample.linear_acceleration, sample.angular_velocity
            print(f"{sample.seq:9d} {sample.sim_time:10.4f} s  acc ({acc[0]:8.2f}, {acc[1]:8.2f}, {acc[2]:8.2f}) cm/s^2  "
                  f"gyro ({gyro[0]:7.2f}, {gyro[1]:7.2f}, {gyro[2]:7.2f}) deg/s")

    duration = a.duration or None
    label = target or "the main body"
    lost, rate, how = 0, 0.0, ""
    try:
        if a.samples:
            how = "over Remote Control"
            follower = sensing.follow_imu_samples(actor, target, period=a.period, duration=duration, offset=offset,
                                                  rotation=rot, source=a.source)
            try:
                for sample in follower:
                    emit(sample)
            finally:
                lost = follower.dropped
                rate = follower.last.rate_hz if follower.last else 0.0
        else:
            info = sensing.start_imu_stream(actor, target, channel=a.channel, port=a.stream_port, offset=offset,
                                            rotation=rot, source=a.source)
            how = f"over RMSS ({info.stream_id}, channel {info.channel})"
            print(f"IMU stream {info.stream_id} on channel {info.channel}", file=sys.stderr)
            try:
                with ImuStream.from_info(info, host=a.stream_host or a.host, port=a.stream_port or info.port) as stream:
                    try:
                        for sample in stream.samples(duration=duration):
                            emit(sample)
                    finally:
                        lost = stream.dropped
            finally:
                if not a.keep_stream:
                    sensing.stop_imu_stream(actor, target)
    except Interrupted as e:
        if e.signame != "SIGINT":
            print(interrupted_message(e), file=sys.stderr)
            return 130
    sys.stdout.flush()
    print(f"{count['n']} samples of {label} {how}" + (f", {lost} lost" if lost else "")
          + (f", {rate:.0f} Hz" if rate else ""), file=sys.stderr)
    return 0 if count["n"] else 1


def _find_camera(sensing: Sensing, actor: str, name: str):
    """The camera `name` on `actor` (SensorInfo); SensingError saying where cameras of that name are when not there."""
    mine = [x for x in sensing.list(actor) if x.kind == "Camera" and x.name == name]
    if mine:
        return mine[0]
    elsewhere = sorted({x.actor_name or x.actor for x in sensing.list("") if x.kind == "Camera" and x.name == name})
    where = actor.rsplit(".", 1)[-1]
    raise SensingError(f"no camera {name!r} on {where}" + (f" (there is one on {', '.join(elsewhere)}: --actor / --robot "
                                                         "picks the actor)" if elsewhere else ""))


def _camera(a: argparse.Namespace, ue, sensing: Sensing) -> int:
    host = a.stream_host or a.host
    actor = resolve_actor(ue, a)
    if a.start:
        st = sensing.start_streams()
        print(f"streams started: {len(st.streams)} streams, RMSS on {st.port}")
    roles = [r.strip() for r in a.roles.split(",") if r.strip()] if a.roles else None
    if a.channel is not None:
        cam = CameraStreams(a.name, base_channel=a.channel, roles=roles or ROLES, host=host,
                            port=a.stream_port or 30030, actor=actor)
    else:
        st = sensing.status()
        if not st.capturing:
            print("warning: the camera capture is off (ramms-sensors streams start, or --start)", file=sys.stderr)
        port = a.stream_port or st.port
        if port is None:
            raise SensingError("the RMSS server is not running (ramms-sensors streams start, or --start)")
        # This actor's camera: the channel it actually streams on and the roles it streams.
        info = _find_camera(sensing, actor, a.name)
        if info.channel is not None:
            kw = {"roles": roles} if roles else {}
            cam = CameraStreams.from_sensor(info, host=host, port=port, **kw)
        else:
            cam = CameraStreams.from_status(st, a.name, actor=info.actor or actor, roles=roles or ROLES, host=host, port=port)
    roles = list(cam.roles)
    out = Path(a.save) if a.save else None
    classes_written = False
    got_sets = 0
    with cam:
        print(f"{cam.camera} on {actor.rsplit('.', 1)[-1]}: "
              + (f"channels {cam.channels}" if cam.channels else "every channel, filtered by stream id and actor")
              + f" on {cam.client.host}:{cam.client.port}")
        for i in range(a.frames):
            frames = cam.grab(timeout=a.wait)
            missing = [r for r in roles if r not in frames]
            desc = "  ".join(f"{r} {tuple(f.array.shape)} {f.array.dtype}" for r, f in frames.items())
            fn = f" (capture {frames.frame_number})" if getattr(frames, "frame_number", None) is not None else ""
            print(f"  set {i}{fn}: {desc}" + (f"   incomplete, missing: {', '.join(missing)}" if missing else ""))
            if not frames:
                break
            got_sets += 1
            if out is not None:
                for f in frames.values():
                    save_frame(f, out, i)
                    if f.role == "mask" and f.classes and not classes_written:
                        (out / "classes.json").write_text(json.dumps({str(k): v for k, v in sorted(f.classes.items())}, indent=1))
                        classes_written = True
    if cam.other_actor:
        print(f"  ({cam.other_actor} frames of another actor's {cam.camera!r} dropped)")
    if out is not None:
        print(f"wrote {got_sets} frame set(s) to {out}")
    if got_sets == 0:
        print("error: no frames arrived (is the capture on, and the camera streaming?)", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
