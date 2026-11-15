#!/usr/bin/env python3
"""
ramms-control -- describe and drive any robot through its control surface
(URammsRobotControlSurfaceComponent) over the Remote Control API.

    ramms-control robots                                # robots in the world, their drive mode
    ramms-control describe [--group Curb]               # every control: kind, units, range, choices
    ramms-control get drive.mode curb.status            # Enum labels, Text with severity, numbers
    ramms-control set curb.assist On                    # Enum by label (or index)
    ramms-control set seat.height 45                    # Position control (cm): holds until released
    ramms-control set drive.forward 0.3 --hold 2        # a Continuous control: re-sent at 10 Hz for 2 s, then released
    ramms-control trigger camera.next                   # an Action (or step an Enum)
    ramms-control release seat.height
    ramms-control text curb.status
    ramms-control watch drive.mode curb.phase curb.status --rate 5
    ramms-control drive --forward 0.4 --turn 0.1 --seconds 3
    ramms-control drive --forward 0.2 --strafe 0.3 --seconds 2   # holonomic modes have drive.strafe
    ramms-control mode                                  # list drive modes
    ramms-control mode holonomic                        # switch (waits for the hand-over)
    ramms-control mode --cycle

--robot NAME picks a robot when the world has several. Commands go out as
Source Remote (--source Autonomy / Script ...). A Continuous control (the
drive axes) only acts while it is re-sent: `set` takes it only with --hold
SECONDS, re-sending at --rate Hz and releasing it at the end. On a sim with the
control-surface deadman a Remote / Autonomy Continuous command springs back
0.3 s after its last send; older sims keep it until it is released. Ctrl+C,
SIGTERM and SIGHUP release whatever is held before exiting.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from ramms_tools.control import SOURCES, ControlError, StatusText, release_all
from ramms_tools.robot import ModeError, Robot, RobotNotFound
from ramms_tools.unreal_remote import UnrealRemoteError

from ._common import (Interrupted, Ticker, add_common, connect, dump_json, exit_signals, finish_parser, fmt_num,
                      interrupted_message, setup_logging)


def _show(axis, value) -> str:
    if isinstance(value, StatusText):
        return f"[{value.severity}] {value.text}"
    if axis.is_enum:
        return f"{value}"
    return f"{fmt_num(value, 3)}" + ("" if axis.units in ("None", "Normalized") else f" {axis.units.lower()}")


def _refused(cs) -> str:
    r = cs.last_hold_refused
    return ("   (refused by the robot: " + ", ".join(f"{k} {n}x" for k, n in r.items())
            + " -- another source holds it, or this drive mode does not take it)") if r else ""


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ramms-control", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common(p, True)
    p.add_argument("--source", default="Remote", choices=SOURCES, help="ERammsControlSource commands are sent as")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("robots", help="robots with a control surface")
    s = sub.add_parser("describe", help="list the controls")
    s.add_argument("--group", "-g", default=None, help="only this group (Drive, Curb, Seat ...)")
    s = sub.add_parser("get", help="read controls (Enum label, Text with severity, number)")
    s.add_argument("ids", nargs="+")
    s.add_argument("--target", action="store_true", help="also show the commanded target")
    s = sub.add_parser("set", help="command a control (an Enum by label or index; a Continuous one needs --hold)")
    s.add_argument("id")
    s.add_argument("value")
    s.add_argument("--hold", type=float, default=0.0, metavar="SECONDS",
                   help="re-send at --rate for this long, then release (required for a Continuous control)")
    s.add_argument("--rate", type=float, default=10.0, help="re-send rate for --hold (Hz)")
    s = sub.add_parser("trigger", help="fire an Action (or step an Enum to its next choice)")
    s.add_argument("id")
    s = sub.add_parser("release", help="let go of controls")
    s.add_argument("ids", nargs="+")
    s = sub.add_parser("text", help="a Text control's words and severity")
    s.add_argument("id")
    s = sub.add_parser("watch", help="print controls at a rate")
    s.add_argument("ids", nargs="*")
    s.add_argument("--group", "-g", default=None, help="watch every readback control of a group")
    s.add_argument("--rate", type=float, default=5.0)
    s.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl+C)")
    s = sub.add_parser("drive", help="drive (normalized -1..1), held for --seconds then released")
    s.add_argument("--forward", "-f", type=float, default=0.0)
    s.add_argument("--turn", "-t", type=float, default=0.0)
    s.add_argument("--strafe", "-s", type=float, default=None, help="holonomic modes only (drive.strafe)")
    s.add_argument("--seconds", type=float, default=1.0)
    s.add_argument("--rate", type=float, default=10.0)
    s = sub.add_parser("mode", help="list, set or cycle the drive mode")
    s.add_argument("mode", nargs="?", default=None, help="mode id, label or index (or 'cycle')")
    s.add_argument("--cycle", action="store_true", help="the next mode in the robot's cycle")
    s.add_argument("--no-wait", action="store_true", help="do not wait for the hand-over")
    s.add_argument("--wait-timeout", type=float, default=15.0, help="seconds to wait for the hand-over")
    finish_parser(p, sub)
    for sp in sub.choices.values():
        sp.add_argument("--source", default=argparse.SUPPRESS, choices=SOURCES)
    return p


def main(argv: list[str] | None = None) -> int:
    p = build_parser()
    a = p.parse_args(argv)
    setup_logging(a.verbose)
    ue = connect(a)
    if ue is None:
        return 2
    try:
        with exit_signals():
            return _run(p, a, ue)
    except Interrupted as e:
        release_all()
        print(interrupted_message(e, "held controls released"), file=sys.stderr)
        return 130
    except (RobotNotFound, ModeError, ControlError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except UnrealRemoteError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


def _run(p: argparse.ArgumentParser, a: argparse.Namespace, ue) -> int:
    if a.cmd == "robots":
        robots = Robot.all(ue=ue, source=a.source)
        rows = []
        for r in robots:
            try:
                mode = r.mode
            except UnrealRemoteError:
                mode = "?"
            try:
                d = r.surface.describe()
                groups, controls = d.groups, len(d.axes)
            except UnrealRemoteError as e:
                groups, controls = [f"? ({e})"], 0
            rows.append({"robot": r.name, "actor": r.actor_path, "generation": r.generation, "mode": mode,
                         "groups": groups, "controls": controls})
        if a.json:
            print(json.dumps(rows, indent=1))
        elif not rows:
            print("no robots with a control surface in the world")
        for row in ([] if a.json else rows):
            gen = {"gen1": "RAMMP Gen1", "gen2": "RAMMP Gen2 (lift-drive)"}.get(row["generation"], "")
            print(f"  {row['robot']:40s} {gen:24s} mode {row['mode']:13s} {row['controls']} controls in "
                  f"{len(row['groups'])} groups")
            print(f"      {row['actor']}")
        return 0

    robot = Robot.find(a.robot, ue=ue, source=a.source)
    cs = robot.surface

    if a.cmd == "describe":
        axes = cs.axes(a.group)
        if a.json:
            print(json.dumps([dict(x.raw) for x in axes], indent=1))
            return 0
        print(f"{cs.robot_name}  ({len(cs.axes())} controls; groups: {', '.join(cs.groups())})")
        group = None
        for ax in sorted(axes, key=lambda x: (cs.groups().index(x.group) if x.group in cs.groups() else 99, x.order)):
            if ax.group != group:
                group = ax.group
                print(f"\n  [{group}]")
            flags = ("ro " if ax.read_only else "") + ("" if ax.readback else "no-readback ")
            rng = "" if ax.is_action or ax.is_text else f"{fmt_num(ax.range[0])}..{fmt_num(ax.range[1])}"
            extra = (" choices: " + " | ".join(f"{i}={lab}" for i, lab in enumerate(ax.enum_labels))) if ax.is_enum else ""
            extra += f" paired {ax.paired_axis}" if ax.paired_axis else ""
            extra += f" outline {len(ax.region_outline)} pts" if ax.region_outline else ""
            print(f"    {ax.id:40s} {ax.kind:10s} {ax.units:12s} {rng:16s} {flags}{ax.display_name!r}{extra}")
        return 0

    if a.cmd == "get":
        out = {}
        for cid in a.ids:
            ax = cs.axis(cid)
            v = cs.read(ax.id)
            tgt = cs.target(ax.id) if a.target else None
            out[ax.id] = {"value": v.text if isinstance(v, StatusText) else v, "target": tgt,
                          **({"severity": v.severity} if isinstance(v, StatusText) else {})}
            if not a.json:
                print(f"  {ax.id:36s} {_show(ax, v)}" + (f"   (target {fmt_num(tgt, 3)})" if a.target else ""))
        if a.json:
            print(json.dumps(out, indent=1))
        return 0

    if a.cmd == "set":
        ax = cs.axis(a.id)
        value = a.value if ax.is_enum else float(a.value)
        if a.hold > 0:
            n = cs.hold({ax.id: value}, a.hold, rate=a.rate)
            print(f"{ax.id} held at {a.value} for {a.hold:.1f} s ({n} sends), released" + _refused(cs))
            return 1 if cs.last_hold_refused.get(ax.id, 0) >= max(n, 1) else 0
        if ax.is_continuous:
            print(f"error: {ax.id} is a Continuous control: it acts only while re-sent. Give --hold SECONDS (re-sent "
                  f"at --rate Hz, then released); `drive` does the same for the drive axes.", file=sys.stderr)
            return 1
        ok = cs.set(ax.id, value)
        shown = ax.enum_labels[ax.enum_index(value)] if ax.is_enum else a.value
        print(f"{ax.id} -> {shown}" + ("" if ok else "   (refused by the robot)"))
        if ok and ax.kind in ("Position", "Velocity"):
            print(f"  it holds that target until released: ramms-control release {ax.id}")
        return 0 if ok else 1

    if a.cmd == "trigger":
        ok = cs.trigger(a.id)
        print(f"{a.id} triggered" if ok else f"{a.id}: refused")
        return 0 if ok else 1

    if a.cmd == "release":
        for cid in a.ids:
            print(f"{cid} released" if cs.release(cid) else f"{cid}: nothing to release")
        return 0

    if a.cmd == "text":
        st = cs.status(a.id)
        print(json.dumps(st._asdict()) if a.json else f"[{st.severity}] {st.text}")
        return 0

    if a.cmd == "watch":
        ids = list(a.ids)
        if a.group:
            ids += [x.id for x in cs.axes(a.group) if (x.readback or x.is_text) and not x.is_action and x.id not in ids]
        if not ids:
            p.error("watch needs control ids or --group")
        axes = [cs.axis(cid) for cid in ids]

        def tick(t: float) -> None:
            vals = {ax.id: cs.read(ax.id) for ax in axes}
            if a.json:
                print(dump_json({"t": round(t, 3), **{k: (v._asdict() if isinstance(v, StatusText) else v) for k, v in vals.items()}}),
                      flush=True)
            else:
                print(f"{t:7.2f} s  " + "  ".join(f"{ax.id}={_show(ax, vals[ax.id])}" for ax in axes), flush=True)
        Ticker(a.rate, a.duration).run(tick)
        return 0

    if a.cmd == "drive":
        values = cs.drive_values(a.forward, a.turn, a.strafe)
        print(f"driving {', '.join(f'{k} {v:+.2f}' for k, v in values.items())} for {a.seconds:.1f} s "
              f"at {a.rate:.0f} Hz (Ctrl+C stops)")
        cs.hold(values, a.seconds, rate=a.rate)           # released at the end, on an error and on a signal
        motion = robot.measured_motion()
        print("released" + (f"; measured {motion.forward_cm_s:.1f} cm/s forward, {motion.yaw_rate_deg_s:.1f} deg/s"
                            if motion else "") + _refused(cs))
        return 0

    if a.cmd == "mode":
        dm = robot.drive_modes
        if not dm.available:
            print(f"{robot.name} has no drive modes", file=sys.stderr)
            return 1
        if a.cycle or (a.mode or "").lower() in ("cycle", "next"):
            before = dm.active()
            now = dm.cycle(wait=not a.no_wait, timeout=a.wait_timeout)
            status = dm.switch_status()
            if now == before and dm.is_switching():
                print(f"drive mode {before}: switching" + (f"   ({status})" if status else ""))
            else:
                print(f"drive mode {before} -> {now}" + (f"   ({status})" if status else ""))
            return 0
        if a.mode is None:
            modes = dm.modes()
            if a.json:
                print(json.dumps([m.__dict__ for m in modes], indent=1))
            else:
                for i, m in enumerate(modes):
                    print(f"  {'*' if m.active else ' '} {i}  {m.id:14s} {m.label}")
                if dm.switch_status():
                    print(f"  last switch: {dm.switch_status()}")
            return 0
        target = dm.resolve(a.mode)
        t0 = time.monotonic()
        status = dm.set(target, wait=not a.no_wait, timeout=a.wait_timeout, on_status=lambda s: print(f"  ... {s}"))
        took = f" in {time.monotonic() - t0:.1f} s" if not a.no_wait else " (switching)"
        print(f"drive mode -> {target}{took}" + (f"   ({status})" if status else ""))
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
