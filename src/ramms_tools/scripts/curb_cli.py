#!/usr/bin/env python3
"""
ramms-curb -- curb crossing on whichever robot is in the world: the RAMMP Gen1
curb controller or the RAMMP Gen2 (lift-drive) curb negotiation.

    ramms-curb status                         # phase, status line, assist, mode, key sensing
    ramms-curb watch --rate 5                 # the same at a rate (--changes: only when it changes)
    ramms-curb assist on                      # curb assist (pushing forward at a curb starts a crossing)
    ramms-curb style rolling                  # RAMMP Gen1: "Stop and step" or "Rolling"
    ramms-curb detect                         # Gen2: the step and terrain it senses ahead
    ramms-curb drive --speed 0.3 --seconds 4  # approach the curb (drive.forward, held, then released)
    ramms-curb go --wait                      # start a crossing, hold the stick while it runs, phase log and stats
    ramms-curb abort
    ramms-curb stats                          # how the last crossing went
    ramms-curb plan                           # Gen2: plan, schedule, step log, planned tilt
    ramms-curb seat                           # Gen2 seat: pivot, tilt, reachable outline
    ramms-curb seat --pivot 0,45 --tilt -5    # walk the pivot (fore/aft, height cm) and tilt it
    ramms-curb seat --reset

A crossing moves only while the rider's stick (drive.forward) is held forward.
`go --wait` holds it at --forward (default 1.0, full; 0.25..1.0 -- a RAMMP Gen2
crossing goes at the stick's share of its planned speed) while the crossing is
under way and the robot is in the curb drive mode. The sender reads both in
the same request as each send, so it lets go in the round that sees the
crossing done, stopped or faulted: at most one send lands after it ends, none
after that. Without --wait, `go` only starts the crossing: it then waits,
paused, for a stick. A crossing already under way that this did not start (the
rider's, curb assist's, another client's) is not taken over -- a Remote stick
outranks the rider's -- unless --join (--no-hold only watches it). Ctrl+C /
SIGTERM / SIGHUP during `go --wait` releases the stick and aborts the crossing
(--on-interrupt pause: leave it paused; `go --wait --join` goes on).

--robot NAME picks a robot when the world has several. The RAMMP Gen2
(lift-drive) robot is only in levels built with the private assets.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from ramms_tools.control import ControlError, release_all
from ramms_tools.robot import CurbError, Gen1Curb, Gen2Curb, ModeError, Robot, RobotNotFound
from ramms_tools.robot.curb import CROSSING_STICK_ENGAGE, DEFAULT_CROSSING_STICK, check_crossing_stick
from ramms_tools.unreal_remote import UnrealRemoteError

from ._common import (Interrupted, Ticker, add_common, connect, dump_json, exit_signals, finish_parser, fmt_num,
                      interrupted_message, setup_logging)

GENERATIONS = {"gen1": "RAMMP Gen1", "gen2": "RAMMP Gen2 (lift-drive)"}


def _sev(sev: str) -> str:
    return "" if sev == "Info" else f" [{sev}]"


def _stick(text: str) -> float:
    try:
        return check_crossing_stick(float(text))
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from None


def status_dict(robot: Robot) -> dict:
    """Everything `status` prints, as plain data."""
    curb = robot.curb
    out: dict = {"robot": robot.name, "generation": robot.generation}
    try:
        out["mode"] = robot.mode
        out["switch_status"] = robot.drive_modes.switch_status()
    except (UnrealRemoteError, ControlError):
        out["mode"] = ""
    if curb is None:
        return out
    out.update(curb.snapshot())
    out["assist"] = curb.assist
    if isinstance(curb, Gen1Curb) and robot.surface.has("curb.style"):
        out["style"] = curb.style
    if isinstance(curb, Gen2Curb):
        out["detection"] = curb.detection_text()
    return out


def print_status(s: dict) -> None:
    print(f"{s['robot']}  ({GENERATIONS.get(s['generation'], 'no curb controller')})  drive mode {s.get('mode') or '-'}")
    if "phase" not in s:
        return
    print(f"  phase    {s['label']}" + (f" ({s['phase']})" if s["label"] != s["phase"] else "")
          + (f", {s['axle']} axle" if s.get("axle") and s["phase"] != "Idle" else ""))
    print(f"  status   {s['status']}{_sev(s['severity'])}")
    print(f"  assist   {'on' if s.get('assist') else 'off'}" + (f"    style {s['style']}" if s.get("style") else ""))
    if s["generation"] == "gen1":
        print(f"  crossing step {fmt_num(s.get('step_cm'), 1)} cm at chassis height {fmt_num(s.get('crossing_height_cm'), 1)} cm")
    if s["generation"] == "gen2":
        print(f"  tilt     pitch {fmt_num(s['pitch_deg'], 1)} deg, roll {fmt_num(s['roll_deg'], 1)} deg   "
              f"base height {fmt_num(s['base_height_cm'], 1)} cm (cmd {fmt_num(s['base_height_cmd_cm'], 1)})")
        print(f"  speed    {fmt_num(s['speed_cm_s'], 1)} cm/s (cmd {fmt_num(s['speed_cmd_cm_s'], 1)})   "
              f"margin {fmt_num(s['margin_cm'], 1)} cm   elapsed {fmt_num(s['elapsed_s'], 1)} s")
        if s.get("detection"):
            print(f"  ahead    {s['detection']}")


def watch_line(t: float, s: dict) -> str:
    line = f"{t:7.2f} s  {s.get('label', '-'):30s} {s.get('status', '')}{_sev(s.get('severity', 'Info'))}"
    if s.get("generation") == "gen2" and "pitch_deg" in s:
        line += (f"  | pitch {s['pitch_deg']:+.1f} roll {s['roll_deg']:+.1f}  h {s['base_height_cm']:.1f}  "
                 f"v {s['speed_cm_s']:.1f}  margin {s['margin_cm']:.1f}")
    elif s.get("generation") == "gen1" and s.get("step_cm"):
        line += f"  | step {s['step_cm']:.1f} cm"
    return line


def print_stats(stats: dict) -> None:
    for k, v in stats.items():
        print(f"  {k:28s} {fmt_num(v, 2)}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ramms-curb", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common(p, True)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status", help="phase, status line, assist, drive mode, key sensing")
    s = sub.add_parser("watch", help="status at a rate")
    s.add_argument("--rate", type=float, default=5.0)
    s.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl+C)")
    s.add_argument("--changes", action="store_true", help="print only when the phase or status line changes")
    s = sub.add_parser("assist", help="curb assist on / off (no argument: show)")
    s.add_argument("state", nargs="?", choices=["on", "off"])
    s = sub.add_parser("style", help="RAMMP Gen1 crossing style (no argument: show)")
    s.add_argument("style", nargs="?", help="'Stop and step' / stepped, or 'Rolling' / blended")
    sub.add_parser("detect", help="what the robot senses ahead (Gen2 step and terrain)")
    s = sub.add_parser("go", help="start a crossing (the controller selects the curb drive mode itself)")
    s.add_argument("--wait", action="store_true", help="hold the stick while it runs; print the phase log and stats")
    s.add_argument("--wait-timeout", type=float, default=180.0, help="seconds to wait with --wait")
    s.add_argument("--forward", type=_stick, default=DEFAULT_CROSSING_STICK,
                   help=f"with --wait: the stick (drive.forward, {CROSSING_STICK_ENGAGE:g}..1.0) held while the crossing "
                        "is under way and the robot is in the curb mode, and only then. A Gen2 crossing goes at this "
                        "share of its planned speed; Gen1 at its planned speeds. Default "
                        f"{DEFAULT_CROSSING_STICK:g} (full); under {CROSSING_STICK_ENGAGE:g} a crossing never engages")
    s.add_argument("--no-hold", action="store_true",
                   help="with --wait: do not hold drive.forward (someone else holds the stick); just watch")
    s.add_argument("--join", action="store_true",
                   help="hold the stick for a crossing already under way that this did not start (the rider's, curb "
                        "assist's, another client's): a Remote stick outranks the rider's own")
    s.add_argument("--on-interrupt", choices=["abort", "pause"], default="abort",
                   help="Ctrl+C / SIGTERM / SIGHUP during --wait: release the stick, then abort the crossing (default) "
                        "or leave it paused")
    s = sub.add_parser("abort", help="stop the crossing")
    s.add_argument("--reason", default="Stopped from ramms-curb")
    sub.add_parser("stats", help="how the current or last crossing went")
    sub.add_parser("plan", help="Gen2: plan, schedule, step log, planned tilt")
    s = sub.add_parser("seat", help="Gen2 seat: show, or move the pivot / tilt")
    s.add_argument("--pivot", default=None, metavar="X,Z", help="pivot target: fore/aft, height (cm, chassis); --pivot=-10,45 for a negative fore/aft")
    s.add_argument("--tilt", type=float, default=None, metavar="DEG", help="platform tilt (nose up positive)")
    s.add_argument("--reset", action="store_true", help="back to the rest pivot and tilt")
    s.add_argument("--outline", action="store_true", help="print the reachable outline")
    s = sub.add_parser("drive", help="approach: drive.forward at --speed for --seconds, then release")
    s.add_argument("--speed", type=float, default=0.3, help="normalized -1..1")
    s.add_argument("--turn", type=float, default=0.0)
    s.add_argument("--seconds", type=float, default=2.0)
    s.add_argument("--rate", type=float, default=10.0)
    finish_parser(p, sub)
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
    except (RobotNotFound, ModeError, CurbError, ControlError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except UnrealRemoteError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


def _go(a: argparse.Namespace, robot: Robot, curb) -> int:
    # Start the crossing and let the controller select the curb drive mode: selecting that mode by hand is
    # not how either robot starts one (Gen2 drops back to its previous mode at once when there is "no
    # crossing to run"; RAMMP Gen1 starts one, but its refusal is then only in its status line).
    holding = a.wait and not a.no_hold and robot.surface.has("drive.forward")
    not_ours = ("that this did not start (the rider, curb assist or another client): holding its stick from here would "
                "outrank the rider's own. --join to hold it anyway, --no-hold to only watch it")
    ph, st = curb.poll_state()
    if curb.is_active(ph) or curb.is_starting(st):
        if holding and not a.join:
            print(f"error: a crossing is already under way ({curb.phase_label(ph)}: {st.text}) {not_ours}", file=sys.stderr)
            return 1
        print(f"crossing under way ({curb.phase_label(ph)}): {st.text}" + ("; joining it" if holding else ""))
    else:
        ok, reason = curb.start()
        if not ok:
            time.sleep(0.3)                     # a refusal can mean one was started meanwhile
            ph, st = curb.poll_state()
            if not (curb.is_active(ph) or curb.is_starting(st)):
                print(f"not started: {reason or st.text}{_sev(st.severity)}", file=sys.stderr)
                return 1
            if holding and not a.join:
                print(f"error: not started ({reason}): a crossing is already under way ({curb.phase_label(ph)}: "
                      f"{st.text}) {not_ours}", file=sys.stderr)
                return 1
            print(f"crossing under way ({curb.phase_label(ph)}): {st.text}")
        else:
            print(f"crossing started (drive mode {robot.mode})" + (f": {reason}" if reason else ""))
    if not a.wait:
        print("it moves only while the stick (drive.forward) is held forward: `ramms-curb go --wait` holds it")
        return 0
    hold = a.forward if holding else None
    if hold is not None:
        print(f"holding drive.forward {hold:+.2f} while the crossing is under way in the curb mode (Ctrl+C: release "
              f"and {'abort' if a.on_interrupt == 'abort' else 'pause'})")
    try:
        run = curb.wait_until_done(timeout=a.wait_timeout, on_event=lambda e: print(f"  {e}", flush=True),
                                   hold_forward=hold)
    except Interrupted as e:
        # wait_until_done released the stick on the way out: the crossing is paused now.
        try:
            if a.on_interrupt == "abort" and curb.is_active():
                curb.abort(f"Stopped from ramms-curb ({e.signame})")
                st = curb.status_line()
                print(interrupted_message(e, f"stick released, crossing aborted: {curb.phase_label()}: {st.text}"),
                      file=sys.stderr)
            else:
                print(interrupted_message(e, "stick released, the crossing is paused (`ramms-curb go --wait --join` "
                                             "goes on, `ramms-curb abort` stops it)"), file=sys.stderr)
        except UnrealRemoteError as err:
            print(interrupted_message(e, f"stick released; could not abort the crossing: {err}"), file=sys.stderr)
        return 130
    verdict = "done" if run.succeeded else ("timed out" if run.timed_out else
                                            ("never started" if not run.started else f"ended {run.phase}"))
    print(f"{verdict} after {run.seconds:.1f} s: {run.status.text}{_sev(run.status.severity)}")
    if run.stick_refused:
        print(f"warning: the robot refused the held stick {run.stick_refused} times (another source holds "
              "drive.forward)", file=sys.stderr)
    if run.stats:
        print("stats:")
        print_stats(run.stats)
    return 0 if run.succeeded else 1


def _run(p: argparse.ArgumentParser, a: argparse.Namespace, ue) -> int:
    robot = Robot.find(a.robot, ue=ue)
    curb = robot.curb
    needs_curb = a.cmd not in ("drive", "seat")
    if needs_curb and curb is None:
        print(f"error: {robot.name} has no curb controller (RAMMP Gen1 or Gen2 curb pawn)", file=sys.stderr)
        return 1

    if a.cmd == "status":
        s = status_dict(robot)
        print(dump_json(s) if a.json else "", end="\n" if a.json else "")
        if not a.json:
            print_status(s)
        return 0

    if a.cmd == "watch":
        last = {"key": None}

        def tick(t: float) -> None:
            s = status_dict(robot)
            key = (s.get("phase"), s.get("status"), s.get("severity"))
            if a.changes and key == last["key"]:
                return
            last["key"] = key
            print(dump_json({"t": round(t, 3), **s}) if a.json else watch_line(t, s), flush=True)
        Ticker(a.rate, a.duration).run(tick)
        return 0

    if a.cmd == "assist":
        if a.state:
            curb.set_assist(a.state == "on")
        print(f"curb assist {'on' if curb.assist else 'off'}")
        return 0

    if a.cmd == "style":
        if not isinstance(curb, Gen1Curb):
            print("error: the crossing style is a RAMMP Gen1 setting", file=sys.stderr)
            return 1
        if a.style:
            curb.set_style(a.style)
        print(f"curb style {curb.style}")
        return 0

    if a.cmd == "detect":
        d = curb.detect()
        if a.json:
            print(dump_json(d))
        elif not d.get("available"):
            print(f"{d['note']}.")
            print(f"  current crossing: step {fmt_num(d['step_cm'], 1)} cm, chassis height {fmt_num(d['crossing_height_cm'], 1)} cm"
                  f"   ({d['status']})")
        else:
            found = "step found" if d["step_found"] else "no step to cross"
            print(f"{found}: {d['reason']}")
            if d["step_found"]:
                print(f"  height {d['step_height_cm']:+.1f} cm, edge {d['edge_distance_cm']:.1f} cm ahead, "
                      f"skew {d['edge_skew_cm']:.1f} cm, approach {d['approach_angle_deg']:+.1f} deg off square")
            print(f"  terrain {'valid' if d['terrain_valid'] else 'not valid'}, far side {d['far_depth_cm']:.0f} cm deep")
        return 0

    if a.cmd == "go":
        return _go(a, robot, curb)

    if a.cmd == "abort":
        curb.abort(a.reason)
        print(f"abort sent; {curb.phase_label()}: {curb.status_line().text}")
        return 0

    if a.cmd == "stats":
        st = curb.stats()
        print(json.dumps(st, indent=1) if a.json else "", end="\n" if a.json else "")
        if not a.json:
            print_stats(st)
        return 0

    if a.cmd == "plan":
        if not isinstance(curb, Gen2Curb):
            print("error: the crossing plan is a RAMMP Gen2 (lift-drive) feature; RAMMP Gen1 follows a fixed sequence",
                  file=sys.stderr)
            return 1
        plan, schedule, log = curb.plan(), curb.schedule(), curb.step_log()
        pitch, roll = curb.planned_tilt()
        if a.json:
            print(json.dumps({"plan": plan, "schedule": schedule, "step_log": log, "planned_tilt_deg": [pitch, roll]}, indent=1))
            return 0
        for title, lines in (("plan", plan), ("schedule", schedule), ("step log", log)):
            print(f"{title}:" + ("" if lines else " (none yet)"))
            for ln in lines:
                print(f"  {ln}")
        print(f"planned tilt: pitch {pitch:+.1f} deg, roll {roll:+.1f} deg")
        return 0

    if a.cmd == "seat":
        seat = robot.seat
        if seat is None:
            print(f"error: {robot.name} has no seat mechanism (a RAMMP Gen2 feature)", file=sys.stderr)
            return 1
        if a.reset:
            print("seat reset" if seat.reset() else "seat busy: not reset")
        if a.pivot:
            try:
                x, z = (float(v) for v in a.pivot.split(","))
            except ValueError:
                p.error("--pivot expects X,Z (cm)")
            if seat.outline() and not seat.contains(x, z):
                print(f"warning: ({x:g}, {z:g}) is outside the reachable outline; the seat stops short", file=sys.stderr)
            print(f"pivot -> ({x:g}, {z:g})" if seat.set_pivot(x, z) else "pivot refused (out of reach, past a motor's range, or busy)")
        if a.tilt is not None:
            print(f"tilt -> {a.tilt:g} deg" if seat.set_tilt(a.tilt) else "tilt refused")
        st = seat.state()
        outline = seat.outline() if a.outline or a.json else []
        if a.json:
            print(json.dumps({"pivot": st.pivot, "tilt_deg": st.tilt_deg, "outline": outline}))
            return 0
        piv = f"({st.pivot[0]:.1f}, {st.pivot[1]:.1f}) cm" if st.pivot else "unknown"
        print(f"seat pivot {piv} (fore/aft, height)   tilt {fmt_num(st.tilt_deg, 1)} deg")
        if a.outline:
            print("reachable outline (fore/aft, height):")
            for x, z in outline:
                print(f"  {x:7.2f} {z:7.2f}")
        return 0

    if a.cmd == "drive":
        print(f"driving forward {a.speed:+.2f}, turn {a.turn:+.2f} for {a.seconds:.1f} s (Ctrl+C stops) ...")
        robot.surface.hold(robot.surface.drive_values(a.speed, a.turn), a.seconds, rate=a.rate)
        msg = "released"
        if curb is not None:
            st = curb.status_line()
            msg += f"; {curb.phase_label()}: {st.text}{_sev(st.severity)}"
        print(msg)
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
