#!/usr/bin/env python3
"""
Control skeletal mesh poses via Unreal Engine Remote Control API.

Drives URammsSkeletalPoseComponent to set bone rotations/translations on
actors with UPoseableMeshComponents. Suitable for testing kinematic posing
of robot UI visualizations from external joint data.

Usage:
    ramms-pose --list                          # Find poseable actors
    ramms-pose --describe                      # Show joints, meshes, and values
    ramms-pose --set-all 0 15 -90 30           # Set all joint targets
    ramms-pose --set Shoulder 45.0             # Set joint by name
    ramms-pose --set-index 0 45.0              # Set joint by index
    ramms-pose --home                          # All joints to 0
    ramms-pose --sweep 0 -90 90 --period 4     # Sweep joint 0 between -90..90
    ramms-pose --interactive                   # Interactive REPL

Requires Unreal Engine running with Remote Control API plugin enabled (port 30010).
"""

import argparse
import logging
import math
import sys
import time

from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError

BRIDGE_CDO = "/Script/RammsCore.Default__RammsCoreBridge"


# ── Discovery ───────────────────────────────────────────────────────


def find_pose_actors(ue: UnrealRemote) -> list[dict]:
    """Find actors with URammsSkeletalPoseComponent via the bridge."""
    bridge = ue.actor(BRIDGE_CDO)
    raw = bridge.call("FindSkeletalPoseActors")
    results = raw if isinstance(raw, list) else raw.get("ReturnValue", [])
    out = []
    for entry in results:
        parts = entry.split("|", 1)
        out.append({
            "actor_path": parts[0],
            "component_name": parts[1] if len(parts) > 1 else "",
        })
    return out


def find_pose_component(ue: UnrealRemote, actor_hint: str = ""):
    """
    Find an actor's SkeletalPoseComponent.

    Returns (actor_proxy, component_proxy) or (None, None).
    """
    if actor_hint:
        comps = ue.find_components(actor_hint, "SkeletalPose")
        if comps:
            return ue.actor(actor_hint), ue.actor(comps[0]["path"])
        return ue.actor(actor_hint), None

    actors = find_pose_actors(ue)
    if not actors:
        return None, None

    actor_path = actors[0]["actor_path"]
    comps = ue.find_components(actor_path, "SkeletalPose")
    if comps:
        return ue.actor(actor_path), ue.actor(comps[0]["path"])
    return ue.actor(actor_path), None


# ── Getters ─────────────────────────────────────────────────────────


def _extract_return(result, fallback=None):
    """Extract ReturnValue from UE Remote Control response."""
    if fallback is None:
        fallback = []
    if isinstance(result, list):
        return result
    if isinstance(result, dict):
        return result.get("ReturnValue", fallback)
    return fallback


def get_joint_values(comp) -> list:
    return _extract_return(comp.call("GetAllJointValues"))


def get_joint_targets(comp) -> list:
    return _extract_return(comp.call("GetAllJointTargets"))


def get_num_joints(comp) -> int:
    result = comp.call("GetNumJoints")
    if isinstance(result, dict):
        return int(result.get("ReturnValue", 0))
    return int(result) if result else 0


def get_mesh_names(comp) -> list:
    return _extract_return(comp.call("GetPoseableMeshNames"))


def get_num_meshes(comp) -> int:
    result = comp.call("GetNumPoseableMeshes")
    if isinstance(result, dict):
        return int(result.get("ReturnValue", 0))
    return int(result) if result else 0


# ── Setters ─────────────────────────────────────────────────────────


def set_all_joints(comp, values: list[float]):
    """Set all joint targets in a single remote call."""
    comp.call("SetAllJointTargets", Values=[float(v) for v in values])


def set_joint_by_name(comp, name: str, value: float):
    comp.call("SetJointTargetByName", JointName=name, Value=value)


def set_joint_by_index(comp, index: int, value: float):
    comp.call("SetJointTarget", JointIndex=index, Value=value)


def snap_to_targets(comp):
    comp.call("SnapToTargets")


# ── Display ─────────────────────────────────────────────────────────


def describe_pose(comp):
    """Print current joint state and mesh info."""
    meshes = get_mesh_names(comp)
    values = get_joint_values(comp)
    targets = get_joint_targets(comp)
    num = max(len(values), len(targets))

    print(f"\nSkeletal Pose — {num} joints, {len(meshes)} mesh(es)")
    if meshes:
        print(f"  Meshes: {', '.join(str(m) for m in meshes)}")
    print()
    print(f"  {'Idx':<5} {'Current':>10} {'Target':>10}")
    print(f"  {'-'*5} {'-'*10} {'-'*10}")
    for i in range(num):
        cur = values[i] if i < len(values) else 0.0
        tgt = targets[i] if i < len(targets) else 0.0
        print(f"  {i:<5} {cur:>10.2f} {tgt:>10.2f}")
    print()


# ── Sweep Animation ────────────────────────────────────────────────


def sweep_joint(comp, joint_index: int, lo: float, hi: float, period: float):
    """Continuously sweep a joint between lo and hi with given period."""
    if period <= 0:
        print(f"  Error: period must be positive, got {period}")
        return
    mid = (lo + hi) / 2.0
    amp = (hi - lo) / 2.0
    print(f"Sweeping joint {joint_index}: {lo}..{hi} (period={period}s)")
    print("Press Ctrl+C to stop.\n")

    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            value = mid + amp * math.sin(2.0 * math.pi * t / period)
            set_joint_by_index(comp, joint_index, value)
            cur = value
            sys.stdout.write(f"\r  Joint {joint_index}: {cur:8.2f}")
            sys.stdout.flush()
            time.sleep(0.033)  # ~30 Hz
    except KeyboardInterrupt:
        print("\nStopped.")


# ── Interactive Mode ────────────────────────────────────────────────


def interactive_mode(comp):
    """Interactive REPL for controlling poses."""
    print("\n=== Skeletal Pose Controller ===")
    print("Commands:")
    print("  state                       Show joint values and mesh info")
    print("  set <idx> <value>           Set joint by index")
    print("  setname <name> <value>      Set joint by name")
    print("  setall <v0> <v1> ...        Set all joints")
    print("  home                        All joints to 0")
    print("  snap                        Snap to targets (bypass interpolation)")
    print("  sweep <idx> <lo> <hi> [p]   Sweep joint (Ctrl+C to stop)")
    print("  meshes                      List poseable meshes")
    print("  quit                        Exit")
    print()

    while True:
        try:
            line = input("pose> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd in ("quit", "exit", "q"):
                break

            elif cmd in ("state", "describe", "values", "read"):
                describe_pose(comp)

            elif cmd == "set" and len(parts) >= 3:
                try:
                    idx = int(parts[1])
                    val = float(parts[2])
                    set_joint_by_index(comp, idx, val)
                    print(f"  Joint {idx} → {val}")
                except (ValueError, IndexError) as e:
                    print(f"  Error: {e}")

            elif cmd == "setname" and len(parts) >= 3:
                try:
                    name = parts[1]
                    val = float(parts[2])
                    set_joint_by_name(comp, name, val)
                    print(f"  Joint '{name}' → {val}")
                except (ValueError, IndexError) as e:
                    print(f"  Error: {e}")

            elif cmd == "setall" and len(parts) >= 2:
                try:
                    vals = [float(x) for x in parts[1:]]
                    n = get_num_joints(comp)
                    if n > 0 and len(vals) != n:
                        print(f"  Warning: provided {len(vals)} values but component has {n} joints")
                    set_all_joints(comp, vals)
                    print(f"  Set {len(vals)} joints: {vals}")
                except ValueError as e:
                    print(f"  Error: {e}")

            elif cmd == "home":
                n = get_num_joints(comp)
                if n == 0:
                    print("  No joints to home (joint count is 0)")
                else:
                    home = [0.0] * n
                    set_all_joints(comp, home)
                    print(f"  Homed {n} joints to 0")

            elif cmd == "snap":
                snap_to_targets(comp)
                print("  Snapped to targets")

            elif cmd == "sweep" and len(parts) >= 4:
                try:
                    idx = int(parts[1])
                    lo = float(parts[2])
                    hi = float(parts[3])
                    period = float(parts[4]) if len(parts) > 4 else 4.0
                    sweep_joint(comp, idx, lo, hi, period)
                except (ValueError, IndexError) as e:
                    print(f"  Error: {e}")

            elif cmd == "meshes":
                meshes = get_mesh_names(comp)
                n = get_num_meshes(comp)
                print(f"\n  {n} poseable mesh(es):")
                for m in meshes:
                    print(f"    - {m}")
                print()

            else:
                print(f"  Unknown command: {cmd}")

        except UnrealRemoteError as e:
            print(f"  Remote error: {e}")


# ── CLI ─────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Control skeletal mesh poses via Unreal Remote Control")
    parser.add_argument("--actor", default="",
                        help="Actor path (auto-discovered if omitted)")
    parser.add_argument("--component", default="",
                        help="Component path override")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=30010)
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--list", action="store_true",
                       help="List actors with SkeletalPoseComponent")
    group.add_argument("--describe", action="store_true",
                       help="Show current joint values and mesh info")
    group.add_argument("--set-all", nargs="+", type=float, metavar="VALUE",
                       help="Set all joint values")
    group.add_argument("--set", nargs=2, metavar=("NAME", "VALUE"),
                       help="Set one joint by name (e.g. --set Shoulder 45)")
    group.add_argument("--set-index", nargs=2, metavar=("IDX", "VALUE"),
                       help="Set one joint by index (e.g. --set-index 0 45)")
    group.add_argument("--home", action="store_true",
                       help="Send all joints to 0")
    group.add_argument("--sweep", nargs=3, metavar=("IDX", "LO", "HI"),
                       help="Sweep a joint between lo and hi (e.g. --sweep 0 -90 90)")
    group.add_argument("--interactive", "-i", action="store_true",
                       help="Interactive control mode")

    parser.add_argument("--period", type=float, default=4.0,
                        help="Sweep period in seconds (default: 4)")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    ue = UnrealRemote(host=args.host, http_port=args.port)
    print(f"Connecting to UE at http://{args.host}:{args.port}...")
    if not ue.ping():
        print("Connection failed!")
        sys.exit(1)
    print("Connected!\n")

    if args.list:
        print("Searching for actors with RammsSkeletalPoseComponent...")
        try:
            actors = find_pose_actors(ue)
        except UnrealRemoteError as e:
            print(f"Remote error: {e}")
            sys.exit(1)
        for a in actors:
            print(f"  Actor: {a['actor_path']}")
            print(f"  Component: {a['component_name']}")
            print()
        if not actors:
            print("  No actors with RammsSkeletalPoseComponent found")
        return

    # Find or connect to the component
    if args.component:
        comp = ue.actor(args.component)
    else:
        print("Searching for SkeletalPoseComponent...")
        try:
            actor, comp = find_pose_component(ue, args.actor)
        except UnrealRemoteError as e:
            print(f"Remote error: {e}")
            sys.exit(1)
        if not comp:
            print("No RammsSkeletalPoseComponent found!")
            print("Use --list to see available actors, or --actor / --component to specify.")
            sys.exit(1)
        print(f"Found: {comp.object_path}\n")

    try:
        if args.describe:
            describe_pose(comp)
        elif args.set_all:
            n = get_num_joints(comp)
            if n > 0 and len(args.set_all) != n:
                print(f"Warning: provided {len(args.set_all)} values but component has {n} joints")
            set_all_joints(comp, args.set_all)
            print(f"Set {len(args.set_all)} joints: {args.set_all}")
        elif args.set:
            name, val = args.set[0], float(args.set[1])
            set_joint_by_name(comp, name, val)
            print(f"Joint '{name}' → {val}")
        elif args.set_index:
            idx, val = int(args.set_index[0]), float(args.set_index[1])
            set_joint_by_index(comp, idx, val)
            print(f"Joint {idx} → {val}")
        elif args.home:
            n = get_num_joints(comp)
            if n == 0:
                print("No joints to home (joint count is 0)")
            else:
                home = [0.0] * n
                set_all_joints(comp, home)
                print(f"Homed {n} joints to 0")
        elif args.sweep:
            idx = int(args.sweep[0])
            lo, hi = float(args.sweep[1]), float(args.sweep[2])
            if args.period <= 0:
                print(f"Error: --period must be positive, got {args.period}")
                sys.exit(1)
            sweep_joint(comp, idx, lo, hi, args.period)
        elif args.interactive:
            interactive_mode(comp)
        else:
            parser.print_help()
    except UnrealRemoteError as e:
        print(f"Remote error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
