#!/usr/bin/env python3
"""
Control KinovaGen3 arm joint positions via Unreal Engine Remote Control API.

Usage:
    python control_kinova.py --list-actors             # Find actors with Kinova component
    python control_kinova.py --describe                # Show joints and current angles
    python control_kinova.py --set-all 0 15 180 -150 0 -10 90  # Set all 7 joints
    python control_kinova.py --set-joint 0 45.0        # Set joint 0 to 45 degrees
    python control_kinova.py --interactive             # Interactive REPL
    python control_kinova.py --home                    # Send all joints to 0

Actor/component discovery:
    By default the script finds the first actor with a KinovaGen3ControllerComponent.
    Use --actor to specify an actor path, and --component for the component name.

Requires Unreal Engine running with Remote Control API plugin enabled (port 30010).
"""

import argparse
import logging
import json
import sys

from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError


def find_kinova_actor(ue: UnrealRemote, actor_hint: str = "") -> tuple:
    """
    Find an actor with a KinovaGen3ControllerComponent.

    Returns (actor_proxy, component_proxy) or (None, None).
    """
    if actor_hint:
        actor = ue.actor(actor_hint)
        comp = _find_component(ue, actor.object_path, "KinovaGen3")
        return (actor, comp) if comp else (actor, None)

    # Single server-side call to find actors with matching component
    results = ue.find_actors_by_component("KinovaGen3")
    if results:
        r = results[0]
        return r["actor_proxy"], ue.actor(r["component_path"])

    # Fallback: iterate actors (slower)
    actors = ue.find_actors()
    for actor in actors:
        comp = _find_component(ue, actor.object_path, "KinovaGen3")
        if comp:
            return actor, comp
    return None, None


def _find_component(ue: UnrealRemote, actor_path: str, class_hint: str):
    """
    Find a component on an actor whose class name contains class_hint.

    Uses ue.find_components() which resolves actual component instance names
    (not UPROPERTY variable names) for correct Remote Control object paths.
    """
    comps = ue.find_components(actor_path, class_hint)
    if comps:
        return ue.actor(comps[0]["path"])
    return None


def get_joint_angles(comp) -> list:
    """Read current joint angles from the component."""
    result = comp.call("GetAllJointAngles")
    if isinstance(result, list):
        return result
    if isinstance(result, dict):
        return result.get("ReturnValue", result.get("OutAngles", []))
    return []


def set_all_joints(comp, angles: list[float]):
    """Set all joint targets in a single remote call."""
    comp.call("SetAllJointTargets", TargetAngles=[float(a) for a in angles])


def set_joint(comp, index: int, angle: float):
    """Set a single joint target."""
    comp.call("SetJointTarget", JointIndex=index, TargetAngle=angle)


def describe_kinova(comp):
    """Print current joint state."""
    angles = get_joint_angles(comp)
    print(f"\nKinova Gen3 — {len(angles)} joints:")
    print("-" * 35)
    for i, angle in enumerate(angles):
        print(f"  Joint {i}: {angle:8.2f}°")
    print()


def interactive_mode(comp):
    """Interactive REPL for controlling the arm."""
    print("\n=== Kinova Gen3 Controller ===")
    print("Commands:")
    print("  angles                     Show current joint angles")
    print("  set <idx> <angle>          Set one joint (e.g. 'set 2 90')")
    print("  setall <a0> <a1> ... <a6>  Set all joints")
    print("  home                       All joints to 0")
    print("  quit                       Exit")
    print()

    while True:
        try:
            line = input("kinova> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            break

        elif cmd in ("angles", "state", "read"):
            describe_kinova(comp)

        elif cmd == "set" and len(parts) >= 3:
            try:
                idx = int(parts[1])
                angle = float(parts[2])
                set_joint(comp, idx, angle)
                print(f"  Joint {idx} → {angle}°")
            except (ValueError, IndexError) as e:
                print(f"  Error: {e}")

        elif cmd == "setall" and len(parts) >= 2:
            try:
                angles = [float(x) for x in parts[1:]]
                set_all_joints(comp, angles)
                print(f"  Set {len(angles)} joints: {angles}")
            except ValueError as e:
                print(f"  Error: {e}")

        elif cmd == "home":
            angles = get_joint_angles(comp)
            home = [0.0] * len(angles) if angles else [0.0] * 7
            set_all_joints(comp, home)
            print(f"  Homed {len(home)} joints to 0°")

        else:
            print(f"  Unknown command: {cmd}")


def main():
    parser = argparse.ArgumentParser(
        description="Control KinovaGen3 arm via Unreal Remote Control")
    parser.add_argument("--actor", default="",
                        help="Actor path (auto-discovered if omitted)")
    parser.add_argument("--component", default="",
                        help="Component name override")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=30010)
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging (shows raw API requests/responses)")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--list-actors", action="store_true",
                       help="List actors with Kinova components")
    group.add_argument("--describe", action="store_true",
                       help="Show current joint angles")
    group.add_argument("--set-all", nargs="+", type=float, metavar="ANGLE",
                       help="Set all joint angles (e.g. --set-all 0 15 180 -150 0 -10 90)")
    group.add_argument("--set-joint", nargs=2, metavar=("IDX", "ANGLE"),
                       help="Set one joint (e.g. --set-joint 2 90)")
    group.add_argument("--home", action="store_true",
                       help="Send all joints to 0 degrees")
    group.add_argument("--interactive", "-i", action="store_true",
                       help="Interactive control mode")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    ue = UnrealRemote(host=args.host, http_port=args.port)
    print(f"Connecting to UE at http://{args.host}:{args.port}...")
    if not ue.ping():
        print("Connection failed!")
        sys.exit(1)
    print("Connected!\n")

    if args.list_actors:
        print("Searching for actors with KinovaGen3ControllerComponent...")
        results = ue.find_actors_by_component("KinovaGen3")
        for r in results:
            print(f"  Actor: {r['actor_path']}")
            print(f"  Component: {r['component_path']} ({r['component_class']})")
            print()
        if not results:
            print("  No actors with KinovaGen3ControllerComponent found")
        return

    # Find or connect to the component
    if args.component:
        # User provided explicit component path
        comp = ue.actor(args.component)
    else:
        print("Searching for Kinova controller...")
        actor, comp = find_kinova_actor(ue, args.actor)
        if not comp:
            print("No KinovaGen3ControllerComponent found!")
            print("Use --list-actors to see available actors, or --actor / --component to specify.")
            sys.exit(1)
        print(f"Found: {comp.object_path}\n")

    if args.describe:
        describe_kinova(comp)
    elif args.set_all:
        set_all_joints(comp, args.set_all)
        print(f"Set {len(args.set_all)} joints: {args.set_all}")
    elif args.set_joint:
        idx, angle = int(args.set_joint[0]), float(args.set_joint[1])
        set_joint(comp, idx, angle)
        print(f"Joint {idx} → {angle}°")
    elif args.home:
        angles = get_joint_angles(comp)
        home = [0.0] * len(angles) if angles else [0.0] * 7
        set_all_joints(comp, home)
        print(f"Homed {len(home)} joints to 0°")
    elif args.interactive:
        interactive_mode(comp)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
