#!/usr/bin/env python3
"""
Control a GripperControllerComponent via Unreal Engine Remote Control API.

Usage:
    ramms-gripper --describe                 # Show gripper state and finger angles
    ramms-gripper --open                     # Open the gripper
    ramms-gripper --close                    # Close the gripper
    ramms-gripper --toggle                   # Toggle open/closed
    ramms-gripper --set-fingers 45.0 45.0    # Set individual finger angles
    ramms-gripper --speed 2.0                # Set motor speed multiplier
    ramms-gripper --interactive              # Interactive REPL
    ramms-gripper --list-actors              # Find actors with gripper components

Actor/component discovery:
    By default the script finds the first actor with a GripperControllerComponent.
    Use --actor to specify an actor path, and --component for the component name.

Requires Unreal Engine running with Remote Control API plugin enabled (port 30010).
"""

from __future__ import annotations

import argparse
import logging
import sys

from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError

# EGripperState enum values (UE Remote Control uses enum name strings)
GRIPPER_STATE_MAP = {
    "open": "EGripperState::Open",
    "closed": "EGripperState::Closed",
    "opening": "EGripperState::Opening",
    "closing": "EGripperState::Closing",
}


def find_gripper(ue: UnrealRemote, actor_hint: str = "") -> tuple:
    """Find an actor with a GripperControllerComponent.

    Returns (actor_proxy, component_proxy) or (None, None).
    """
    if actor_hint:
        actor = ue.actor(actor_hint)
        comp = _find_component(ue, actor.object_path, "GripperController")
        return (actor, comp) if comp else (actor, None)

    results = ue.find_actors_by_component("GripperController")
    if results:
        r = results[0]
        return r["actor_proxy"], ue.actor(r["component_path"])

    # Fallback: iterate actors
    actors = ue.find_actors()
    for actor in actors:
        comp = _find_component(ue, actor.object_path, "GripperController")
        if comp:
            return actor, comp
    return None, None


def _find_component(ue: UnrealRemote, actor_path: str, class_hint: str):
    """Find a component on an actor whose class/instance name contains class_hint."""
    comps = ue.find_components(actor_path, class_hint)
    if comps:
        return ue.actor(comps[0]["path"])
    return None


def get_gripper_state(comp) -> str:
    """Read current gripper state as a string."""
    result = comp.call("GetGripperState")
    if isinstance(result, str):
        # Strip enum prefix if present
        return result.split("::")[-1] if "::" in result else result
    return str(result) if result is not None else "Unknown"


def get_finger_angles(comp) -> tuple[float, float]:
    """Read current finger angles. Returns (finger1, finger2)."""
    result = comp.call("GetFingerAngles")
    if isinstance(result, dict):
        f1 = result.get("OutFinger1Angle", result.get("Finger1Angle", 0.0))
        f2 = result.get("OutFinger2Angle", result.get("Finger2Angle", 0.0))
        return float(f1), float(f2)
    return 0.0, 0.0


def is_open(comp) -> bool:
    """Check if gripper is fully open."""
    result = comp.call("IsOpen")
    return bool(result)


def is_closed(comp) -> bool:
    """Check if gripper is fully closed."""
    result = comp.call("IsClosed")
    return bool(result)


def open_gripper(comp) -> None:
    """Open the gripper."""
    comp.call("Open")


def close_gripper(comp) -> None:
    """Close the gripper."""
    comp.call("Close")


def toggle_gripper(comp) -> None:
    """Toggle the gripper open/closed."""
    comp.call("Toggle")


def set_finger_angles(comp, finger1: float, finger2: float) -> None:
    """Set individual finger angles."""
    comp.call("SetFingerAngles",
              Finger1Angle=finger1, Finger2Angle=finger2)


def set_speed_multiplier(comp, multiplier: float) -> None:
    """Set the motor speed multiplier."""
    comp.call("SetMotorSpeedMultiplier", SpeedMultiplier=multiplier)


def describe_gripper(comp) -> None:
    """Print current gripper state."""
    state = get_gripper_state(comp)
    f1, f2 = get_finger_angles(comp)

    print(f"\nGripper Controller:")
    print("-" * 35)
    print(f"  State:   {state}")
    print(f"  Finger1: {f1:7.2f}°")
    print(f"  Finger2: {f2:7.2f}°")
    print()


def interactive_mode(comp) -> None:
    """Interactive REPL for controlling the gripper."""
    print("\n=== Gripper Controller ===")
    print("Commands:")
    print("  state                      Show gripper state & finger angles")
    print("  open                       Open the gripper")
    print("  close                      Close the gripper")
    print("  toggle                     Toggle open/closed")
    print("  fingers <f1> <f2>          Set finger angles (degrees)")
    print("  speed <multiplier>         Set motor speed multiplier")
    print("  quit                       Exit")
    print()

    while True:
        try:
            line = input("gripper> ").strip()
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
            elif cmd in ("state", "describe", "read"):
                describe_gripper(comp)
            elif cmd == "open":
                open_gripper(comp)
                print("  → Opening gripper")
            elif cmd == "close":
                close_gripper(comp)
                print("  → Closing gripper")
            elif cmd == "toggle":
                toggle_gripper(comp)
                print("  → Toggling gripper")
            elif cmd == "fingers" and len(parts) >= 3:
                f1, f2 = float(parts[1]), float(parts[2])
                set_finger_angles(comp, f1, f2)
                print(f"  → Fingers: {f1:.1f}°, {f2:.1f}°")
            elif cmd == "speed" and len(parts) >= 2:
                mult = float(parts[1])
                set_speed_multiplier(comp, mult)
                print(f"  → Speed multiplier: {mult:.2f}")
            else:
                print(f"  Unknown command: {cmd}")
        except (ValueError, UnrealRemoteError) as e:
            print(f"  Error: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Control GripperControllerComponent via Unreal Remote Control")
    parser.add_argument("--actor", default="",
                        help="Actor path (auto-discovered if omitted)")
    parser.add_argument("--component", default="",
                        help="Component name override")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=30010)
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--list-actors", action="store_true",
                       help="List actors with Gripper components")
    group.add_argument("--describe", action="store_true",
                       help="Show gripper state and finger angles")
    group.add_argument("--open", action="store_true",
                       help="Open the gripper")
    group.add_argument("--close", action="store_true",
                       help="Close the gripper")
    group.add_argument("--toggle", action="store_true",
                       help="Toggle gripper open/closed")
    group.add_argument("--set-fingers", nargs=2, type=float,
                       metavar=("F1", "F2"),
                       help="Set finger angles (degrees)")
    group.add_argument("--speed", type=float, metavar="MULT",
                       help="Set motor speed multiplier")
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
        print("Searching for actors with GripperControllerComponent...")
        results = ue.find_actors_by_component("GripperController")
        for r in results:
            print(f"  Actor: {r['actor_path']}")
            print(f"  Component: {r['component_path']} ({r['component_class']})")
            print()
        if not results:
            print("  No actors with GripperControllerComponent found")
        return

    # Find or connect to the component
    if args.component:
        comp = ue.actor(args.component)
    else:
        print("Searching for gripper controller...")
        actor, comp = find_gripper(ue, args.actor)
        if not comp:
            print("No GripperControllerComponent found!")
            print("Use --list-actors to see available actors, "
                  "or --actor / --component to specify.")
            sys.exit(1)
        print(f"Found: {comp.object_path}\n")

    if args.describe:
        describe_gripper(comp)
    elif args.open:
        open_gripper(comp)
        print("Gripper opening")
    elif args.close:
        close_gripper(comp)
        print("Gripper closing")
    elif args.toggle:
        toggle_gripper(comp)
        print("Gripper toggled")
    elif args.set_fingers:
        f1, f2 = args.set_fingers
        set_finger_angles(comp, f1, f2)
        print(f"Finger angles set: {f1:.1f}°, {f2:.1f}°")
    elif args.speed is not None:
        set_speed_multiplier(comp, args.speed)
        print(f"Speed multiplier set: {args.speed:.2f}")
    elif args.interactive:
        interactive_mode(comp)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
