#!/usr/bin/env python3
"""
Control Mebot motor positions via Unreal Engine Remote Control API.

Usage:
    python control_mebot.py --list-actors              # Find actors with Mebot component
    python control_mebot.py --describe                 # Show motors and current positions
    python control_mebot.py --set-angular Yaw 45.0     # Set angular motor target
    python control_mebot.py --set-linear Lift 50.0     # Set linear motor target (cm)
    python control_mebot.py --interactive              # Interactive REPL
    python control_mebot.py --home                     # All motors to 0

Actor/component discovery:
    By default the script finds the first actor with a MebotControllerComponent.
    Use --actor to specify an actor path, and --component for the component name.

Requires Unreal Engine running with Remote Control API plugin enabled (port 30010).
"""

import argparse
import sys

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else __file__.rsplit("/", 1)[0])
from unreal_remote import UnrealRemote, UnrealRemoteError


def find_mebot_actor(ue: UnrealRemote, actor_hint: str = "") -> tuple:
    """
    Find an actor with a MebotControllerComponent.

    Returns (actor_proxy, component_proxy) or (None, None).
    """
    if actor_hint:
        actor = ue.actor(actor_hint)
        comp = _find_component_on_actor(ue, actor.object_path, "MebotController")
        return (actor, comp) if comp else (actor, None)

    actors = ue.find_actors()
    for actor in actors:
        comp = _find_component_on_actor(ue, actor.object_path, "MebotController")
        if comp:
            return actor, comp
    return None, None


def _find_component_on_actor(ue: UnrealRemote, actor_path: str, class_hint: str):
    """
    Find a component on an actor whose type contains class_hint.

    Uses the built-in /remote/object/describe endpoint.
    """
    try:
        desc = ue.describe_object(actor_path)
    except UnrealRemoteError:
        return None

    for prop in desc.get("Properties", []):
        prop_type = prop.get("Type", "")
        if class_hint.lower() in prop_type.lower() and "Component" in prop_type:
            comp_name = prop.get("Name", "")
            comp_path = f"{actor_path}.{comp_name}"
            return ue.actor(comp_path)
    return None


def get_angular_motors(comp) -> list[dict]:
    """Read angular motor configs from the component."""
    try:
        result = comp.get_property("AngularMotors")
        if isinstance(result, list):
            return result
    except UnrealRemoteError:
        pass
    return []


def get_linear_motors(comp) -> list[dict]:
    """Read linear motor configs from the component."""
    try:
        result = comp.get_property("LinearMotors")
        if isinstance(result, list):
            return result
    except UnrealRemoteError:
        pass
    return []


def describe_mebot(comp):
    """Print current motor state."""
    print("\nMebot Controller")
    print("=" * 50)

    angular = get_angular_motors(comp)
    if angular:
        print(f"\n  Angular Motors ({len(angular)}):")
        print("  " + "-" * 40)
        for m in angular:
            name = m.get("ConstraintName", "?")
            target = m.get("TargetAngle", 0)
            speed = m.get("MaxSpeed", 0)
            print(f"    {name:20s}  target={target:8.2f}°  maxSpeed={speed:.0f}°/s")

    linear = get_linear_motors(comp)
    if linear:
        print(f"\n  Linear Motors ({len(linear)}):")
        print("  " + "-" * 40)
        for m in linear:
            name = m.get("ConstraintName", "?")
            target = m.get("TargetPosition", 0)
            speed = m.get("MaxSpeed", 0)
            print(f"    {name:20s}  target={target:8.2f}cm  maxSpeed={speed:.0f}cm/s")

    if not angular and not linear:
        print("  No motors found (try --describe after motors are initialized)")
    print()


def set_angular_motor(comp, name: str, angle: float):
    """Set an angular motor target angle."""
    comp.call("SetAngularMotorTarget", MotorName=name, TargetAngle=angle)


def set_linear_motor(comp, name: str, position: float):
    """Set a linear motor target position in cm."""
    comp.call("SetLinearMotorTarget", MotorName=name, TargetPosition=position)


def home_all(comp):
    """Set all motors to 0."""
    count = 0
    for m in get_angular_motors(comp):
        name = m.get("ConstraintName", "")
        if name:
            set_angular_motor(comp, name, 0.0)
            count += 1
    for m in get_linear_motors(comp):
        name = m.get("ConstraintName", "")
        if name:
            set_linear_motor(comp, name, 0.0)
            count += 1
    return count


def interactive_mode(comp):
    """Interactive REPL for controlling mebot motors."""
    print("\n=== Mebot Motor Controller ===")
    print("Commands:")
    print("  motors                          Show all motors")
    print("  angular <name> <angle>          Set angular motor (degrees)")
    print("  linear  <name> <position>       Set linear motor (cm)")
    print("  home                            All motors to 0")
    print("  quit                            Exit")
    print()

    while True:
        try:
            line = input("mebot> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            break

        elif cmd in ("motors", "state", "read", "describe"):
            describe_mebot(comp)

        elif cmd in ("angular", "ang", "a") and len(parts) >= 3:
            try:
                name = parts[1]
                angle = float(parts[2])
                set_angular_motor(comp, name, angle)
                print(f"  Angular {name} → {angle}°")
            except (ValueError, UnrealRemoteError) as e:
                print(f"  Error: {e}")

        elif cmd in ("linear", "lin", "l") and len(parts) >= 3:
            try:
                name = parts[1]
                pos = float(parts[2])
                set_linear_motor(comp, name, pos)
                print(f"  Linear {name} → {pos}cm")
            except (ValueError, UnrealRemoteError) as e:
                print(f"  Error: {e}")

        elif cmd == "home":
            count = home_all(comp)
            print(f"  Homed {count} motors to 0")

        else:
            print(f"  Unknown command: {cmd}")


def main():
    parser = argparse.ArgumentParser(
        description="Control Mebot motors via Unreal Remote Control")
    parser.add_argument("--actor", default="",
                        help="Actor path (auto-discovered if omitted)")
    parser.add_argument("--component", default="",
                        help="Component path override")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=30010)

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--list-actors", action="store_true",
                       help="List actors with Mebot components")
    group.add_argument("--describe", action="store_true",
                       help="Show motors and current state")
    group.add_argument("--set-angular", nargs=2, metavar=("NAME", "ANGLE"),
                       help="Set angular motor target (degrees)")
    group.add_argument("--set-linear", nargs=2, metavar=("NAME", "POS"),
                       help="Set linear motor target (cm)")
    group.add_argument("--home", action="store_true",
                       help="All motors to 0")
    group.add_argument("--interactive", "-i", action="store_true",
                       help="Interactive control mode")

    args = parser.parse_args()

    ue = UnrealRemote(host=args.host, http_port=args.port)
    print(f"Connecting to UE at http://{args.host}:{args.port}...")
    if not ue.ping():
        print("Connection failed!")
        sys.exit(1)
    print("Connected!\n")

    if args.list_actors:
        print("Searching for actors with MebotControllerComponent...")
        actors = ue.find_actors()
        found = 0
        for actor in actors:
            comp = _find_component_on_actor(ue, actor.object_path, "MebotController")
            if comp:
                found += 1
                print(f"  Actor: {actor.object_path}")
                print(f"  Component: {comp.object_path}")
                print()
        if not found:
            print("  No actors with MebotControllerComponent found")
        return

    # Find or connect to the component
    if args.component:
        comp = ue.actor(args.component)
    else:
        print("Searching for Mebot controller...")
        actor, comp = find_mebot_actor(ue, args.actor)
        if not comp:
            print("No MebotControllerComponent found!")
            print("Use --list-actors to see available actors, or --actor / --component to specify.")
            sys.exit(1)
        print(f"Found: {comp.object_path}\n")

    if args.describe:
        describe_mebot(comp)
    elif args.set_angular:
        name, angle = args.set_angular[0], float(args.set_angular[1])
        set_angular_motor(comp, name, angle)
        print(f"Angular {name} → {angle}°")
    elif args.set_linear:
        name, pos = args.set_linear[0], float(args.set_linear[1])
        set_linear_motor(comp, name, pos)
        print(f"Linear {name} → {pos}cm")
    elif args.home:
        count = home_all(comp)
        print(f"Homed {count} motors to 0")
    elif args.interactive:
        interactive_mode(comp)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
