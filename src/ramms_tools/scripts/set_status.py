"""
Set robot status values on the RAMMS UI StatusPanel via Unreal Remote Control API.

Uses the URammsRemoteBridge function library which finds StatusPanel widgets
automatically via TObjectIterator (no actor path discovery needed).

Usage:
    python set_status.py                         # Interactive mode
    python set_status.py --battery 0.85          # Set battery level
    python set_status.py --speed 1.5             # Set speed (m/s)
    python set_status.py --mode Autonomous       # Set mode
    python set_status.py --estop                 # Activate e-stop
    python set_status.py --all                   # Set all fields with example values
    python set_status.py --discover              # List all RAMMS widgets and actors

The script connects to the UE Remote Control API (default localhost:30010)
and calls functions on URammsRemoteBridge to update StatusPanel widgets.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError

# Bridge CDO path (UI-specific bridge stays in RammsUI)
BRIDGE = "/Script/RammsUI.Default__RammsRemoteBridge"

# Mode name -> ERammsRobotMode enum string (UE Remote Control uses enum name strings)
MODE_MAP = {
    "unknown": "ERammsRobotMode::Unknown",
    "standby": "ERammsRobotMode::Standby",
    "manual": "ERammsRobotMode::Manual",
    "autonomous": "ERammsRobotMode::Autonomous",
    "emergency": "ERammsRobotMode::Emergency",
}


def call_bridge(ue: UnrealRemote, func: str, **params) -> any:
    """Call a function on the RammsRemoteBridge CDO."""
    return ue._call_function(BRIDGE, func, params if params else None)


def set_battery(ue: UnrealRemote, level: float) -> int:
    """Set battery level (0.0-1.0). Returns number of panels updated."""
    result = call_bridge(ue, "SetBatteryLevel", Level=level)
    count = result if isinstance(result, int) else 0
    print(f"  ✓ Battery: {int(level * 100)}% ({count} panel(s))")
    return count


def set_speed(ue: UnrealRemote, speed: float) -> int:
    """Set speed in m/s. Returns number of panels updated."""
    result = call_bridge(ue, "SetSpeed", SpeedMPS=speed)
    count = result if isinstance(result, int) else 0
    print(f"  ✓ Speed: {speed:.2f} m/s ({count} panel(s))")
    return count


def set_mode(ue: UnrealRemote, mode_name: str) -> int:
    """Set robot mode by name. Returns number of panels updated."""
    mode_val = MODE_MAP.get(mode_name.lower())
    if mode_val is None:
        print(f"  ✗ Unknown mode '{mode_name}'. Valid: {', '.join(MODE_MAP.keys())}")
        return 0
    result = call_bridge(ue, "SetRobotMode", Mode=mode_val)
    count = result if isinstance(result, int) else 0
    print(f"  ✓ Mode: {mode_name} ({count} panel(s))")
    return count


def set_estop(ue: UnrealRemote, active: bool) -> int:
    """Set emergency stop. Returns number of panels updated."""
    result = call_bridge(ue, "SetEmergencyStop", bActive=active)
    count = result if isinstance(result, int) else 0
    status = "ACTIVE" if active else "cleared"
    print(f"  ✓ E-Stop: {status} ({count} panel(s))")
    return count


def set_full_state(ue: UnrealRemote, *,
                   speed: float = 0.0,
                   battery: float = 0.0,
                   mode: str = "Unknown",
                   estop: bool = False) -> int:
    """Set the full robot state at once."""
    mode_val = MODE_MAP.get(mode.lower(), "ERammsRobotMode::Unknown")
    state = {
        "Mode": mode_val,
        "BatteryLevel": battery,
        "LinearVelocity": {"X": speed, "Y": 0, "Z": 0},
        "AngularVelocity": {"Pitch": 0, "Yaw": 0, "Roll": 0},
        "bEmergencyStop": estop,
        "Timestamp": int(time.time() * 1_000_000),
    }
    result = call_bridge(ue, "SetRobotState", State=state)
    count = result if isinstance(result, int) else 0
    print(f"  ✓ Full state set ({count} panel(s))")
    return count


def discover(ue: UnrealRemote) -> None:
    """Discover all RAMMS widgets and actors."""
    print("=== RAMMS Widgets (via TObjectIterator) ===")
    try:
        result = call_bridge(ue, "GetAllRammsWidgetPaths")
        paths = result if isinstance(result, list) else []
        if paths:
            for p in paths:
                print(f"  {p}")
        else:
            print("  (none found - is a RAMMS widget added to viewport?)")
    except UnrealRemoteError as e:
        print(f"  Failed: {e}")
        print("  Note: RammsRemoteBridge must be compiled into the plugin.")
        return

    print("\n=== Actors (via GEngine WorldContexts) ===")
    try:
        result = call_bridge(ue, "GetAllActorPaths")
        paths = result if isinstance(result, list) else []
        if paths:
            for p in paths[:50]:
                print(f"  {p}")
            if len(paths) > 50:
                print(f"  ... and {len(paths) - 50} more")
        else:
            print("  (none found - is a level loaded?)")
    except UnrealRemoteError as e:
        print(f"  Failed: {e}")

    print("\n=== Current Robot State ===")
    try:
        result = ue._request("PUT", "/remote/object/call", {
            "objectPath": BRIDGE,
            "functionName": "GetRobotState",
        })
        if isinstance(result, dict):
            state = result.get("OutState", result)
            print(f"  Mode: {state.get('Mode', '?')}")
            print(f"  Battery: {state.get('BatteryLevel', '?')}")
            vel = state.get("LinearVelocity", {})
            speed = (vel.get("X", 0)**2 + vel.get("Y", 0)**2 + vel.get("Z", 0)**2)**0.5
            print(f"  Speed: {speed:.2f} m/s")
            print(f"  E-Stop: {state.get('bEmergencyStop', '?')}")
    except UnrealRemoteError as e:
        print(f"  Failed: {e}")


def interactive_mode(ue: UnrealRemote) -> None:
    """Interactive REPL for updating status values."""
    print("\n=== RAMMS Status Control (Interactive) ===")
    print("Commands:")
    print("  speed <value>       Set speed in m/s")
    print("  battery <0-1>       Set battery level (0.0 to 1.0)")
    print("  mode <mode>         Set mode (Standby/Manual/Autonomous/Emergency)")
    print("  estop               Toggle e-stop")
    print("  state               Show current state")
    print("  widgets             List RAMMS widget instances")
    print("  demo                Run a demo sequence")
    print("  quit                Exit")
    print()

    estop_active = False

    while True:
        try:
            cmd = input("status> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not cmd:
            continue

        parts = cmd.split(maxsplit=1)
        command = parts[0].lower()
        arg = parts[1] if len(parts) > 1 else ""

        try:
            if command in ("quit", "exit", "q"):
                break
            elif command == "speed":
                set_speed(ue, float(arg))
            elif command == "battery":
                set_battery(ue, float(arg))
            elif command == "mode":
                set_mode(ue, arg)
            elif command == "estop":
                estop_active = not estop_active
                set_estop(ue, estop_active)
            elif command == "state":
                discover(ue)
            elif command == "widgets":
                try:
                    result = call_bridge(ue, "GetAllRammsWidgetPaths")
                    paths = result if isinstance(result, list) else []
                    for p in paths:
                        print(f"  {p}")
                    if not paths:
                        print("  (none)")
                except UnrealRemoteError as e:
                    print(f"  Failed: {e}")
            elif command == "demo":
                run_demo(ue)
            else:
                print(f"  Unknown command: {command}")
        except ValueError as e:
            print(f"  Invalid value: {e}")
        except UnrealRemoteError as e:
            print(f"  Remote error: {e}")


def run_demo(ue: UnrealRemote) -> None:
    """Run a demo sequence cycling through various states."""
    print("  Running demo sequence...")

    states = [
        {"speed": 0.0, "battery": 1.0, "mode": "Standby", "estop": False},
        {"speed": 0.5, "battery": 0.95, "mode": "Manual", "estop": False},
        {"speed": 1.2, "battery": 0.80, "mode": "Autonomous", "estop": False},
        {"speed": 0.8, "battery": 0.60, "mode": "Autonomous", "estop": False},
        {"speed": 0.3, "battery": 0.35, "mode": "Manual", "estop": False},
        {"speed": 0.0, "battery": 0.15, "mode": "Standby", "estop": False},
        {"speed": 0.0, "battery": 0.10, "mode": "Emergency", "estop": True},
        {"speed": 0.0, "battery": 0.10, "mode": "Standby", "estop": False},
    ]

    for i, state in enumerate(states):
        print(f"\n  --- State {i + 1}/{len(states)} ---")
        set_full_state(ue, **state)
        if i < len(states) - 1:
            time.sleep(2.0)

    print("\n  Demo complete!")


def main():
    parser = argparse.ArgumentParser(
        description="Control RAMMS StatusPanel via Unreal Remote Control API"
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="UE host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=30010,
                        help="HTTP API port (default: 30010)")

    # Direct value setters
    parser.add_argument("--speed", type=float, default=None,
                        help="Set speed value (m/s)")
    parser.add_argument("--battery", type=float, default=None,
                        help="Set battery level (0.0 to 1.0)")
    parser.add_argument("--mode", default=None,
                        help="Set mode (Standby/Manual/Autonomous/Emergency)")
    parser.add_argument("--estop", action="store_true",
                        help="Set emergency stop active")
    parser.add_argument("--all", action="store_true",
                        help="Set all fields with example values")
    parser.add_argument("--demo", action="store_true",
                        help="Run demo sequence")
    parser.add_argument("--discover", action="store_true",
                        help="Discover RAMMS widgets and actors")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    ue = UnrealRemote(host=args.host, http_port=args.port,
                      ui_bridge="/Script/RammsUI.Default__RammsRemoteBridge")

    # Check connectivity
    print(f"Connecting to UE Remote Control at {ue.base_url}...")
    if not ue.ping():
        print("ERROR: Cannot reach Unreal Remote Control API.")
        print("Make sure:")
        print("  1. The Remote Control API plugin is enabled in UE")
        print("  2. The editor/game is running")
        print("  3. WebControl.StartServer has been run (or auto-start is enabled)")
        sys.exit(1)
    print("Connected!\n")

    if args.discover:
        discover(ue)
        return

    # Handle commands
    has_direct_args = any([
        args.speed is not None, args.battery is not None,
        args.mode is not None, args.estop,
    ])

    if args.all:
        set_full_state(ue, speed=1.23, battery=0.75, mode="Autonomous")
    elif args.demo:
        run_demo(ue)
    elif has_direct_args:
        if args.speed is not None:
            set_speed(ue, args.speed)
        if args.battery is not None:
            set_battery(ue, args.battery)
        if args.mode is not None:
            set_mode(ue, args.mode)
        if args.estop:
            set_estop(ue, True)
    else:
        interactive_mode(ue)


if __name__ == "__main__":
    main()
