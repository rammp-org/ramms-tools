#!/usr/bin/env python3
"""
Send notifications to the RAMMS UI via Unreal Engine Remote Control API.

Usage:
    python send_notification.py "Task complete!"
    python send_notification.py "Low battery" --level Warning --duration 8
    python send_notification.py "Mission accomplished" --level Success --title "Robot"
    python send_notification.py "Emergency stop triggered" --level Error --duration 0
    python send_notification.py --dismiss-all
    python send_notification.py --interactive
    python send_notification.py --demo

Requires Unreal Engine running with Remote Control API plugin enabled (port 30010).
"""

import argparse
import sys
import time

from ramms_tools.unreal_remote import UnrealRemote

# Map friendly level names to UE enum strings
LEVEL_MAP = {
    "info":    "ERammsNotificationLevel::Info",
    "success": "ERammsNotificationLevel::Success",
    "warning": "ERammsNotificationLevel::Warning",
    "error":   "ERammsNotificationLevel::Error",
}


def send_notification(ue: UnrealRemote, message: str, level: str = "info",
                      duration: float = 4.0, title: str = "") -> bool:
    """Send a notification toast to all RAMMS UI viewports."""
    level_enum = LEVEL_MAP.get(level.lower())
    if not level_enum:
        print(f"Unknown level '{level}'. Valid: {', '.join(LEVEL_MAP.keys())}")
        return False

    result = ue.ui_bridge.ShowNotification(
        Message=message,
        Level=level_enum,
        Duration=duration,
        Title=title,
    )
    # _call_function auto-extracts ReturnValue, so result is already bool
    return bool(result)


def dismiss_all(ue: UnrealRemote) -> int:
    """Dismiss all active notifications."""
    result = ue.ui_bridge.DismissAllNotifications()
    return int(result) if result else 0


def interactive_mode(ue: UnrealRemote):
    """Interactive REPL for sending notifications."""
    print("\n=== RAMMS Notification REPL ===")
    print("Commands:")
    print("  <message>                  Send info notification")
    print("  !info <msg>                Info notification")
    print("  !success <msg>             Success notification")
    print("  !warning <msg>             Warning notification")
    print("  !error <msg>               Error notification")
    print("  !title <title> | <msg>     Notification with title")
    print("  !duration <secs> <msg>     Custom duration")
    print("  !dismiss                   Dismiss all")
    print("  !quit                      Exit")
    print()

    while True:
        try:
            line = input("notify> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not line:
            continue

        if line.lower() in ("!quit", "!exit", "!q"):
            break

        if line.lower() == "!dismiss":
            count = dismiss_all(ue)
            print(f"Dismissed {count} notification(s)")
            continue

        # Parse level prefix
        level = "info"
        duration = 4.0
        title = ""
        msg = line

        if line.startswith("!"):
            parts = line.split(None, 1)
            cmd = parts[0].lower().lstrip("!")
            rest = parts[1] if len(parts) > 1 else ""

            if cmd in LEVEL_MAP:
                level = cmd
                msg = rest
            elif cmd == "title":
                if "|" in rest:
                    title, msg = rest.split("|", 1)
                    title = title.strip()
                    msg = msg.strip()
                else:
                    msg = rest
            elif cmd == "duration":
                dur_parts = rest.split(None, 1)
                if len(dur_parts) >= 2:
                    try:
                        duration = float(dur_parts[0])
                    except ValueError:
                        pass
                    msg = dur_parts[1]
                else:
                    msg = rest
            else:
                print(f"Unknown command: !{cmd}")
                continue

        if not msg:
            print("No message provided")
            continue

        ok = send_notification(ue, msg, level, duration, title)
        if ok:
            print(f"  [{level.upper()}] {msg}")
        else:
            print("  Failed to send notification")


def demo_mode(ue: UnrealRemote):
    """Send a series of demo notifications."""
    demos = [
        ("System initialized", "info", 5.0, "System"),
        ("Navigation waypoint reached", "success", 4.0, "Navigation"),
        ("Battery below 20%", "warning", 6.0, "Battery"),
        ("Motor controller fault", "error", 8.0, "Hardware"),
    ]

    print("\nSending demo notifications...")
    for msg, level, dur, title in demos:
        ok = send_notification(ue, msg, level, dur, title)
        status = "OK" if ok else "FAILED"
        print(f"  [{level.upper():7s}] {title}: {msg} [{status}]")
        time.sleep(0.5)

    print("\nDemo complete! Notifications will auto-dismiss.")


def main():
    parser = argparse.ArgumentParser(
        description="Send notifications to RAMMS UI via Unreal Remote Control")
    parser.add_argument("message", nargs="?", default=None,
                        help="Notification message text")
    parser.add_argument("--level", "-l", default="info",
                        choices=["info", "success", "warning", "error"],
                        help="Notification level (default: info)")
    parser.add_argument("--duration", "-d", type=float, default=4.0,
                        help="Auto-dismiss time in seconds (0=manual, default=4)")
    parser.add_argument("--title", "-t", default="",
                        help="Optional title line")
    parser.add_argument("--host", default="127.0.0.1",
                        help="UE Remote Control host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=30010,
                        help="UE Remote Control port (default: 30010)")
    parser.add_argument("--dismiss-all", action="store_true",
                        help="Dismiss all active notifications")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="Interactive REPL mode")
    parser.add_argument("--demo", action="store_true",
                        help="Send demo notifications")

    args = parser.parse_args()

    # Connect
    print(f"Connecting to UE Remote Control at http://{args.host}:{args.port}...")
    ue = UnrealRemote(host=args.host, http_port=args.port,
                      ui_bridge="/Script/RammsUI.Default__RammsRemoteBridge")
    try:
        ue.ping()
        print("Connected!\n")
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

    # Dispatch
    if args.dismiss_all:
        count = dismiss_all(ue)
        print(f"Dismissed {count} notification(s)")

    elif args.demo:
        demo_mode(ue)

    elif args.interactive:
        interactive_mode(ue)

    elif args.message:
        ok = send_notification(ue, args.message, args.level, args.duration, args.title)
        if ok:
            print(f"Notification sent: [{args.level.upper()}] {args.message}")
        else:
            print("Failed to send notification")
            sys.exit(1)

    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
