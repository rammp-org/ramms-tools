#!/usr/bin/env python3
"""RAMMS TUI — Terminal UI for monitoring and controlling RAMMS robotics.

Launch the interactive TUI dashboard for the Mebot, Kinova arm, and
IMU data streaming.

Usage:
    ramms-tui                         # Connect to localhost:30010
    ramms-tui --host 192.168.1.10     # Connect to remote host
    ramms-tui --port 30020            # Use non-default port
"""

import argparse
import logging
import sys


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RAMMS TUI — Terminal dashboard for robotics control")
    parser.add_argument("--host", default="127.0.0.1",
                        help="UE Remote Control API host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=30010,
                        help="UE Remote Control API port (default: 30010)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    try:
        import textual  # noqa: F401
    except ImportError:
        print("Error: The 'textual' package is required for the TUI.")
        print("Install it with:  pip install 'ramms-tools[tui]'")
        print("  or:             pip install textual")
        sys.exit(1)

    from ramms_tools.tui.app import RammsTUI

    app = RammsTUI(host=args.host, port=args.port)
    app.run()


if __name__ == "__main__":
    main()
