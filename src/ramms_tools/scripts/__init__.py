"""CLI scripts for ramms-tools.

Their output uses symbols such as °, → and ✓. A piped or redirected stdout on
Windows is cp1252, which cannot encode all of them, and printing one raised
after the command had already reached the simulator. Unencodable characters
are replaced instead.
"""
import sys

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass   # not a text stream we can reconfigure (e.g. replaced by a test)
