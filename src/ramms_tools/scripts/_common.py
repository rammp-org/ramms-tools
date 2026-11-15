"""Shared bits of the robot / sensing CLIs (ramms-control, ramms-curb, ramms-sensors)."""

from __future__ import annotations

import argparse
import contextlib
import json
import logging
import math
import signal
import sys
import threading
import time
from typing import Any, Callable, Iterator, Optional

from ramms_tools.unreal_remote import UnrealRemote

logger = logging.getLogger(__name__)

# The signals that end a CLI: Ctrl+C, `kill` / `timeout`, a closed terminal (and Ctrl+Break on Windows).
EXIT_SIGNALS = tuple(s for s in (getattr(signal, n, None) for n in ("SIGINT", "SIGTERM", "SIGHUP", "SIGBREAK")) if s is not None)


class Interrupted(BaseException):
    """Raised in the main thread by SIGINT / SIGTERM / SIGHUP while exit_signals() is active, so that every
    `finally` (the releases of held controls) runs. A BaseException, like KeyboardInterrupt, so `except Exception`
    does not swallow it."""

    def __init__(self, signum: int):
        self.signum = signum
        try:
            self.signame = signal.Signals(signum).name
        except ValueError:
            self.signame = str(signum)
        super().__init__(self.signame)


@contextlib.contextmanager
def exit_signals() -> Iterator[dict]:
    """
    While active, SIGINT / SIGTERM / SIGHUP raise Interrupted in the main thread (instead of killing the process
    without running `finally`, as SIGTERM / SIGHUP do by default). A second signal while the first is being handled
    (the robot's controls being released) is only reported; a third raises Interrupted again. Yields a dict whose
    "count" says how many signals arrived. Outside the main thread (tests) it does nothing.
    """
    state = {"count": 0}

    def handler(signum, frame):
        state["count"] += 1
        if state["count"] == 2:
            print("still releasing the robot's controls; signal once more to quit without waiting", file=sys.stderr,
                  flush=True)
            return
        raise Interrupted(signum)

    previous = {}
    if threading.current_thread() is threading.main_thread():
        for sig in EXIT_SIGNALS:
            try:
                previous[sig] = signal.signal(sig, handler)
            except (ValueError, OSError):
                pass
    try:
        yield state
    finally:
        for sig, prev in previous.items():
            try:
                signal.signal(sig, prev)
            except (ValueError, OSError):
                pass


def interrupted_message(e: Interrupted, what: str = "") -> str:
    return f"interrupted ({e.signame})" + (f": {what}" if what else "")


def add_common(parser: argparse.ArgumentParser, top: bool, robot: bool = True, json_flag: bool = True) -> None:
    """The connection flags, accepted before or after the subcommand: the subparsers' copies default to SUPPRESS so
    they do not clobber a value given at the top level."""
    d = (lambda v: v) if top else (lambda v: argparse.SUPPRESS)
    parser.add_argument("--host", default=d("127.0.0.1"), help="Remote Control host (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=d(30010), help="Remote Control HTTP port (default 30010)")
    parser.add_argument("--timeout", type=float, default=d(10.0), help="HTTP timeout, seconds")
    if robot:
        parser.add_argument("--robot", "-r", default=d(""), help="robot actor name / path or robotName (default: the first robot)")
    if json_flag:
        parser.add_argument("--json", action="store_true", default=d(False), help="machine-readable output")
    parser.add_argument("-v", "--verbose", action="store_true", default=d(False), help="log the HTTP traffic")


def finish_parser(parser: argparse.ArgumentParser, sub: argparse._SubParsersAction, robot: bool = True,
                  no_json: tuple[str, ...] = ()) -> None:
    for name, sp in sub.choices.items():
        add_common(sp, False, robot=robot, json_flag=name not in no_json)


def setup_logging(verbose: bool) -> None:
    logging.basicConfig(level=logging.DEBUG if verbose else logging.WARNING, format="%(levelname)s %(name)s: %(message)s")


def make_remote(host: str, port: int, timeout: float) -> UnrealRemote:
    """The Remote Control client the CLIs use (tests replace this)."""
    return UnrealRemote(host=host, http_port=port, timeout=timeout)


def connect(args: argparse.Namespace) -> Optional[UnrealRemote]:
    """A client for --host / --port, or None (after saying why) when nothing answers there."""
    ue = make_remote(args.host, args.port, args.timeout)
    if not ue.ping():
        print(f"error: no Remote Control API at {args.host}:{args.port} (is the editor / game running with it enabled?)",
              file=sys.stderr)
        return None
    return ue


def fmt_num(v: Any, digits: int = 2) -> str:
    if isinstance(v, bool) or v is None:
        return str(v)
    if isinstance(v, (int, float)):
        if isinstance(v, float) and math.isnan(v):
            return "nan"
        return f"{v:.{digits}f}" if isinstance(v, float) else str(v)
    return str(v)


def dump_json(obj: Any) -> str:
    def default(o):
        if hasattr(o, "_asdict"):
            return o._asdict()
        if hasattr(o, "__dict__"):
            return {k: v for k, v in o.__dict__.items() if k != "raw"}
        return str(o)
    return json.dumps(obj, default=default)


class Ticker:
    """Run a function at `rate` Hz for `duration` s (0 = until Ctrl+C, which ends the loop quietly)."""

    def __init__(self, rate: float, duration: float = 0.0):
        if rate <= 0:
            raise ValueError("--rate must be > 0")
        self.period = 1.0 / rate
        self.duration = duration
        self.stopped = False

    def run(self, tick: Callable[[float], None]) -> None:
        prev = None
        try:
            prev = signal.signal(signal.SIGINT, self._stop)
        except ValueError:      # not the main thread (tests)
            pass
        try:
            t0 = time.monotonic()
            n = 0
            while not self.stopped:
                now = time.monotonic() - t0
                if self.duration and now >= self.duration:
                    break
                tick(now)
                n += 1
                delay = t0 + n * self.period - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        finally:
            if prev is not None:
                signal.signal(signal.SIGINT, prev)

    def _stop(self, *_):
        self.stopped = True
