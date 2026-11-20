"""Run Python inside a running Unreal editor over Python Remote Execution.

Speaks the PythonScriptPlugin remote-execution protocol (Project Settings > Python > Remote
Execution; RAMMP UI and RAMMS use multicast 239.0.0.1:6766 bound to loopback): a UDP ping/pong
finds the editor, then the editor connects back over TCP and runs each command sent on it.

This is the editor's own Python, not Remote Control: it reaches anything the editor's Python can
-- the play-in-editor world, components with no Remote Control exposure -- and is unavailable in
packaged builds. Use it for checks Remote Control's allow-list (deliberately narrow) does not
cover, never for anything a running robot UI depends on.

Ported from RAMMS's Scripts/editor_remote_exec.py, including its macOS fix: discovery is sent
unicast to 127.0.0.1:6766, where the editor's socket reads, because a group-addressed ping goes
out of whichever NIC owns the 239/8 route and never reaches a loopback-bound editor.
"""

from __future__ import annotations

import json
import socket
import struct
import sys
import time
import uuid

MULTICAST_GROUP = ("239.0.0.1", 6766)
BIND_ADDR = "127.0.0.1"
EDITOR_ENDPOINT = (BIND_ADDR, MULTICAST_GROUP[1])
MAGIC = "ue_py"
PROTO_VERSION = 1


class EditorExecError(Exception):
    """No editor answered, the connection failed, or the command raised in the editor."""


def make_msg(msg_type, source, dest=None, data=None):
    msg = {"version": PROTO_VERSION, "magic": MAGIC, "type": msg_type, "source": source}
    if dest is not None:
        msg["dest"] = dest
    if data is not None:
        msg["data"] = data
    return json.dumps(msg).encode("utf-8")


def parse_msg(payload):
    """The message in payload, or None while it is incomplete or not this protocol's."""
    try:
        msg = json.loads(payload.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None
    if isinstance(msg, dict) and msg.get("magic") == MAGIC and msg.get("version") == PROTO_VERSION:
        return msg
    return None


class EditorExec:
    """One command connection to one editor, used as a context manager.

        with EditorExec(project="RammpUI") as editor:
            lines = editor.run("print(1 + 1)")

    project: only an editor of this project is used (another project's editor answers the same
    discovery, and must not receive these commands); "*" takes any.
    """

    def __init__(self, project="RammpUI", discover_timeout=6.0, command_timeout=120.0):
        self.project = project
        self.discover_timeout = discover_timeout
        self.command_timeout = command_timeout
        self.node_id = str(uuid.uuid4())
        self.remote_id = None
        self.project_name = None
        self._mcast = None
        self._listener = None
        self._conn = None
        self._skipped = []

    def _send_to(self):
        # The group only when another project's editor answered and this is not macOS: a unicast to a
        # port several editors share reaches only one of them. (macOS cannot reach the group at all.)
        return MULTICAST_GROUP if self._skipped and sys.platform != "darwin" else EDITOR_ENDPOINT

    def open(self):
        mcast = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._mcast = mcast
        mcast.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            mcast.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        mcast.bind(("", MULTICAST_GROUP[1]))
        mreq = struct.pack("4s4s", socket.inet_aton(MULTICAST_GROUP[0]), socket.inet_aton(BIND_ADDR))
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(BIND_ADDR))
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        mcast.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 0)
        mcast.settimeout(0.5)

        deadline = time.monotonic() + self.discover_timeout
        while time.monotonic() < deadline and self.remote_id is None:
            mcast.sendto(make_msg("ping", self.node_id), self._send_to())
            try:
                payload, _ = mcast.recvfrom(4096)
            except socket.timeout:
                continue
            msg = parse_msg(payload)
            if msg and msg.get("type") == "pong" and msg.get("source") != self.node_id:
                name = msg.get("data", {}).get("project_name", "?")
                if self.project != "*" and name != self.project:
                    if name not in self._skipped:
                        self._skipped.append(name)
                    continue
                self.remote_id, self.project_name = msg["source"], name
        if self.remote_id is None:
            others = f" (other projects' editors answered: {', '.join(self._skipped)})" if self._skipped else ""
            self.close()
            raise EditorExecError(
                f"no {self.project if self.project != '*' else 'Unreal'} editor answered Python remote execution "
                f"on {MULTICAST_GROUP[0]}:{MULTICAST_GROUP[1]}{others} -- is it running, with Project Settings > "
                f"Python > Remote Execution enabled?")

        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener = listener
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((BIND_ADDR, 0))
        listener.listen(1)
        listener.settimeout(self.discover_timeout)
        ip, port = listener.getsockname()
        mcast.sendto(make_msg("open_connection", self.node_id, self.remote_id,
                              {"command_ip": ip, "command_port": port}), self._send_to())
        try:
            self._conn, _ = listener.accept()
        except socket.timeout:
            self.close()
            raise EditorExecError("the editor answered but did not open the command connection")
        self._conn.settimeout(self.command_timeout)
        return self

    def run(self, code):
        """Run a multi-line script in the editor; the lines it printed, in order.

        Raises EditorExecError if the script raised (with the editor's traceback) or no result came.
        """
        if self._conn is None:
            raise EditorExecError("not connected: use EditorExec as a context manager, or call open()")
        # ExecuteFile runs a literal multi-statement script as well as a path; ExecuteStatement takes one.
        self._conn.sendall(make_msg("command", self.node_id, self.remote_id,
                                    {"command": code, "unattended": True, "exec_mode": "ExecuteFile"}))
        buf, result = b"", None
        try:
            while result is None:
                chunk = self._conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                msg = parse_msg(buf)
                if msg and msg.get("type") == "command_result":
                    result = msg
        except socket.timeout:
            raise EditorExecError(f"no result from the editor within {self.command_timeout:g} s")
        if result is None:
            raise EditorExecError("the editor closed the connection without a result")
        data = result.get("data", {})
        lines = [entry.get("output", "").rstrip("\n") for entry in data.get("output", [])]
        if not data.get("success", False):
            # The exception itself comes back in `result`; `output` holds only what was printed.
            error = str(data.get("result") or "").strip()
            raise EditorExecError("the script failed in the editor:\n" + "\n".join(lines[-20:] + ([error] if error else [])))
        return lines

    def close(self):
        if self._mcast is not None and self.remote_id is not None:
            try:
                self._mcast.sendto(make_msg("close_connection", self.node_id, self.remote_id), self._send_to())
            except OSError:
                pass
        for sock in (self._conn, self._listener, self._mcast):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._conn = self._listener = self._mcast = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False
