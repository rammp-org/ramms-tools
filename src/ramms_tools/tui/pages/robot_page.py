"""Robot page: any robot's control surface -- drive mode, curb crossing, enum and action controls, live
readback -- and its sensors' latest readings."""

from __future__ import annotations

import math
import re
from typing import TYPE_CHECKING, Any, Optional

from textual import work
from textual.app import ComposeResult
from textual.containers import Container, Horizontal, Vertical, VerticalScroll
from textual.widgets import Button, DataTable, Label, Select, Static, Switch

from ramms_tools.control import ControlAxis, StatusText

if TYPE_CHECKING:
    from ramms_tools.robot import Robot
    from ramms_tools.tui.app import RammsTUI

# Controls the curb panel and the mode selector already show.
_DEDICATED = {"drive.mode", "curb.assist", "curb.style", "curb.negotiate", "curb.abort", "curb.phase", "curb.status"}
_GENERATIONS = {"gen1": "RAMMP Gen1", "gen2": "RAMMP Gen2 (lift-drive)"}


def _wid(prefix: str, control_id: str) -> str:
    return f"{prefix}-" + re.sub(r"[^A-Za-z0-9_-]", "_", control_id)


def _index(value: Any) -> Optional[int]:
    """An Enum control's value as an index; None for an unreadable one (NaN)."""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return int(round(f)) if math.isfinite(f) else None


def _show(axis: ControlAxis, value: Any) -> str:
    if isinstance(value, StatusText):
        return f"[{value.severity}] {value.text}"
    if isinstance(value, float):
        unit = "" if axis.units in ("None", "Normalized") else f" {axis.units.lower()}"
        return f"{value:.2f}{unit}"
    return str(value)


class RobotPage(Container):
    """Drive mode, curb crossing, enum selectors and action buttons per group, readback, sensors."""

    DEFAULT_CSS = """
    RobotPage {
        layout: vertical;
        padding: 0 1;
    }
    RobotPage .robot-header {
        text-style: bold;
        color: $accent;
        height: 1;
    }
    RobotPage #robot-top {
        height: 3;
    }
    RobotPage #robot-top Select {
        width: 40;
        margin: 0 1 0 0;
    }
    RobotPage #robot-mode-status {
        width: 1fr;
        padding: 1 1 0 1;
        color: $text-muted;
    }
    RobotPage #robot-body {
        height: 1fr;
    }
    RobotPage #robot-left {
        width: 1fr;
    }
    RobotPage #robot-right {
        width: 1fr;
    }
    RobotPage .robot-section {
        text-style: bold;
        color: $secondary;
        padding: 1 0 0 0;
    }
    RobotPage #curb-panel {
        height: auto;
        border: round $primary;
        padding: 0 1;
    }
    RobotPage .curb-row {
        height: 3;
    }
    RobotPage .curb-row Button {
        margin: 0 1 0 0;
    }
    RobotPage .curb-row Label {
        padding: 1 1 0 0;
    }
    RobotPage .sev-info {
        color: $text;
    }
    RobotPage .sev-attention {
        color: $warning;
        text-style: bold;
    }
    RobotPage .sev-fault {
        color: $error;
        text-style: bold;
    }
    RobotPage .ctl-row {
        height: 3;
    }
    RobotPage .ctl-row Label {
        width: 28;
        padding: 1 1 0 0;
    }
    RobotPage .ctl-row Select {
        width: 30;
    }
    RobotPage .ctl-buttons {
        height: auto;
        layout: grid;
        grid-size: 3;
        grid-gutter: 0 1;
    }
    RobotPage .ctl-buttons Button {
        width: 100%;
    }
    RobotPage DataTable {
        height: auto;
        max-height: 24;
    }
    RobotPage #robot-status-bar {
        height: 1;
        dock: bottom;
        color: $text-muted;
    }
    """

    _active: bool = False

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.robots: list[Robot] = []
        self.robot: Optional[Robot] = None
        self._known: dict[str, Any] = {}           # widget id -> value last shown (to tell user changes from ours)
        self._buttons: dict[str, str] = {}         # button id -> control id
        self._selects: dict[str, str] = {}         # select id -> control id
        self._readback: list[ControlAxis] = []
        self._enums: list[ControlAxis] = []
        self._sensor_rows: set[str] = set()
        self._readback_rows: set[str] = set()
        self._tick = 0
        self._sensing_ok = True
        # One poll at a time: a thread worker cannot be cancelled, so starting one per interval while the engine is
        # slow would pile pollers up (and an old one could overwrite newer values). A poll is skipped while one runs,
        # and a poll of a robot no longer selected is dropped (_robot_gen).
        self._polling = False
        self._robot_gen = 0
        self.polls_skipped = 0

    # ── layout ──────────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Static("Robot", classes="robot-header", id="robot-title")
        with Horizontal(id="robot-top"):
            yield Select([], prompt="Robot", id="robot-select", allow_blank=True)
            yield Select([], prompt="Drive mode", id="robot-mode", allow_blank=True)
            yield Button("Refresh", id="robot-refresh")
            yield Static("", id="robot-mode-status")
        with Horizontal(id="robot-body"):
            with VerticalScroll(id="robot-left"):
                with Vertical(id="curb-panel"):
                    yield Static("Curb crossing", classes="robot-section")
                    yield Static("-", id="curb-phase")
                    yield Static("", id="curb-status", classes="sev-info")
                    with Horizontal(classes="curb-row"):
                        yield Label("Assist")
                        yield Switch(value=False, id="curb-assist")
                        yield Select([], prompt="Style", id="curb-style", allow_blank=True)
                    with Horizontal(classes="curb-row"):
                        yield Button("Cross the curb ahead", id="curb-negotiate", variant="primary")
                        yield Button("Stop crossing", id="curb-abort", variant="error")
                        yield Button("Detect", id="curb-detect")
                    yield Static("", id="curb-detection")
                yield Vertical(id="robot-controls")
            with VerticalScroll(id="robot-right"):
                yield Static("Readback", classes="robot-section")
                yield DataTable(id="robot-readback", show_cursor=False)
                yield Static("Sensors", classes="robot-section")
                yield DataTable(id="robot-sensors", show_cursor=False)
                yield Static("", id="robot-sensor-status")
        yield Static("", id="robot-status-bar")

    def on_mount(self) -> None:
        rb = self.query_one("#robot-readback", DataTable)
        rb.add_column("Control", key="id")
        rb.add_column("Value", key="value")
        se = self.query_one("#robot-sensors", DataTable)
        se.add_column("Sensor", key="name")
        se.add_column("Kind", key="kind")
        se.add_column("Reading", key="reading")
        self.set_interval(0.5, self._poll)

    # ── activation ──────────────────────────────────────────────────

    def activate(self) -> None:
        self._active = True
        if not self.robots:
            self._discover()

    def deactivate(self) -> None:
        self._active = False

    def refresh_page(self) -> None:
        self._discover()

    def _set_status(self, text: str) -> None:
        try:
            self.query_one("#robot-status-bar", Static).update(text)
        except Exception:
            pass

    # ── discovery and building ──────────────────────────────────────

    @work(exclusive=True, thread=True, group="robot-discover")
    def _discover(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.ue or not app.connected:
            return
        from ramms_tools.robot import Robot
        try:
            robots = Robot.all(ue=app.ue)
        except Exception as exc:
            self.app.call_from_thread(self._set_status, f"  robots: {exc}")
            return

        def update() -> None:
            self.robots = robots
            sel = self.query_one("#robot-select", Select)
            sel.set_options([(r.name, i) for i, r in enumerate(robots)])
            if robots:
                self._known["robot-select"] = None
                sel.value = 0
            else:
                self._set_status("  no robot with a control surface in the world")
        self.app.call_from_thread(update)

    def _select_robot(self, index: int) -> None:
        if 0 <= index < len(self.robots):
            self.robot = self.robots[index]
            self._robot_gen += 1
            self._known["robot-select"] = index
            self._load_robot(self.robot)

    @work(exclusive=True, thread=True, group="robot-load")
    def _load_robot(self, robot: "Robot") -> None:
        try:
            desc = robot.surface.describe(refresh=True)
            modes = robot.modes() if robot.drive_modes.available else []
            curb = robot.curb
            style_labels = robot.surface.axis("curb.style").enum_labels if robot.surface.has("curb.style") else []
        except Exception as exc:
            self.app.call_from_thread(self._set_status, f"  {robot.name}: {exc}")
            return
        sensors: list = []
        try:
            sensors = robot.sensors.list()
            self._sensing_ok = True
        except Exception:
            self._sensing_ok = False
        self.app.call_from_thread(self._build, robot, desc, modes, curb, style_labels, sensors)

    def _build(self, robot: "Robot", desc, modes, curb, style_labels, sensors) -> None:
        gen = _GENERATIONS.get(robot.generation, "")
        self.query_one("#robot-title", Static).update(f"Robot  {robot.name}" + (f"  ({gen})" if gen else ""))
        mode_sel = self.query_one("#robot-mode", Select)
        mode_sel.set_options([(f"{m.label} ({m.id})", m.id) for m in modes])
        active = next((m.id for m in modes if m.active), None)
        if active is not None:
            self._known["robot-mode"] = active
            mode_sel.value = active
        mode_sel.disabled = not modes

        panel = self.query_one("#curb-panel")
        panel.display = curb is not None
        style = self.query_one("#curb-style", Select)
        style.set_options([(lab, i) for i, lab in enumerate(style_labels)])
        # Rebuilt (another robot, a mode change, Refresh): the selectors are blank now, so the next poll must fill
        # them whatever they showed before.
        self._known.pop("curb-style", None)
        for sid in list(self._known):
            if sid.startswith("sel-"):
                self._known.pop(sid, None)
        style.display = bool(style_labels)
        self.query_one("#curb-detect", Button).display = robot.generation == "gen2"

        box = self.query_one("#robot-controls", Vertical)
        box.remove_children()
        self._buttons.clear()
        self._selects.clear()
        widgets = []
        for group in desc.groups:
            axes = [a for a in desc.in_group(group) if a.id not in _DEDICATED]
            enums = [a for a in axes if a.is_enum and not a.read_only]
            actions = [a for a in axes if a.is_action]
            if not enums and not actions:
                continue
            widgets.append(Static(group, classes="robot-section"))
            for a in enums:
                sid = _wid("sel", a.id)
                self._selects[sid] = a.id
                widgets.append(Horizontal(Label(a.label), Select([(lab, i) for i, lab in enumerate(a.enum_labels)],
                                                                 prompt=a.label, id=sid, allow_blank=True),
                                          classes="ctl-row"))
            if actions:
                buttons = []
                for a in actions:
                    bid = _wid("btn", a.id)
                    self._buttons[bid] = a.id
                    buttons.append(Button(a.label, id=bid))
                widgets.append(Vertical(*buttons, classes="ctl-buttons"))
        if widgets:
            box.mount(*widgets)
        self._enums = [a for a in desc.axes if a.is_enum and not a.read_only]
        self._readback = [a for a in desc.axes if (a.kind in ("Position", "Text") or a.read_only) and a.readback
                          and a.id not in ("curb.status", "curb.phase")]
        table = self.query_one("#robot-readback", DataTable)
        table.clear()
        self._readback_rows.clear()
        sens = self.query_one("#robot-sensors", DataTable)
        sens.clear()
        self._sensor_rows.clear()
        for s in sensors:
            reading = ", ".join(f"{st.role} ch {st.channel}" for st in s.streams) if s.kind == "Camera" else ""
            sens.add_row(s.name, s.kind, reading, key=s.name)
            self._sensor_rows.add(s.name)
        self.query_one("#robot-sensor-status", Static).update(
            "" if self._sensing_ok else "  sensing bridge not available (RammsSensingBridge)")
        self._set_status(f"  {robot.name}: {len(desc.axes)} controls in {len(desc.groups)} groups")
        self._tick = 0

    # ── polling ─────────────────────────────────────────────────────

    def _poll(self) -> None:
        if not self._active or self.robot is None:
            return
        if self._polling:
            self.polls_skipped += 1
            return
        self._polling = True
        self._read_state(self._robot_gen)

    @work(thread=True, group="robot-poll")
    def _read_state(self, gen: int) -> None:
        try:
            self._read_state_now(gen)
        finally:
            self._polling = False

    def _read_state_now(self, gen: int) -> None:
        robot = self.robot
        app: RammsTUI = self.app  # type: ignore[assignment]
        if robot is None or not app.connected:
            return
        self._tick += 1
        cs = robot.surface
        state: dict[str, Any] = {"gen": gen}
        try:
            curb = robot.curb
            if curb is not None:
                ph = curb.phase()
                state["phase"] = curb.phase_label(ph) + (f" ({ph})" if curb.phase_label(ph) != ph else "")
                state["status"] = curb.status_line()
                state["assist"] = curb.assist
                if cs.has("curb.style"):
                    state["style"] = _index(cs.get("curb.style"))
            state["readback"] = [(a, cs.read(a.id)) for a in self._readback]
            if self._tick % 3 == 1:
                state["enums"] = {a.id: _index(cs.get(a.id)) for a in self._enums}
                if robot.drive_modes.available:
                    state["mode"] = robot.mode
                    state["switch"] = robot.drive_modes.switch_status()
            if self._sensing_ok and self._tick % 2 == 0:
                state["sensors"] = robot.sensors.read_all()
        except Exception as exc:
            self.app.call_from_thread(self._set_status, f"  poll: {exc}")
            return
        self.app.call_from_thread(self._show_state, state)

    def _show_state(self, state: dict) -> None:
        if state.get("gen", self._robot_gen) != self._robot_gen:
            return                       # a poll of the robot shown before
        if "phase" in state:
            self.query_one("#curb-phase", Static).update(f"Phase  {state['phase']}")
            st: StatusText = state["status"]
            w = self.query_one("#curb-status", Static)
            w.update(st.text or "-")
            for sev in ("info", "attention", "fault"):
                w.set_class(st.severity.lower() == sev, f"sev-{sev}")
            self._set_quiet("curb-assist", bool(state["assist"]))
            if "style" in state:
                self._set_quiet("curb-style", state["style"])
        table = self.query_one("#robot-readback", DataTable)
        for a, v in state.get("readback", []):
            text = _show(a, v)
            if a.id in self._readback_rows:
                table.update_cell(a.id, "value", text)
            else:
                table.add_row(a.id, text, key=a.id)
                self._readback_rows.add(a.id)
        for cid, v in state.get("enums", {}).items():
            self._set_quiet(_wid("sel", cid), v)
        if "mode" in state:
            self._set_quiet("robot-mode", state["mode"])
            self.query_one("#robot-mode-status", Static).update(state.get("switch", ""))
        if "sensors" in state:
            sens = self.query_one("#robot-sensors", DataTable)
            for r in state["sensors"]:
                if r.name in self._sensor_rows:
                    sens.update_cell(r.name, "reading", r.summary())
                else:
                    sens.add_row(r.name, r.kind, r.summary(), key=r.name)
                    self._sensor_rows.add(r.name)

    def _set_quiet(self, widget_id: str, value: Any) -> None:
        """Show a value read from the robot without sending it back as a command (None: unreadable, left as is)."""
        if value is None:
            return
        try:
            w = self.query_one(f"#{widget_id}")
        except Exception:
            return
        if self._known.get(widget_id) == value:
            return
        self._known[widget_id] = value
        try:
            w.value = value  # type: ignore[attr-defined]
        except Exception:
            pass

    # ── user input ──────────────────────────────────────────────────

    def on_select_changed(self, event: Select.Changed) -> None:
        sid = event.select.id or ""
        value = event.value
        if not isinstance(value, (int, str)) or isinstance(value, bool) or self._known.get(sid) == value:
            return
        self._known[sid] = value
        if sid == "robot-select":
            self._select_robot(int(value))
        elif sid == "robot-mode":
            self._command("mode", value)
        elif sid == "curb-style":
            self._command("enum", ("curb.style", value))
        elif sid in self._selects:
            self._command("enum", (self._selects[sid], value))

    def on_switch_changed(self, event: Switch.Changed) -> None:
        if event.switch.id == "curb-assist" and self._known.get("curb-assist") != event.value:
            self._known["curb-assist"] = event.value
            self._command("assist", event.value)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id or ""
        if bid == "robot-refresh":
            self.robots = []
            self._discover()
        elif bid == "curb-negotiate":
            self._command("negotiate", None)
        elif bid == "curb-abort":
            self._command("abort", None)
        elif bid == "curb-detect":
            self._command("detect", None)
        elif bid in self._buttons:
            self._command("trigger", self._buttons[bid])

    @work(thread=True, group="robot-command")
    def _command(self, what: str, arg: Any) -> None:
        robot = self.robot
        if robot is None:
            return
        msg = ""
        try:
            if what == "mode":
                msg = f"  drive mode -> {arg}: " + (robot.set_mode(arg, wait=True, timeout=15.0) or "done")
                self.app.call_from_thread(self._load_robot, robot)      # the controls depend on the mode
            elif what == "enum":
                cid, index = arg
                robot.surface.set_enum(cid, index)
                msg = f"  {cid} -> {robot.surface.axis(cid).enum_label(index)}"
            elif what == "assist":
                robot.curb.set_assist(bool(arg))
                msg = f"  curb assist {'on' if arg else 'off'}"
            elif what == "negotiate":
                if robot.surface.has("curb.negotiate"):
                    robot.surface.trigger("curb.negotiate")
                    msg = "  cross the curb ahead"
                else:
                    ok, reason = robot.curb.start()
                    msg = "  crossing started" if ok else f"  not started: {reason}"
            elif what == "abort":
                if robot.surface.has("curb.abort"):
                    robot.surface.trigger("curb.abort")
                else:
                    robot.curb.abort()
                msg = "  stop crossing"
            elif what == "detect":
                d = robot.curb.detect()
                text = d.get("detection_text") or d.get("reason") or d.get("note", "")
                self.app.call_from_thread(self.query_one("#curb-detection", Static).update, text)
                msg = "  detected" if d.get("step_found") else "  no step to cross"
            elif what == "trigger":
                ok = robot.surface.trigger(arg)
                msg = f"  {arg}" + ("" if ok else ": refused")
        except Exception as exc:
            msg = f"  {what}: {exc}"
        self.app.call_from_thread(self._set_status, msg)
