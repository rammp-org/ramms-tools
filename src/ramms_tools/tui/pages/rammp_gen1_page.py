"""RAMMP Gen1 motors control page."""

from __future__ import annotations

from typing import TYPE_CHECKING

from textual.app import ComposeResult
from textual.containers import Container, Horizontal, VerticalScroll
from textual import work
from textual.widgets import Button, Static

from ramms_tools.tui.widgets import MotorControl
from ramms_tools.tui.widgets.motor_control import _slugify

if TYPE_CHECKING:
    from ramms_tools.tui.app import RammsTUI


class RammpGen1Page(Container):
    """RAMMP Gen1 motor control page with dynamic angular/linear motor controls."""

    DEFAULT_CSS = """
    RammpGen1Page {
        layout: vertical;
        padding: 1;
    }
    RammpGen1Page .rammp-gen1-header {
        text-style: bold;
        color: $accent;
        padding: 0 1 1 1;
    }
    RammpGen1Page .rammp-gen1-section {
        text-style: bold;
        padding: 1 1 0 1;
        color: $secondary;
    }
    RammpGen1Page .rammp-gen1-scroll {
        height: 1fr;
    }
    RammpGen1Page .rammp-gen1-actions {
        height: 3;
        padding: 0 1;
        dock: bottom;
    }
    RammpGen1Page .rammp-gen1-actions Button {
        margin: 0 1 0 0;
    }
    RammpGen1Page .rammp-gen1-status {
        height: 1;
        padding: 0 1;
        color: $text-muted;
        dock: bottom;
    }
    RammpGen1Page .rammp-gen1-empty {
        padding: 2;
        text-align: center;
        color: $text-muted;
    }
    """

    _active: bool = False
    _motors_built: bool = False

    def compose(self) -> ComposeResult:
        yield Static("🤖 RAMMP Gen1 Motor Control", classes="rammp-gen1-header")
        yield VerticalScroll(id="rammp-gen1-scroll", classes="rammp-gen1-scroll")
        with Horizontal(classes="rammp-gen1-actions"):
            yield Button("🏠 Home All", id="rammp-gen1-home", variant="warning")
            yield Button("🔄 Refresh", id="rammp-gen1-refresh")
        yield Static("", classes="rammp-gen1-status", id="rammp-gen1-status-bar")

    def on_mount(self) -> None:
        self._poll_timer = self.set_interval(1.5, self._poll)

    def _poll(self) -> None:
        if not self._active:
            return
        self._refresh_motors()

    @work(exclusive=True, thread=True)
    def _refresh_motors(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.ue or not app.connected or not app.rammp_gen1_comp:
            return

        angular: list[dict] = []
        linear: list[dict] = []
        try:
            result = app.rammp_gen1_comp.call("GetAngularMotors")
            if isinstance(result, list):
                angular = result
        except Exception:
            pass
        try:
            result = app.rammp_gen1_comp.call("GetLinearMotors")
            if isinstance(result, list):
                linear = result
        except Exception:
            pass

        def _update():
            if not self._motors_built and (angular or linear):
                self._build_motor_widgets(angular, linear)
            elif self._motors_built:
                self._update_motor_values(angular, linear)

            total = len(angular) + len(linear)
            try:
                self.query_one("#rammp-gen1-status-bar", Static).update(
                    f"  {len(angular)} angular + {len(linear)} linear motors")
            except Exception:
                pass

        self.app.call_from_thread(_update)

    def _build_motor_widgets(self, angular: list[dict],
                              linear: list[dict]) -> None:
        """Dynamically build motor control widgets."""
        scroll = self.query_one("#rammp-gen1-scroll", VerticalScroll)
        scroll.remove_children()

        if angular:
            scroll.mount(Static(f"Angular Motors ({len(angular)})",
                                 classes="rammp-gen1-section"))
            for m in angular:
                name = m.get("ConstraintName", "?")
                target = m.get("TargetAngle", 0.0)
                slug = _slugify(name)
                widget = MotorControl(
                    motor_name=name,
                    motor_type="angular",
                    min_val=-180.0,
                    max_val=180.0,
                    current=float(target),
                    id=f"mc-ang-{slug}",
                )
                scroll.mount(widget)

        if linear:
            scroll.mount(Static(f"Linear Motors ({len(linear)})",
                                 classes="rammp-gen1-section"))
            for m in linear:
                name = m.get("ConstraintName", "?")
                target = m.get("TargetPosition", 0.0)
                slug = _slugify(name)
                widget = MotorControl(
                    motor_name=name,
                    motor_type="linear",
                    min_val=-200.0,
                    max_val=200.0,
                    current=float(target),
                    id=f"mc-lin-{slug}",
                )
                scroll.mount(widget)

        if not angular and not linear:
            scroll.mount(Static("No motors found", classes="rammp-gen1-empty"))

        self._motors_built = True

    def _update_motor_values(self, angular: list[dict],
                              linear: list[dict]) -> None:
        """Update existing motor widget values."""
        for m in angular:
            name = m.get("ConstraintName", "?")
            target = m.get("TargetAngle", 0.0)
            slug = _slugify(name)
            try:
                mc = self.query_one(f"#mc-ang-{slug}", MotorControl)
                mc.update_value(float(target))
            except Exception:
                pass

        for m in linear:
            name = m.get("ConstraintName", "?")
            target = m.get("TargetPosition", 0.0)
            slug = _slugify(name)
            try:
                mc = self.query_one(f"#mc-lin-{slug}", MotorControl)
                mc.update_value(float(target))
            except Exception:
                pass

    def on_motor_control_target_changed(
        self, event: MotorControl.TargetChanged
    ) -> None:
        self._send_motor_target(event.motor_name, event.value,
                                 event.motor_type)

    @work(thread=True)
    def _send_motor_target(self, name: str, value: float,
                            motor_type: str) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.rammp_gen1_comp:
            return
        try:
            if motor_type == "angular":
                app.rammp_gen1_comp.call("SetAngularMotorTarget",
                                     MotorName=name, TargetAngle=value)
            else:
                app.rammp_gen1_comp.call("SetLinearMotorTarget",
                                     MotorName=name, TargetPosition=value)
        except Exception as exc:
            self.app.call_from_thread(
                self._set_status, f"  ✗ Error: {exc}")

    def _set_status(self, text: str) -> None:
        try:
            self.query_one("#rammp-gen1-status-bar", Static).update(text)
        except Exception:
            pass

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "rammp-gen1-home":
            self._home_all()
        elif event.button.id == "rammp-gen1-refresh":
            self._motors_built = False
            self._refresh_motors()

    @work(thread=True)
    def _home_all(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.rammp_gen1_comp:
            return
        count = 0
        try:
            angular = app.rammp_gen1_comp.call("GetAngularMotors")
            if isinstance(angular, list):
                for m in angular:
                    name = m.get("ConstraintName", "")
                    if name:
                        app.rammp_gen1_comp.call("SetAngularMotorTarget",
                                             MotorName=name, TargetAngle=0.0)
                        count += 1
        except Exception:
            pass
        try:
            linear = app.rammp_gen1_comp.call("GetLinearMotors")
            if isinstance(linear, list):
                for m in linear:
                    name = m.get("ConstraintName", "")
                    if name:
                        app.rammp_gen1_comp.call("SetLinearMotorTarget",
                                             MotorName=name,
                                             TargetPosition=0.0)
                        count += 1
        except Exception:
            pass

        self.app.call_from_thread(
            self._set_status, f"  🏠 Homed {count} motors to 0")
        self._refresh_motors()

    def activate(self) -> None:
        self._active = True
        self._refresh_motors()

    def deactivate(self) -> None:
        self._active = False
