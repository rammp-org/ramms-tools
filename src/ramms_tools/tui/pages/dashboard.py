"""Dashboard page — overview of all RAMMS subsystems."""

from __future__ import annotations

from typing import TYPE_CHECKING

from textual.app import ComposeResult
from textual.containers import Container, Horizontal, Vertical
from textual.widget import Widget
from textual import work
from textual.widgets import Static

if TYPE_CHECKING:
    from ramms_tools.tui.app import RammsTUI


class SummaryCard(Widget):
    """A bordered card showing a subsystem summary."""

    DEFAULT_CSS = """
    SummaryCard {
        width: 1fr;
        height: auto;
        min-height: 5;
        margin: 1;
        padding: 1 2;
        border: round $primary;
    }
    SummaryCard .card-title {
        text-style: bold;
        color: $accent;
        margin: 0 0 1 0;
    }
    SummaryCard .card-status {
        color: $success;
    }
    SummaryCard .card-status.not-found {
        color: $error;
    }
    SummaryCard .card-detail {
        color: $text-muted;
    }
    """

    def __init__(self, title: str, card_id: str, **kwargs) -> None:
        super().__init__(id=card_id, **kwargs)
        self._title = title

    def compose(self) -> ComposeResult:
        yield Static(self._title, classes="card-title")
        yield Static("Searching...", classes="card-status", id=f"{self.id}-status")
        yield Static("", classes="card-detail", id=f"{self.id}-detail")


class DashboardPage(Container):
    """Overview dashboard showing status of all RAMMS subsystems."""

    DEFAULT_CSS = """
    DashboardPage {
        layout: vertical;
        padding: 1;
    }
    DashboardPage .dash-header {
        text-align: center;
        text-style: bold;
        width: 1fr;
        padding: 1;
        color: $accent;
    }
    DashboardPage .dash-row {
        height: auto;
    }
    """

    _active: bool = False

    def compose(self) -> ComposeResult:
        yield Static("RAMMS Control Dashboard", classes="dash-header")
        with Horizontal(classes="dash-row"):
            yield SummaryCard("🦾  Kinova Gen3 Arm", "card-arm")
            yield SummaryCard("🤖  Mebot Controller", "card-mebot")
        with Horizontal(classes="dash-row"):
            yield SummaryCard("📡  System Info", "card-system")
            yield SummaryCard("🧭  IMU", "card-imu")

    def on_mount(self) -> None:
        self._poll_timer = self.set_interval(2.0, self._poll)

    def _poll(self) -> None:
        if not self._active:
            return
        self._refresh_dashboard()

    @work(exclusive=True, thread=True)
    def _refresh_dashboard(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.ue or not app.connected:
            return

        # Arm summary
        arm_status = "Not found"
        arm_detail = ""
        arm_found = False
        if app.arm_comp:
            arm_found = True
            arm_status = "Connected"
            try:
                result = app.arm_comp.call("GetAllJointAngles")
                if isinstance(result, list):
                    parts = [f"J{i}:{a:6.1f}°" for i, a in enumerate(result)]
                    arm_detail = "  ".join(parts)
            except Exception:
                arm_detail = "(read error)"

        # Mebot summary
        mebot_status = "Not found"
        mebot_detail = ""
        mebot_found = False
        if app.mebot_comp:
            mebot_found = True
            mebot_status = "Connected"
            try:
                angular = app.mebot_comp.call("GetAngularMotors")
                linear = app.mebot_comp.call("GetLinearMotors")
                na = len(angular) if isinstance(angular, list) else 0
                nl = len(linear) if isinstance(linear, list) else 0
                mebot_detail = f"{na} angular + {nl} linear motors"
            except Exception:
                mebot_detail = "(read error)"

        # System info
        host = app.ue.base_url
        arm_path = app.arm_comp.object_path if app.arm_comp else "—"
        mebot_path = app.mebot_comp.object_path if app.mebot_comp else "—"

        def _update():
            try:
                s = self.query_one("#card-arm-status", Static)
                s.update(f"{'✓' if arm_found else '✗'} {arm_status}")
                s.set_class(not arm_found, "not-found")
                self.query_one("#card-arm-detail", Static).update(arm_detail)

                s = self.query_one("#card-mebot-status", Static)
                s.update(f"{'✓' if mebot_found else '✗'} {mebot_status}")
                s.set_class(not mebot_found, "not-found")
                self.query_one("#card-mebot-detail", Static).update(mebot_detail)

                self.query_one("#card-system-status", Static).update(
                    f"Host: {host}")
                self.query_one("#card-system-detail", Static).update(
                    f"Arm: {arm_path}\nMebot: {mebot_path}")

                self.query_one("#card-imu-status", Static).update(
                    "Available" if arm_found or mebot_found else "No targets")
            except Exception:
                pass

        self.app.call_from_thread(_update)

    def activate(self) -> None:
        self._active = True
        self._refresh_dashboard()

    def deactivate(self) -> None:
        self._active = False
