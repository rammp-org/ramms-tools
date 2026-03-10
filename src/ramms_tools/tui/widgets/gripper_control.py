"""Gripper control widget — state display and finger angle controls."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Button, Input, Label, Static


class GripperControl(Widget):
    """Control widget for a GripperControllerComponent.

    Shows current state (Open/Closed/Opening/Closing), finger angles,
    and provides Open/Close/Toggle buttons plus direct finger angle input.
    """

    DEFAULT_CSS = """
    GripperControl {
        layout: vertical;
        height: auto;
        padding: 0 1;
    }
    GripperControl .gc-row {
        layout: horizontal;
        height: 3;
    }
    GripperControl .gc-header {
        text-style: bold;
        color: $secondary;
        padding: 1 0 0 0;
    }
    GripperControl .gc-state-label {
        width: 8;
        padding: 1 0;
        text-style: bold;
    }
    GripperControl .gc-state-value {
        width: 14;
        padding: 1 0;
        color: $accent;
    }
    GripperControl .gc-btn {
        margin: 0 1 0 0;
    }
    GripperControl .gc-finger-label {
        width: 10;
        padding: 1 0;
        text-align: right;
        text-style: bold;
    }
    GripperControl .gc-finger-val {
        width: 10;
        padding: 1 1;
        text-align: right;
        color: $accent;
    }
    GripperControl .gc-finger-input {
        width: 12;
    }
    GripperControl .gc-unit {
        width: 3;
        padding: 1 0;
    }
    GripperControl .gc-speed-label {
        width: 8;
        padding: 1 0;
    }
    GripperControl .gc-speed-input {
        width: 10;
    }
    """

    class GripperAction(Message):
        """Posted when the user triggers a gripper action."""

        def __init__(self, action: str, **kwargs_data) -> None:
            super().__init__()
            self.action = action
            self.data = kwargs_data

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._state: str = "Unknown"
        self._finger1: float = 0.0
        self._finger2: float = 0.0

    def compose(self) -> ComposeResult:
        yield Static("🤏 Gripper", classes="gc-header")

        with Horizontal(classes="gc-row"):
            yield Label("State:", classes="gc-state-label")
            yield Static("Unknown", classes="gc-state-value",
                          id="gc-state-val")
            yield Button("Open", id="gc-open", classes="gc-btn",
                          variant="success")
            yield Button("Close", id="gc-close", classes="gc-btn",
                          variant="error")
            yield Button("Toggle", id="gc-toggle", classes="gc-btn")

        with Horizontal(classes="gc-row"):
            yield Label("Finger 1:", classes="gc-finger-label")
            yield Static("  0.00°", classes="gc-finger-val",
                          id="gc-f1-val")
            yield Input(value="0.0", restrict=r"[\-\d.]+",
                        id="gc-f1-inp", classes="gc-finger-input")
            yield Static("°", classes="gc-unit")
            yield Label("Finger 2:", classes="gc-finger-label")
            yield Static("  0.00°", classes="gc-finger-val",
                          id="gc-f2-val")
            yield Input(value="0.0", restrict=r"[\-\d.]+",
                        id="gc-f2-inp", classes="gc-finger-input")
            yield Static("°", classes="gc-unit")
            yield Button("Set", id="gc-set-fingers", classes="gc-btn",
                          variant="primary")

        with Horizontal(classes="gc-row"):
            yield Label("Speed:", classes="gc-speed-label")
            yield Input(value="1.0", restrict=r"[\d.]+",
                        id="gc-speed-inp", classes="gc-speed-input")
            yield Button("Set Speed", id="gc-set-speed", classes="gc-btn")

    def update_state(self, state: str, finger1: float, finger2: float) -> None:
        """Update displayed gripper state and finger angles from polling."""
        self._state = state
        self._finger1 = finger1
        self._finger2 = finger2
        try:
            self.query_one("#gc-state-val", Static).update(state)
            self.query_one("#gc-f1-val", Static).update(f"{finger1:7.2f}°")
            self.query_one("#gc-f2-val", Static).update(f"{finger2:7.2f}°")
        except Exception:
            pass

    def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id or ""
        if bid == "gc-open":
            self.post_message(self.GripperAction("open"))
        elif bid == "gc-close":
            self.post_message(self.GripperAction("close"))
        elif bid == "gc-toggle":
            self.post_message(self.GripperAction("toggle"))
        elif bid == "gc-set-fingers":
            try:
                f1 = float(self.query_one("#gc-f1-inp", Input).value)
                f2 = float(self.query_one("#gc-f2-inp", Input).value)
                self.post_message(self.GripperAction("set_fingers",
                                                      finger1=f1, finger2=f2))
            except (ValueError, Exception):
                pass
        elif bid == "gc-set-speed":
            try:
                spd = float(self.query_one("#gc-speed-inp", Input).value)
                self.post_message(self.GripperAction("set_speed",
                                                      multiplier=spd))
            except (ValueError, Exception):
                pass

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id in ("gc-f1-inp", "gc-f2-inp"):
            try:
                f1 = float(self.query_one("#gc-f1-inp", Input).value)
                f2 = float(self.query_one("#gc-f2-inp", Input).value)
                self.post_message(self.GripperAction("set_fingers",
                                                      finger1=f1, finger2=f2))
            except (ValueError, Exception):
                pass
        elif event.input.id == "gc-speed-inp":
            try:
                spd = float(event.value)
                self.post_message(self.GripperAction("set_speed",
                                                      multiplier=spd))
            except (ValueError, Exception):
                pass
