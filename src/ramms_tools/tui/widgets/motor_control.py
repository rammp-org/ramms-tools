"""Motor control widget — for Mebot angular/linear motors."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Button, Input, Label, Static


class MotorControl(Widget):
    """Control widget for a single Mebot motor (angular or linear).

    Provides current value display, +/- nudge buttons, precise input, and Send.
    """

    DEFAULT_CSS = """
    MotorControl {
        layout: horizontal;
        height: 3;
        padding: 0 1;
    }
    MotorControl .mc-label {
        width: 18;
        padding: 1 0;
        text-align: right;
        text-style: bold;
    }
    MotorControl .mc-type {
        width: 5;
        padding: 1 0;
        color: $text-muted;
    }
    MotorControl .mc-current {
        width: 10;
        padding: 1 1;
        text-align: right;
        color: $accent;
    }
    MotorControl .mc-nudge {
        width: 5;
        min-width: 5;
    }
    MotorControl .mc-input {
        width: 14;
    }
    MotorControl .mc-send {
        width: 8;
        min-width: 8;
    }
    MotorControl .mc-unit {
        width: 5;
        padding: 1 0;
    }
    """

    class TargetChanged(Message):
        """Posted when the user sets a new motor target."""

        def __init__(self, motor_name: str, value: float,
                     motor_type: str = "angular") -> None:
            super().__init__()
            self.motor_name = motor_name
            self.value = value
            self.motor_type = motor_type

    def __init__(
        self,
        motor_name: str,
        motor_type: str = "angular",
        min_val: float = -180.0,
        max_val: float = 180.0,
        step: float = 5.0,
        current: float = 0.0,
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        self.motor_name = motor_name
        self.motor_type = motor_type  # "angular" or "linear"
        self.min_val = min_val
        self.max_val = max_val
        self.step = step
        self._current: float = current
        self._unit = "°" if motor_type == "angular" else "cm"
        self._widget_id = f"mc-{motor_name.replace(' ', '_')}"

    def compose(self) -> ComposeResult:
        tag = "ANG" if self.motor_type == "angular" else "LIN"
        yield Label(self.motor_name, classes="mc-label")
        yield Static(tag, classes="mc-type")
        yield Static(f"{self._current:7.2f}", classes="mc-current",
                      id=f"{self._widget_id}-val")
        yield Button("−", id=f"{self._widget_id}-dec", classes="mc-nudge")
        yield Input(
            value=f"{self._current:.1f}",
            restrict=r"[\-\d.]+",
            id=f"{self._widget_id}-inp",
            classes="mc-input",
        )
        yield Button("+", id=f"{self._widget_id}-inc", classes="mc-nudge")
        yield Button("Set", id=f"{self._widget_id}-set", classes="mc-send",
                      variant="primary")
        yield Static(self._unit, classes="mc-unit")

    def update_value(self, value: float) -> None:
        """Update displayed current value from polling."""
        self._current = value
        try:
            val = self.query_one(f"#{self._widget_id}-val", Static)
            val.update(f"{value:7.2f}")
        except Exception:
            pass

    def _send_target(self, value: float) -> None:
        value = max(self.min_val, min(self.max_val, value))
        try:
            inp = self.query_one(f"#{self._widget_id}-inp", Input)
            inp.value = f"{value:.1f}"
        except Exception:
            pass
        self.post_message(self.TargetChanged(self.motor_name, value,
                                              self.motor_type))

    def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id or ""
        if bid.endswith("-dec"):
            self._send_target(self._current - self.step)
        elif bid.endswith("-inc"):
            self._send_target(self._current + self.step)
        elif bid.endswith("-set"):
            try:
                inp = self.query_one(f"#{self._widget_id}-inp", Input)
                self._send_target(float(inp.value))
            except (ValueError, Exception):
                pass

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id == f"{self._widget_id}-inp":
            try:
                self._send_target(float(event.value))
            except ValueError:
                pass
