"""Joint control widget — slider + input for a single arm joint."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Button, Input, Label, Static


class JointControl(Widget):
    """Control widget for a single Kinova arm joint.

    Provides a label showing the current angle, a slider-like +/- control,
    an input field for precise entry, and a Send button.
    """

    DEFAULT_CSS = """
    JointControl {
        layout: horizontal;
        height: 3;
        padding: 0 1;
        margin: 0 0 0 0;
    }
    JointControl .jc-label {
        width: 10;
        padding: 1 0;
        text-align: right;
        text-style: bold;
    }
    JointControl .jc-current {
        width: 10;
        padding: 1 1;
        text-align: right;
        color: $accent;
    }
    JointControl .jc-nudge {
        width: 5;
        min-width: 5;
    }
    JointControl .jc-input {
        width: 14;
    }
    JointControl .jc-send {
        width: 8;
        min-width: 8;
    }
    JointControl .jc-unit {
        width: 3;
        padding: 1 0;
    }
    """

    class TargetChanged(Message):
        """Posted when the user sets a new joint target."""

        def __init__(self, joint_index: int, angle: float) -> None:
            super().__init__()
            self.joint_index = joint_index
            self.angle = angle

    def __init__(
        self,
        joint_index: int,
        name: str = "",
        min_angle: float = -180.0,
        max_angle: float = 180.0,
        step: float = 5.0,
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        self.joint_index = joint_index
        self.joint_name = name or f"Joint {joint_index}"
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.step = step
        self._current_angle: float = 0.0

    def compose(self) -> ComposeResult:
        yield Label(self.joint_name, classes="jc-label")
        yield Static("  0.00°", classes="jc-current", id=f"jc-val-{self.joint_index}")
        yield Button("−", id=f"jc-dec-{self.joint_index}", classes="jc-nudge")
        yield Input(
            value="0.0",
            restrict=r"[\-\d.]+",
            id=f"jc-inp-{self.joint_index}",
            classes="jc-input",
        )
        yield Button("+", id=f"jc-inc-{self.joint_index}", classes="jc-nudge")
        yield Button("Set", id=f"jc-set-{self.joint_index}", classes="jc-send",
                      variant="primary")
        yield Static("°", classes="jc-unit")

    def update_angle(self, angle: float) -> None:
        """Update the displayed current angle from polling."""
        self._current_angle = angle
        val = self.query_one(f"#jc-val-{self.joint_index}", Static)
        val.update(f"{angle:7.2f}°")

    def _send_target(self, value: float) -> None:
        value = max(self.min_angle, min(self.max_angle, value))
        inp = self.query_one(f"#jc-inp-{self.joint_index}", Input)
        inp.value = f"{value:.1f}"
        self.post_message(self.TargetChanged(self.joint_index, value))

    def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id or ""
        if bid.startswith("jc-dec-"):
            self._send_target(self._current_angle - self.step)
        elif bid.startswith("jc-inc-"):
            self._send_target(self._current_angle + self.step)
        elif bid.startswith("jc-set-"):
            inp = self.query_one(f"#jc-inp-{self.joint_index}", Input)
            try:
                self._send_target(float(inp.value))
            except ValueError:
                pass

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id == f"jc-inp-{self.joint_index}":
            try:
                self._send_target(float(event.value))
            except ValueError:
                pass
