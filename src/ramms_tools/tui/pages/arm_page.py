"""Arm control page — Kinova Gen3 joint controls + gripper."""

from __future__ import annotations

from typing import TYPE_CHECKING

from textual.app import ComposeResult
from textual.containers import Container, Horizontal, Vertical, VerticalScroll
from textual import work
from textual.widgets import Button, Static

from ramms_tools.tui.widgets import GripperControl, JointControl

if TYPE_CHECKING:
    from ramms_tools.tui.app import RammsTUI


class ArmPage(Container):
    """Kinova Gen3 arm control page with 7 joint controls and gripper."""

    DEFAULT_CSS = """
    ArmPage {
        layout: vertical;
        padding: 1;
    }
    ArmPage .arm-header {
        text-style: bold;
        color: $accent;
        padding: 0 1 1 1;
    }
    ArmPage .arm-actions {
        height: 3;
        padding: 0 1;
        dock: bottom;
    }
    ArmPage .arm-actions Button {
        margin: 0 1 0 0;
    }
    ArmPage .arm-status {
        height: 1;
        padding: 0 1;
        color: $text-muted;
        dock: bottom;
    }
    ArmPage .arm-scroll {
        height: 1fr;
    }
    ArmPage .arm-not-found {
        padding: 2;
        text-align: center;
        color: $error;
    }
    ArmPage .arm-separator {
        height: 1;
        margin: 1 0 0 0;
    }
    """

    _active: bool = False
    NUM_JOINTS = 7

    def compose(self) -> ComposeResult:
        yield Static("🦾 Kinova Gen3 Arm Control", classes="arm-header")
        with VerticalScroll(classes="arm-scroll"):
            for i in range(self.NUM_JOINTS):
                yield JointControl(joint_index=i, name=f"Joint {i}",
                                   id=f"arm-joint-{i}")
            yield Static("", classes="arm-separator")
            yield GripperControl(id="arm-gripper")
        with Horizontal(classes="arm-actions"):
            yield Button("🏠 Home All", id="arm-home", variant="warning")
            yield Button("🔄 Refresh", id="arm-refresh")
            yield Button("📋 Copy Angles", id="arm-copy")
        yield Static("", classes="arm-status", id="arm-status-bar")

    def on_mount(self) -> None:
        self._poll_timer = self.set_interval(1.0, self._poll)

    def _poll(self) -> None:
        if not self._active:
            return
        self._refresh_angles()

    @work(exclusive=True, thread=True)
    def _refresh_angles(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.ue or not app.connected or not app.arm_comp:
            self.app.call_from_thread(self._show_not_found)
            return
        try:
            result = app.arm_comp.call("GetAllJointAngles")
            if isinstance(result, list):
                angles = result
            elif isinstance(result, dict):
                angles = result.get("ReturnValue", [])
            else:
                angles = []
        except Exception:
            angles = []

        # Read gripper state
        gripper_state = "Not found"
        gripper_f1 = 0.0
        gripper_f2 = 0.0
        has_gripper = False
        if app.gripper_comp:
            has_gripper = True
            try:
                state_raw = app.gripper_comp.call("GetGripperState")
                if isinstance(state_raw, str):
                    gripper_state = (state_raw.split("::")[-1]
                                     if "::" in state_raw else state_raw)
                else:
                    gripper_state = str(state_raw) if state_raw else "Unknown"
            except Exception:
                gripper_state = "Error"
            try:
                finger_result = app.gripper_comp.call("GetFingerAngles")
                if isinstance(finger_result, dict):
                    gripper_f1 = float(finger_result.get(
                        "OutFinger1Angle",
                        finger_result.get("Finger1Angle", 0.0)))
                    gripper_f2 = float(finger_result.get(
                        "OutFinger2Angle",
                        finger_result.get("Finger2Angle", 0.0)))
            except Exception:
                pass

        def _update():
            for i, angle in enumerate(angles):
                try:
                    jc = self.query_one(f"#arm-joint-{i}", JointControl)
                    jc.update_angle(float(angle))
                except Exception:
                    pass
            # Update gripper
            try:
                gc = self.query_one("#arm-gripper", GripperControl)
                if has_gripper:
                    gc.update_state(gripper_state, gripper_f1, gripper_f2)
                else:
                    gc.update_state("Not found", 0.0, 0.0)
            except Exception:
                pass
            try:
                parts = [f"{len(angles)} joints"]
                if has_gripper:
                    parts.append(f"gripper: {gripper_state}")
                self.query_one("#arm-status-bar", Static).update(
                    f"  {' | '.join(parts)}  |  Last update: OK")
            except Exception:
                pass

        self.app.call_from_thread(_update)

    def _show_not_found(self) -> None:
        try:
            self.query_one("#arm-status-bar", Static).update(
                "  ⚠ Kinova arm not found — check connection")
        except Exception:
            pass

    def on_joint_control_target_changed(
        self, event: JointControl.TargetChanged
    ) -> None:
        self._send_joint_target(event.joint_index, event.angle)

    def on_gripper_control_gripper_action(
        self, event: GripperControl.GripperAction
    ) -> None:
        self._send_gripper_action(event.action, event.data)

    @work(thread=True)
    def _send_joint_target(self, index: int, angle: float) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.arm_comp:
            return
        try:
            app.arm_comp.call("SetJointTarget",
                              JointIndex=index, TargetAngle=angle)
        except Exception as exc:
            self.app.call_from_thread(
                self._set_status, f"  ✗ Error setting joint {index}: {exc}")

    @work(thread=True)
    def _send_gripper_action(self, action: str, data: dict) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.gripper_comp:
            self.app.call_from_thread(
                self._set_status, "  ✗ Gripper not found")
            return
        try:
            if action == "open":
                app.gripper_comp.call("Open")
            elif action == "close":
                app.gripper_comp.call("Close")
            elif action == "toggle":
                app.gripper_comp.call("Toggle")
            elif action == "set_fingers":
                app.gripper_comp.call(
                    "SetFingerAngles",
                    Finger1Angle=data.get("finger1", 0.0),
                    Finger2Angle=data.get("finger2", 0.0))
            elif action == "set_speed":
                app.gripper_comp.call(
                    "SetMotorSpeedMultiplier",
                    SpeedMultiplier=data.get("multiplier", 1.0))
        except Exception as exc:
            self.app.call_from_thread(
                self._set_status, f"  ✗ Gripper error: {exc}")

    def _set_status(self, text: str) -> None:
        try:
            self.query_one("#arm-status-bar", Static).update(text)
        except Exception:
            pass

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "arm-home":
            self._home_all()
        elif event.button.id == "arm-refresh":
            self._refresh_angles()
        elif event.button.id == "arm-copy":
            self._copy_angles()

    @work(thread=True)
    def _home_all(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.arm_comp:
            return
        for i in range(self.NUM_JOINTS):
            try:
                app.arm_comp.call("SetJointTarget",
                                  JointIndex=i, TargetAngle=0.0)
            except Exception:
                pass
        self.app.call_from_thread(
            self._set_status, f"  🏠 Homed {self.NUM_JOINTS} joints to 0°")
        self._refresh_angles()

    @work(thread=True)
    def _copy_angles(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.arm_comp:
            return
        try:
            result = app.arm_comp.call("GetAllJointAngles")
            if isinstance(result, list):
                text = " ".join(f"{a:.1f}" for a in result)
                self.app.call_from_thread(self._set_status,
                                          f"  📋 Angles: {text}")
        except Exception:
            pass

    def activate(self) -> None:
        self._active = True
        self._refresh_angles()

    def deactivate(self) -> None:
        self._active = False
