"""IMU streaming page — real-time position, orientation, velocity display."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from textual.app import ComposeResult
from textual.containers import Container, Horizontal, Vertical
from textual import work
from textual.widgets import Button, Input, Label, Static

from ramms_tools.tui.widgets import ValueDisplay
from ramms_tools.transforms import (
    LowPassFilter, angle_diff, apply_deadzone, cm_to_m_vec,
    quat_to_euler, world_to_local,
)

if TYPE_CHECKING:
    from ramms_tools.tui.app import RammsTUI


class IMUPage(Container):
    """IMU data streaming page with real-time display."""

    DEFAULT_CSS = """
    IMUPage {
        layout: vertical;
        padding: 1;
    }
    IMUPage .imu-header {
        text-style: bold;
        color: $accent;
        padding: 0 1 1 1;
    }
    IMUPage .imu-target-row {
        height: 3;
        padding: 0 1;
    }
    IMUPage .imu-target-row Label {
        width: 8;
        padding: 1 0;
    }
    IMUPage .imu-target-row Input {
        width: 1fr;
    }
    IMUPage .imu-target-row Button {
        margin: 0 0 0 1;
    }
    IMUPage .imu-controls {
        height: 3;
        padding: 0 1;
    }
    IMUPage .imu-controls Button {
        margin: 0 1 0 0;
    }
    IMUPage .imu-rate-label {
        width: 6;
        padding: 1 0;
    }
    IMUPage .imu-rate-input {
        width: 8;
    }
    IMUPage .imu-rate-unit {
        width: 4;
        padding: 1 0;
    }
    IMUPage .imu-frame-row {
        height: 3;
        padding: 0 1;
    }
    IMUPage .imu-frame-label {
        width: 8;
        padding: 1 0;
    }
    IMUPage .imu-frame-btn {
        width: auto;
        min-width: 8;
        margin: 0 1 0 0;
    }
    IMUPage .imu-frame-btn.active-frame {
        background: $accent;
        color: $text;
    }
    IMUPage .imu-data {
        height: 1fr;
        padding: 1;
    }
    IMUPage .imu-data-row {
        height: auto;
    }
    IMUPage .imu-status {
        height: 1;
        padding: 0 1;
        color: $text-muted;
        dock: bottom;
    }
    """

    _active: bool = False
    _streaming: bool = False
    _target_path: str = ""
    _is_component: bool = False
    _bone_name: str = ""
    _frame: str = "world"
    _prev_sample: dict | None = None
    _prev_time: float = 0.0
    _sample_count: int = 0
    _deadzone_cm: float = 0.5
    _ori_deadzone_deg: float = 0.5
    _lpf_alpha: float = 0.0
    _lpf: dict | None = None

    def compose(self) -> ComposeResult:
        yield Static("🧭 IMU Data Stream", classes="imu-header")

        with Horizontal(classes="imu-target-row"):
            yield Label("Actor:", classes="imu-target-label")
            yield Input(placeholder="Actor name or path...",
                        id="imu-actor-input")
        with Horizontal(classes="imu-target-row"):
            yield Label("Comp:", classes="imu-target-label")
            yield Input(placeholder="(optional) component class filter...",
                        id="imu-comp-input")
        with Horizontal(classes="imu-target-row"):
            yield Label("Bone:", classes="imu-target-label")
            yield Input(placeholder="(optional) bone/socket name...",
                        id="imu-bone-input")

        with Horizontal(classes="imu-controls"):
            yield Button("▶ Start", id="imu-start", variant="success")
            yield Button("⏹ Stop", id="imu-stop", variant="error",
                          disabled=True)
            yield Label("Rate:", classes="imu-rate-label")
            yield Input(value="10", restrict=r"[\d]+", id="imu-rate",
                        classes="imu-rate-input")
            yield Label("Hz", classes="imu-rate-unit")
            yield Label("DZ:", classes="imu-rate-label")
            yield Input(value="0.5", restrict=r"[\d.]+", id="imu-deadzone",
                        classes="imu-rate-input")
            yield Label("cm", classes="imu-rate-unit")
            yield Label("ODZ:", classes="imu-rate-label")
            yield Input(value="0.5", restrict=r"[\d.]+", id="imu-ori-deadzone",
                        classes="imu-rate-input")
            yield Label("°", classes="imu-rate-unit")
            yield Label("LPF:", classes="imu-rate-label")
            yield Input(value="0", restrict=r"[\d.]+", id="imu-lpf-alpha",
                        classes="imu-rate-input")

        with Horizontal(classes="imu-frame-row"):
            yield Label("Frame:", classes="imu-frame-label")
            yield Button("World", id="imu-frame-world",
                          classes="imu-frame-btn active-frame")
            yield Button("Local", id="imu-frame-local",
                          classes="imu-frame-btn")
            yield Button("Both", id="imu-frame-both",
                          classes="imu-frame-btn")

        with Horizontal(classes="imu-data"):
            with Vertical(classes="imu-data-row"):
                yield ValueDisplay("Position", ("x", "y", "z"), "m",
                                    id="imu-position")
                yield ValueDisplay("Lin Velocity", ("x", "y", "z"),
                                    "m/s", id="imu-lin-vel")
                yield ValueDisplay("Lin Velocity Local", ("x", "y", "z"),
                                    "m/s", id="imu-lin-vel-local")
            with Vertical(classes="imu-data-row"):
                yield ValueDisplay("Orientation",
                                    ("roll", "pitch", "yaw"), "deg",
                                    id="imu-orientation")
                yield ValueDisplay("Ang Velocity", ("x", "y", "z"),
                                    "deg/s", id="imu-ang-vel")
                yield ValueDisplay("Ang Velocity Local", ("x", "y", "z"),
                                    "deg/s", id="imu-ang-vel-local")
            with Vertical(classes="imu-data-row"):
                yield ValueDisplay("Lin Accel", ("x", "y", "z"),
                                    "m/s²", id="imu-lin-accel")
                yield ValueDisplay("Lin Accel Local", ("x", "y", "z"),
                                    "m/s²", id="imu-lin-accel-local")

        yield Static("  Configure target and press Start",
                      classes="imu-status", id="imu-status-bar")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "imu-start":
            self._start_streaming()
        elif event.button.id == "imu-stop":
            self._stop_streaming()
        elif event.button.id and event.button.id.startswith("imu-frame-"):
            frame = event.button.id.replace("imu-frame-", "")
            self._set_frame(frame)

    def on_mount(self) -> None:
        self._update_local_visibility()

    def _set_frame(self, frame: str) -> None:
        self._frame = frame
        for btn_id in ("imu-frame-world", "imu-frame-local", "imu-frame-both"):
            try:
                btn = self.query_one(f"#{btn_id}", Button)
                btn.set_class(btn_id == f"imu-frame-{frame}", "active-frame")
            except Exception:
                pass
        self._update_local_visibility()

    def _update_local_visibility(self) -> None:
        """Show/hide local-frame value widgets based on selected frame."""
        show_local = self._frame in ("local", "both")
        show_world_vel = self._frame in ("world", "both")
        for wid in ("imu-lin-vel-local", "imu-ang-vel-local",
                     "imu-lin-accel-local"):
            try:
                self.query_one(f"#{wid}").display = show_local
            except Exception:
                pass
        for wid in ("imu-lin-vel", "imu-ang-vel", "imu-lin-accel"):
            try:
                self.query_one(f"#{wid}").display = show_world_vel
            except Exception:
                pass

    def _start_streaming(self) -> None:
        actor_input = self.query_one("#imu-actor-input", Input).value.strip()
        if not actor_input:
            self._set_status("  ✗ Enter an actor name or path")
            return

        comp_input = self.query_one("#imu-comp-input", Input).value.strip()
        self._bone_name = self.query_one("#imu-bone-input", Input).value.strip()

        try:
            rate_hz = int(self.query_one("#imu-rate", Input).value)
        except ValueError:
            rate_hz = 10

        try:
            self._deadzone_cm = float(
                self.query_one("#imu-deadzone", Input).value)
        except ValueError:
            self._deadzone_cm = 0.5

        try:
            self._ori_deadzone_deg = float(
                self.query_one("#imu-ori-deadzone", Input).value)
        except ValueError:
            self._ori_deadzone_deg = 0.5

        try:
            alpha = float(
                self.query_one("#imu-lpf-alpha", Input).value)
        except ValueError:
            alpha = 0.0
        self._lpf_alpha = alpha
        if alpha > 0:
            self._lpf = {
                "velocity": LowPassFilter(alpha),
                "acceleration": LowPassFilter(alpha),
                "angular_velocity": LowPassFilter(alpha),
            }
        else:
            self._lpf = None

        self._resolve_and_stream(actor_input, comp_input, rate_hz)

    @work(exclusive=True, thread=True)
    def _resolve_and_stream(self, actor_hint: str, comp_hint: str,
                             rate_hz: int) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.ue or not app.connected:
            self.app.call_from_thread(
                self._set_status, "  ✗ Not connected to UE")
            return

        # Resolve target
        try:
            actor_path = None
            if "/" in actor_hint:
                actor_path = actor_hint
            else:
                actors = app.ue.find_actors()
                for a in actors:
                    if actor_hint.lower() in a.object_path.lower():
                        actor_path = a.object_path
                        break
                if not actor_path:
                    self.app.call_from_thread(
                        self._set_status,
                        f"  ✗ No actor matching '{actor_hint}'")
                    return

            self._target_path = actor_path
            self._is_component = False

            if comp_hint:
                comps = app.ue.find_components(actor_path, comp_hint)
                if comps:
                    self._target_path = comps[0]["path"]
                    self._is_component = True
                else:
                    self.app.call_from_thread(
                        self._set_status,
                        f"  ✗ No component matching '{comp_hint}'")
                    return
        except Exception as exc:
            self.app.call_from_thread(
                self._set_status, f"  ✗ Resolve error: {exc}")
            return

        # Enable streaming UI
        def _enable():
            self._streaming = True
            self._prev_sample = None
            self._prev_time = 0.0
            self._sample_count = 0
            try:
                self.query_one("#imu-start", Button).disabled = True
                self.query_one("#imu-stop", Button).disabled = False
            except Exception:
                pass
            label = self._target_path.rsplit(".", 1)[-1]
            if self._bone_name:
                label += f"[{self._bone_name}]"
            self._set_status(f"  ▶ Streaming from {label} @ {rate_hz} Hz")

        self.app.call_from_thread(_enable)

        # Streaming loop
        interval = 1.0 / max(1, rate_hz)
        while self._streaming and self._active:
            t0 = time.time()
            self._read_one_sample()
            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _read_one_sample(self) -> None:
        app: RammsTUI = self.app  # type: ignore[assignment]
        if not app.ue:
            return

        now = time.time()
        dt = now - self._prev_time if self._prev_time > 0 else 0.0
        self._prev_time = now

        obj_path = self._target_path
        position = {"x": 0, "y": 0, "z": 0}
        orientation = {"roll": 0, "pitch": 0, "yaw": 0}
        phys_velocity_cm = {"x": 0, "y": 0, "z": 0}
        phys_angular_vel = {"x": 0, "y": 0, "z": 0}

        try:
            if self._bone_name:
                result = app.ue._call_function(
                    obj_path, "GetSocketTransform",
                    {"InSocketName": self._bone_name,
                     "TransformSpace": "RTS_World"})
                if isinstance(result, dict):
                    pos = result.get("Translation", {})
                    position = {"x": pos.get("X", 0),
                                "y": pos.get("Y", 0),
                                "z": pos.get("Z", 0)}
                    rot = result.get("Rotation", {})
                    orientation = quat_to_euler(
                        rot.get("X", 0.0), rot.get("Y", 0.0),
                        rot.get("Z", 0.0), rot.get("W", 1.0))
                # Physics APIs for bone
                try:
                    vel = app.ue._call_function(
                        obj_path, "GetPhysicsLinearVelocity",
                        {"BoneName": self._bone_name})
                    if isinstance(vel, dict):
                        phys_velocity_cm = {"x": vel.get("X", 0),
                                            "y": vel.get("Y", 0),
                                            "z": vel.get("Z", 0)}
                except Exception:
                    pass
                try:
                    av = app.ue._call_function(
                        obj_path, "GetPhysicsAngularVelocityInDegrees",
                        {"BoneName": self._bone_name})
                    if isinstance(av, dict):
                        phys_angular_vel = {"x": av.get("X", 0),
                                            "y": av.get("Y", 0),
                                            "z": av.get("Z", 0)}
                except Exception:
                    pass
            elif self._is_component:
                loc = app.ue._get_property(obj_path, "RelativeLocation")
                if isinstance(loc, dict):
                    position = {"x": loc.get("X", 0),
                                "y": loc.get("Y", 0),
                                "z": loc.get("Z", 0)}
                rot = app.ue._get_property(obj_path, "RelativeRotation")
                if isinstance(rot, dict):
                    orientation = {"roll": rot.get("Roll", 0),
                                   "pitch": rot.get("Pitch", 0),
                                   "yaw": rot.get("Yaw", 0)}
                try:
                    vel = app.ue._call_function(
                        obj_path, "GetPhysicsLinearVelocity")
                    if isinstance(vel, dict):
                        phys_velocity_cm = {"x": vel.get("X", 0),
                                            "y": vel.get("Y", 0),
                                            "z": vel.get("Z", 0)}
                except Exception:
                    pass
                try:
                    av = app.ue._call_function(
                        obj_path, "GetPhysicsAngularVelocityInDegrees")
                    if isinstance(av, dict):
                        phys_angular_vel = {"x": av.get("X", 0),
                                            "y": av.get("Y", 0),
                                            "z": av.get("Z", 0)}
                except Exception:
                    pass
            else:
                loc = app.ue._call_function(obj_path, "K2_GetActorLocation")
                if isinstance(loc, dict):
                    position = {"x": loc.get("X", 0),
                                "y": loc.get("Y", 0),
                                "z": loc.get("Z", 0)}
                rot = app.ue._call_function(obj_path, "K2_GetActorRotation")
                if isinstance(rot, dict):
                    orientation = {"roll": rot.get("Roll", 0),
                                   "pitch": rot.get("Pitch", 0),
                                   "yaw": rot.get("Yaw", 0)}
                vel = app.ue._call_function(obj_path, "GetVelocity")
                if isinstance(vel, dict):
                    phys_velocity_cm = {"x": vel.get("X", 0),
                                        "y": vel.get("Y", 0),
                                        "z": vel.get("Z", 0)}
        except Exception:
            pass

        # Determine velocity and angular velocity from physics or deltas
        angular_velocity = {"x": 0, "y": 0, "z": 0}
        linear_accel = {"x": 0, "y": 0, "z": 0}
        has_phys_vel = any(phys_velocity_cm.values())
        has_phys_angvel = any(phys_angular_vel.values())

        if self._prev_sample and dt > 0:
            pp = self._prev_sample["position"]  # cm

            # Linear velocity
            if has_phys_vel:
                velocity_cm = phys_velocity_cm
            else:
                pos_delta = {
                    "x": position["x"] - pp["x"],
                    "y": position["y"] - pp["y"],
                    "z": position["z"] - pp["z"],
                }
                if self._deadzone_cm > 0:
                    pos_delta = apply_deadzone(pos_delta, self._deadzone_cm)
                velocity_cm = {k: pos_delta[k] / dt for k in pos_delta}

            velocity_m = cm_to_m_vec(velocity_cm)
            prev_vel = self._prev_sample.get(
                "velocity_m", {"x": 0, "y": 0, "z": 0})

            # Acceleration from velocity delta (consistent source)
            linear_accel = {
                "x": (velocity_m["x"] - prev_vel["x"]) / dt,
                "y": (velocity_m["y"] - prev_vel["y"]) / dt,
                "z": (velocity_m["z"] - prev_vel["z"]) / dt,
            }

            # Angular velocity
            if has_phys_angvel:
                angular_velocity = phys_angular_vel
            else:
                po = self._prev_sample["orientation"]
                ori_delta = {
                    "x": angle_diff(orientation["roll"], po["roll"]),
                    "y": angle_diff(orientation["pitch"], po["pitch"]),
                    "z": angle_diff(orientation["yaw"], po["yaw"]),
                }
                if self._ori_deadzone_deg > 0:
                    ori_delta = apply_deadzone(ori_delta,
                                               self._ori_deadzone_deg)
                angular_velocity = {k: ori_delta[k] / dt for k in ori_delta}

            vel_for_next = velocity_m
        else:
            vel_for_next = cm_to_m_vec(phys_velocity_cm) if has_phys_vel \
                else {"x": 0, "y": 0, "z": 0}

        linear_velocity_m = vel_for_next if self._prev_sample \
            else cm_to_m_vec(phys_velocity_cm)

        # Apply low-pass filters
        if self._lpf:
            if "velocity" in self._lpf:
                linear_velocity_m = self._lpf["velocity"](linear_velocity_m)
            if "acceleration" in self._lpf:
                linear_accel = self._lpf["acceleration"](linear_accel)
            if "angular_velocity" in self._lpf:
                angular_velocity = self._lpf["angular_velocity"](
                    angular_velocity)

        # Convert position to metres for display
        position_m = cm_to_m_vec(position)

        self._prev_sample = {
            "position": position,   # keep cm for delta computation
            "orientation": orientation,
            "velocity_m": vel_for_next,  # m/s for next iteration
        }
        self._sample_count += 1

        # Compute local-frame values
        lin_vel_local = world_to_local(linear_velocity_m, orientation)
        ang_vel_local = world_to_local(angular_velocity, orientation)
        lin_accel_local = world_to_local(linear_accel, orientation)

        def _update():
            try:
                self.query_one("#imu-position", ValueDisplay).update_values(
                    position_m)
                self.query_one("#imu-orientation", ValueDisplay).update_values(
                    orientation)
                self.query_one("#imu-lin-vel", ValueDisplay).update_values(
                    linear_velocity_m)
                self.query_one("#imu-ang-vel", ValueDisplay).update_values(
                    angular_velocity)
                self.query_one("#imu-lin-accel", ValueDisplay).update_values(
                    linear_accel)
                self.query_one("#imu-lin-vel-local", ValueDisplay).update_values(
                    lin_vel_local)
                self.query_one("#imu-ang-vel-local", ValueDisplay).update_values(
                    ang_vel_local)
                self.query_one("#imu-lin-accel-local", ValueDisplay).update_values(
                    lin_accel_local)
            except Exception:
                pass

        self.app.call_from_thread(_update)

    def _stop_streaming(self) -> None:
        self._streaming = False
        try:
            self.query_one("#imu-start", Button).disabled = False
            self.query_one("#imu-stop", Button).disabled = True
        except Exception:
            pass
        self._set_status(
            f"  ⏹ Stopped ({self._sample_count} samples collected)")

    def _set_status(self, text: str) -> None:
        try:
            self.query_one("#imu-status-bar", Static).update(text)
        except Exception:
            pass

    def activate(self) -> None:
        self._active = True

    def deactivate(self) -> None:
        self._active = False
        if self._streaming:
            self._stop_streaming()
