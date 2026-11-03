"""Offline tests for ramms_tools.scripts.control_rammp_gen1 (run: python -m pytest tests)."""
import pytest

from ramms_tools.scripts.control_rammp_gen1 import set_angular_motor, set_linear_motor


class FakeController:
    def __init__(self):
        self.sent = []

    def call(self, func, **params):
        if func == "GetAngularMotors":
            return [{"ConstraintName": "motor_swing_arm_l"}, {"ConstraintName": "front_caster_swing_arm"}]
        if func == "GetLinearMotors":
            return [{"ConstraintName": "dw_main_plate_l"}]
        self.sent.append((func, params))


def test_known_motor_is_commanded():
    c = FakeController()
    set_angular_motor(c, "motor_swing_arm_l", 5.0)
    set_linear_motor(c, "DW_Main_Plate_L", 2.0)   # Unreal's names ignore case
    assert [f for f, _ in c.sent] == ["SetAngularMotorTarget", "SetLinearMotorTarget"]


def test_unknown_motor_is_refused():
    # The controller ignores an unknown name, so the command would silently do nothing.
    c = FakeController()
    with pytest.raises(ValueError, match="no angular motor 'YawMotor'"):
        set_angular_motor(c, "YawMotor", 45.0)
    with pytest.raises(ValueError, match="no linear motor"):
        set_linear_motor(c, "motor_swing_arm_l", 1.0)
    assert c.sent == []
