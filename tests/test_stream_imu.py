"""Offline tests for ramms_tools.scripts.stream_imu (run: python -m pytest tests)."""
import pytest

from ramms_tools.scripts.stream_imu import read_imu_sample, resolve_target
from ramms_tools.unreal_remote import UnrealRemoteError


class FakeUE:
    """Answers the Remote Control calls the IMU makes, from canned values."""

    def __init__(self, locations=(), socket_transform=None, sockets=None):
        self.locations = list(locations)
        self.socket_transform = socket_transform
        self.sockets = sockets
        self.calls = []

    def _call_function(self, path, func, params=None):
        self.calls.append(func)
        if func == "K2_GetActorLocation":
            return self.locations.pop(0)
        if func == "K2_GetActorRotation":
            return {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
        if func in ("GetVelocity", "GetPhysicsLinearVelocity", "GetPhysicsAngularVelocityInDegrees"):
            return {"X": 0.0, "Y": 0.0, "Z": 0.0}   # at rest, or kinematic: no physics velocity
        if func == "GetSocketTransform":
            return self.socket_transform
        if func == "DoesSocketExist":
            if self.sockets is None:
                raise UnrealRemoteError("not allow-listed")
            return params["InSocketName"] in self.sockets
        raise AssertionError(func)

    def _get_property(self, path, name):
        raise AssertionError("components are read in world space, not from Relative* properties")

    def find_components(self, actor_path, hint):
        return [{"path": actor_path + ".Mesh", "name": "Mesh"}]


ACTOR = {"object_path": "/Game/M.M:PersistentLevel.Chair_C_0", "bone_name": None, "label": "Chair"}


def test_velocity_from_positions_when_physics_reads_zero():
    # An actor standing still at (500, 0, 100) cm, then moving 10 cm in x in 0.1 s.
    ue = FakeUE([{"X": 500.0, "Y": 0.0, "Z": 100.0}] * 2 + [{"X": 510.0, "Y": 0.0, "Z": 100.0}])
    s0 = read_imu_sample(ue, ACTOR, None, 0.0)
    s1 = read_imu_sample(ue, ACTOR, s0, 0.1)
    assert s1["linear_velocity"] == pytest.approx({"x": 0.0, "y": 0.0, "z": 0.0})
    assert s1["position"] == pytest.approx({"x": 5.0, "y": 0.0, "z": 1.0})   # metres
    s2 = read_imu_sample(ue, ACTOR, s1, 0.1)
    assert s2["linear_velocity"]["x"] == pytest.approx(1.0)                   # 10 cm / 0.1 s


def test_component_pose_is_world_space():
    target = {"object_path": "/Game/M.M:PersistentLevel.Chair_C_0.Mesh", "bone_name": None,
              "label": "Chair.Mesh", "is_component": True}
    ue = FakeUE(socket_transform={"Translation": {"X": 100.0, "Y": 200.0, "Z": 300.0},
                                  "Rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0}})
    s = read_imu_sample(ue, target, None, 0.0)
    assert s["position"] == pytest.approx({"x": 1.0, "y": 2.0, "z": 3.0})
    assert "GetSocketTransform" in ue.calls


class Actor:
    object_path = "/Game/M.M:PersistentLevel.Chair_C_0"


def test_bone_needs_a_component():
    ue = FakeUE()
    ue.find_actors = lambda: [Actor()]
    with pytest.raises(RuntimeError, match="--component"):
        resolve_target(ue, "Chair", "", "link_6")


def test_unknown_bone_is_refused():
    ue = FakeUE(sockets={"link_6", "end_effector"})
    ue.find_actors = lambda: [Actor()]
    assert resolve_target(ue, "Chair", "Mesh", "end_effector")["bone_name"] == "end_effector"
    with pytest.raises(RuntimeError, match="No bone or socket"):
        resolve_target(ue, "Chair", "Mesh", "end_effector_link")


def test_bone_check_skipped_when_not_allow_listed():
    ue = FakeUE(sockets=None)
    ue.find_actors = lambda: [Actor()]
    assert resolve_target(ue, "Chair", "Mesh", "anything")["bone_name"] == "anything"
