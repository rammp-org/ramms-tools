"""Coordinate-frame transform utilities for Unreal Engine data.

UE uses a **left-handed** coordinate system (X-forward, Y-right, Z-up) with
intrinsic Euler rotation order Roll(X) → Pitch(Y) → Yaw(Z).  Positive yaw
rotates from X toward Y, positive pitch tilts from X toward Z (nose-up), and
positive roll tilts from Y toward -Z (right-side-down).

Because UE's pitch and roll conventions are sign-flipped relative to the
standard right-handed Rz·Ry·Rx formula, the matrix builder negates roll and
pitch internally.
"""

from __future__ import annotations

import math


def angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference *a − b* in degrees (−180, 180]."""
    d = a - b
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def rotation_matrix_from_euler(
    roll_deg: float, pitch_deg: float, yaw_deg: float,
) -> list[list[float]]:
    """Build a 3×3 local→world rotation matrix from UE Euler angles.

    Returns *R* such that ``v_world = R @ v_local`` (column-vector convention).

    Internally computes the standard Rz(yaw)·Ry(pitch)·Rx(roll) product with
    roll and pitch negated to account for UE's left-handed sign conventions.
    """
    # Negate roll and pitch for UE left-handed convention
    r = math.radians(-roll_deg)
    p = math.radians(-pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    # R = Rz(yaw) · Ry(-pitch) · Rx(-roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def world_to_local(vec_xyz: dict, orientation: dict) -> dict:
    """Transform a world-frame ``{x, y, z}`` vector into the local body frame.

    Uses the transpose (inverse) of the local→world rotation matrix built from
    *orientation* ``{roll, pitch, yaw}`` in degrees.
    """
    r = rotation_matrix_from_euler(
        orientation["roll"], orientation["pitch"], orientation["yaw"],
    )
    # v_local = R^T @ v_world
    rt = _transpose_3x3(r)
    lx, ly, lz = _mat_vec_mul(
        rt, (vec_xyz["x"], vec_xyz["y"], vec_xyz["z"]),
    )
    return {"x": lx, "y": ly, "z": lz}


# ── internal helpers ────────────────────────────────────────────────

def _transpose_3x3(m: list[list[float]]) -> list[list[float]]:
    return [[m[j][i] for j in range(3)] for i in range(3)]


def _mat_vec_mul(
    m: list[list[float]], v: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def quat_to_euler(x: float, y: float, z: float, w: float) -> dict:
    """Convert a quaternion (x, y, z, w) to UE Euler angles in degrees.

    Returns ``{roll, pitch, yaw}`` matching UE's ``FQuat::Rotator()``
    convention exactly.
    """
    # Roll (X) — note the negated numerator vs standard formula
    roll = math.degrees(math.atan2(
        -2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y)))

    # Pitch (Y) — UE uses Z*X - W*Y, not W*Y - Z*X
    sinp = 2.0 * (z * x - w * y)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    # Yaw (Z) — same as standard
    yaw = math.degrees(math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)))

    return {"roll": roll, "pitch": pitch, "yaw": yaw}


# ── Signal filtering ────────────────────────────────────────────────

CM_TO_M = 0.01


def apply_deadzone(vec: dict, threshold_cm: float) -> dict:
    """Zero out a position-delta vector if its magnitude is below threshold.

    *threshold_cm* is in centimetres (UE native unit).  Returns a new dict.
    """
    mag_sq = sum(v * v for v in vec.values())
    if mag_sq < threshold_cm * threshold_cm:
        return {k: 0.0 for k in vec}
    return dict(vec)


def cm_to_m_vec(vec: dict) -> dict:
    """Convert a ``{x, y, z}`` vector from centimetres to metres."""
    return {k: v * CM_TO_M for k, v in vec.items()}


class LowPassFilter:
    """First-order exponential low-pass filter for ``{x, y, z}`` dicts.

    Parameters
    ----------
    alpha : float
        Smoothing factor in (0, 1].  Smaller = smoother / more lag.
        A good starting point is ``alpha = dt / (rc + dt)`` where *rc* is
        the desired time-constant and *dt* the sample period.
    """

    def __init__(self, alpha: float = 0.3) -> None:
        self._alpha = max(0.0, min(1.0, alpha))
        self._state: dict | None = None

    @property
    def alpha(self) -> float:
        return self._alpha

    @alpha.setter
    def alpha(self, value: float) -> None:
        self._alpha = max(0.0, min(1.0, value))

    def reset(self) -> None:
        self._state = None

    def __call__(self, raw: dict) -> dict:
        if self._state is None:
            self._state = dict(raw)
            return dict(raw)
        a = self._alpha
        self._state = {
            k: a * raw[k] + (1.0 - a) * self._state[k]
            for k in raw
        }
        return dict(self._state)
