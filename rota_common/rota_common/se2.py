"""
SE(2) pose algebra on plain ``(x, y, theta)`` tuples.

Conventions (see ``docs/tp1/arquitectura.md``):

* ``compose(a, b) = a ⊕ b``: pose ``b`` given in frame ``a``, expressed in the
  parent frame of ``a``.
* ``between(a, b) = a⁻¹ ⊕ b``: pose of ``b`` seen from ``a``.

No ROS imports, so it can be used and tested anywhere.
"""

import math
from typing import Sequence, Tuple

import numpy as np

Pose2D = Tuple[float, float, float]
IDENTITY: Pose2D = (0.0, 0.0, 0.0)


def normalize_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    if a <= -math.pi or math.isclose(a, -math.pi, abs_tol=1e-12):
        return math.pi
    return a


def compose(a: Sequence[float], b: Sequence[float]) -> Pose2D:
    """Return ``a ⊕ b``."""
    xa, ya, ta = a
    xb, yb, tb = b
    c, s = math.cos(ta), math.sin(ta)
    return (xa + c * xb - s * yb,
            ya + s * xb + c * yb,
            normalize_angle(ta + tb))


def inverse(a: Sequence[float]) -> Pose2D:
    """Return ``a⁻¹``, so that ``compose(a, inverse(a))`` is the identity."""
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y,
            s * x - c * y,
            normalize_angle(-t))


def between(a: Sequence[float], b: Sequence[float]) -> Pose2D:
    """Return ``a⁻¹ ⊕ b``: the pose of ``b`` seen from ``a``."""
    return compose(inverse(a), b)


def norm(a: Sequence[float]) -> float:
    """Translational length of a pose (the angle is ignored)."""
    return math.hypot(a[0], a[1])


def to_matrix(p: Sequence[float]) -> np.ndarray:
    """Return the 4x4 homogeneous matrix of a planar pose."""
    x, y, t = p
    c, s = math.cos(t), math.sin(t)
    T = np.eye(4)
    T[0, 0], T[0, 1], T[0, 3] = c, -s, x
    T[1, 0], T[1, 1], T[1, 3] = s, c, y
    return T


def from_matrix(T) -> Pose2D:
    """Return ``(x, y, theta)`` of a 3x3 or 4x4 homogeneous matrix (drops z, roll, pitch)."""
    T = np.asarray(T, dtype=np.float64)
    tx = T[0, -1]
    ty = T[1, -1]
    return (float(tx), float(ty), normalize_angle(math.atan2(T[1, 0], T[0, 0])))


def yaw_from_quaternion(q) -> float:
    """Yaw of a quaternion, given as a message with ``x, y, z, w`` or as ``[x, y, z, w]``."""
    if hasattr(q, 'w'):
        x, y, z, w = q.x, q.y, q.z, q.w
    else:
        x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return the quaternion ``(x, y, z, w)`` of a rotation of ``yaw`` about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
