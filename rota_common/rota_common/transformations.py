"""
Drop-in replacements for the subset of ``tf_transformations`` used in this repo.

``tf_transformations`` depends on ``transforms3d``; the version shipped with
Ubuntu 22.04 (ROS 2 Humble) breaks under NumPy 2. These pure-NumPy versions
work on Humble and Jazzy with NumPy 1.x or 2.x.

Conventions match ``tf_transformations``: quaternions are ``[x, y, z, w]`` and
Euler angles use static 'sxyz' axes (roll, pitch, yaw).
"""

import math

import numpy as np


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return quaternion [x, y, z, w] from static-xyz Euler angles."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quaternion_matrix(quaternion) -> np.ndarray:
    """Return the 4x4 homogeneous rotation matrix of quaternion [x, y, z, w]."""
    x, y, z, w = (float(v) for v in quaternion)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w), 0.0],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w), 0.0],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def euler_from_matrix(matrix, axes: str = 'sxyz'):
    """Return (roll, pitch, yaw) from a 3x3 or 4x4 rotation matrix ('sxyz' only)."""
    if axes != 'sxyz':
        raise ValueError(f"Only 'sxyz' axes are supported, got '{axes}'")
    R = np.asarray(matrix, dtype=np.float64)[:3, :3]
    cy = math.hypot(R[0, 0], R[1, 0])
    if cy > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], cy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], cy)
        yaw = 0.0
    return roll, pitch, yaw


def euler_from_quaternion(quaternion, axes: str = 'sxyz'):
    """Return (roll, pitch, yaw) from quaternion [x, y, z, w] ('sxyz' only)."""
    return euler_from_matrix(quaternion_matrix(quaternion), axes)
