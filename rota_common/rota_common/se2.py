"""
SE(2) poses as (x, y, theta) tuples. compose(a, b) = a ⊕ b and
between(a, b) = a⁻¹ ⊕ b, the pose of b seen from a.
"""

import math

import numpy as np

IDENTITY = (0.0, 0.0, 0.0)


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            normalize_angle(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], normalize_angle(-a[2]))


def between(a, b):
    return compose(inverse(a), b)


def norm(a):
    return math.hypot(a[0], a[1])


def to_matrix(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([[c, -s, 0.0, p[0]], [s, c, 0.0, p[1]], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


def from_matrix(T):
    return (float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0]))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
