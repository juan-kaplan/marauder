import math
import random

import numpy as np
import pytest

from rota_common import se2
from rota_common.transformations import euler_from_quaternion, quaternion_from_euler


def close(a, b, tol=1e-9):
    return (math.isclose(a[0], b[0], abs_tol=tol) and math.isclose(a[1], b[1], abs_tol=tol)
            and math.isclose(se2.normalize_angle(a[2] - b[2]), 0.0, abs_tol=tol))


def random_pose(rng):
    return (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi))


@pytest.mark.parametrize('a, expected', [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi),
    (-3 * math.pi, math.pi), (math.pi / 2, math.pi / 2), (-math.pi / 2, -math.pi / 2),
    (2 * math.pi + 0.1, 0.1), (-2 * math.pi - 0.1, -0.1),
])
def test_normalize_angle(a, expected):
    assert math.isclose(se2.normalize_angle(a), expected, abs_tol=1e-9)


def test_compose_known():
    # Robot at (1, 0) facing +y moves 2 m forward and turns left.
    assert close(se2.compose((1.0, 0.0, math.pi / 2), (2.0, 0.0, math.pi / 2)), (1.0, 2.0, math.pi))


def test_between_known():
    # Target 2 m in front of a reference that faces +y.
    assert close(se2.between((0.0, 0.0, math.pi / 2), (0.0, 2.0, math.pi / 2)), (2.0, 0.0, 0.0))


def test_group_identities():
    rng = random.Random(0)
    for _ in range(200):
        a, b, c = random_pose(rng), random_pose(rng), random_pose(rng)
        assert close(se2.compose(a, se2.inverse(a)), se2.IDENTITY)
        assert close(se2.compose(se2.inverse(a), a), se2.IDENTITY)
        assert close(se2.compose(se2.compose(a, b), c), se2.compose(a, se2.compose(b, c)))
        assert close(se2.compose(a, se2.between(a, b)), b)
        assert close(se2.between(b, a), se2.inverse(se2.between(a, b)))


def test_matches_matrices():
    rng = random.Random(1)
    for _ in range(100):
        a, b = random_pose(rng), random_pose(rng)
        assert close(se2.from_matrix(se2.to_matrix(a)), a)
        assert close(se2.from_matrix(se2.to_matrix(a) @ se2.to_matrix(b)), se2.compose(a, b))
        assert close(se2.from_matrix(np.linalg.inv(se2.to_matrix(a))), se2.inverse(a))


def test_from_matrix_3x3():
    T = se2.to_matrix((0.5, -0.2, 0.3))
    assert close(se2.from_matrix(T[np.ix_([0, 1, 3], [0, 1, 3])]), (0.5, -0.2, 0.3))


def test_norm():
    assert se2.norm((3.0, 4.0, 1.0)) == pytest.approx(5.0)


def test_quaternion_roundtrip_and_tf_convention():
    for yaw in np.linspace(-3.1, 3.1, 25):
        q = se2.quaternion_from_yaw(yaw)
        assert se2.yaw_from_quaternion(q) == pytest.approx(yaw)
        assert np.allclose(q, quaternion_from_euler(0.0, 0.0, yaw))
        assert euler_from_quaternion(q)[2] == pytest.approx(yaw)


def test_yaw_from_quaternion_message_like():
    class Q:
        x, y, z, w = se2.quaternion_from_yaw(-1.2)
    assert se2.yaw_from_quaternion(Q()) == pytest.approx(-1.2)
