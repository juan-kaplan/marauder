import math

import numpy as np
import pytest

from rota_common import se2
from rota_common.icp import IcpParams, align

PLANE = IcpParams(method='point_to_plane')
POINT = IcpParams(method='point_to_point')

# Point-to-point ICP stops within about half the point spacing of the truth
# (each point settles on a neighbouring sample of the same wall), so with
# 2 cm spacing we only ask it for 1 cm and 0.01 rad. Point-to-plane has no
# such floor.
TOL = {'point_to_plane': 1e-3, 'point_to_point': 0.01}
both = pytest.mark.parametrize('params', [PLANE, POINT], ids=lambda p: p.method)


def segment(a, b, step):
    n = max(2, int(np.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return np.linspace(a, b, n)


def room(step=0.02, offset=0.0):
    """
    Points on the walls of a 4 x 3 m room with a box in one corner (no symmetries).

    ``offset`` shifts the samples along each wall by that fraction of ``step``,
    like a second scan that hits the walls at other places.
    """
    corners = [(-1.5, -1.0), (2.5, -1.0), (2.5, 2.0), (-1.5, 2.0), (-1.5, -1.0)]
    lines = [(corners[i], corners[i + 1]) for i in range(4)]
    lines += [((1.5, 1.2), (2.5, 1.2)), ((1.5, 1.2), (1.5, 2.0))]
    walls = []
    for a, b in lines:
        a, b = np.asarray(a), np.asarray(b)
        u = (b - a) / np.linalg.norm(b - a)
        walls.append(segment(a + offset * step * u, b, step))
    return np.vstack(walls)


def seen_from(points, pose):
    """The points of ``points`` (target frame) as seen by a robot at ``pose`` in that frame."""
    T = np.linalg.inv(se2.to_matrix(pose))
    return points @ T[:2, :2].T + T[:2, 3]


@both
def test_recovers_motion_from_noisy_guess(params):
    target = room()
    truth = (0.25, -0.08, 0.15)
    guess = se2.compose(truth, (0.04, -0.03, 0.04))
    r = align(seen_from(target, truth), target, guess, params)
    assert r.ok
    assert r.fitness > 0.95
    assert r.rmse < 0.02
    assert r.pose == pytest.approx(truth, abs=TOL[params.method])


@both
def test_recovers_small_motion_from_identity(params):
    target = room()
    truth = (0.08, 0.03, -0.05)
    r = align(seen_from(target, truth), target, se2.IDENTITY, params)
    assert r.ok
    assert r.pose == pytest.approx(truth, abs=TOL[params.method])


def test_point_to_plane_ignores_different_sampling():
    # The two scans see the same walls at different points, as real scans do.
    target = room(0.05)
    truth = (0.2, -0.1, 0.12)
    source = seen_from(room(0.05, offset=0.5), truth)
    guess = se2.compose(truth, (0.03, 0.03, 0.03))
    plane = align(source, target, guess, PLANE)
    point = align(source, target, guess, POINT)
    assert plane.ok and point.ok
    assert plane.pose == pytest.approx(truth, abs=1e-3)
    plane_err = np.abs(np.subtract(plane.pose, truth))
    point_err = np.abs(np.subtract(point.pose, truth))
    assert plane_err[2] < point_err[2]


@both
def test_handles_sensor_noise(params):
    rng = np.random.default_rng(0)
    target = room() + rng.normal(0.0, 0.01, (len(room()), 2))
    truth = (0.2, 0.1, -0.1)
    source = seen_from(room(), truth) + rng.normal(0.0, 0.01, (len(room()), 2))
    r = align(source, target, se2.compose(truth, (0.03, 0.03, 0.03)), params)
    assert r.ok
    assert r.pose == pytest.approx(truth, abs=0.02)


@both
def test_too_few_points_returns_guess(params):
    guess = (0.1, 0.2, 0.3)
    r = align(np.zeros((3, 2)), room(), guess, params)
    assert not r.ok
    assert r.pose == guess
    assert r.fitness == 0.0
    assert math.isinf(r.rmse)


@both
def test_unrelated_scan_is_rejected(params):
    target = room()
    far_away = target + np.array([10.0, 10.0])
    r = align(far_away, target, se2.IDENTITY, params)
    assert not r.ok
    assert r.fitness < params.min_fitness


def test_unknown_method():
    with pytest.raises(ValueError):
        align(room(), room(), se2.IDENTITY, IcpParams(method='nope'))
