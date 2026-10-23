import math

import numpy as np
import pytest

from rota_common import se2
from rota_localization.localizer import (EncoderNoise, Localizer, integrate_diff_drive,
                                         interpolate)

R = 0.033   # wheel radius
B = 0.160   # wheel separation
SEC = 1_000_000_000

# A 6 x 4 m room with a few boxes, so ICP has corners to lock onto. The test
# trajectories stay clear of them.
SEGMENTS = np.array([
    [(-1.0, -2.0), (5.0, -2.0)], [(5.0, -2.0), (5.0, 2.0)], [(5.0, 2.0), (-1.0, 2.0)],
    [(-1.0, 2.0), (-1.0, -2.0)],
    [(3.8, 1.2), (4.4, 1.2)], [(3.8, 1.2), (3.8, 2.0)],
    [(3.0, -2.0), (3.0, -1.4)], [(3.0, -1.4), (3.5, -1.4)],
    [(0.2, -1.0), (0.5, -1.0)], [(0.5, -1.0), (0.5, -0.7)],
])


def raycast(pose, beams=360, max_range=3.5):
    """Points in the robot frame hit by a 360-beam LiDAR at ``pose``."""
    x, y, th = pose
    a = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    d = np.stack([np.cos(th + a), np.sin(th + a)], axis=1)          # (N, 2)
    p, e = SEGMENTS[:, 0], SEGMENTS[:, 1] - SEGMENTS[:, 0]           # (M, 2)
    w = p - np.array([x, y])
    den = d[:, None, 0] * -e[None, :, 1] - d[:, None, 1] * -e[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (w[None, :, 0] * -e[None, :, 1] - w[None, :, 1] * -e[None, :, 0]) / den
        u = (d[:, None, 0] * w[None, :, 1] - d[:, None, 1] * w[None, :, 0]) / den
    s = np.where((np.abs(den) > 1e-12) & (s > 0.12) & (u >= 0) & (u <= 1), s, np.inf)
    r = s.min(axis=1)
    keep = r < max_range
    return np.stack([r[keep] * np.cos(a[keep]), r[keep] * np.sin(a[keep])], axis=1)


def drive(loc, commands, wheel_hz=30, scan_hz=10, scan_latency=0.02, scans=True,
          scan_gap=None, start=(0.0, 0.0, 0.0), t0=5 * SEC):
    """
    Feed ``loc`` wheels and scans for a list of ``(v, omega, seconds)``.

    Returns the ground truth pose at the end and all references created, in
    order. ``scan_gap=(a, b)`` drops the scans stamped between a and b seconds.
    """
    truth = start
    left = right = 0.0
    refs = []
    events = []
    t = 0.0
    dt = 1.0 / wheel_hz
    next_scan = 0.0
    events.append((t0, 'wheels', (left, right)))
    for v, omega, duration in commands:
        for _ in range(int(round(duration * wheel_hz))):
            dl = (v - omega * B / 2) * dt
            dr = (v + omega * B / 2) * dt
            truth = integrate_diff_drive(truth, dl, dr, B)
            left += dl / R
            right += dr / R
            t += dt
            stamp = t0 + int(round(t * SEC))
            events.append((stamp, 'wheels', (left, right)))
            if scans and t >= next_scan - 1e-9:
                next_scan += 1.0 / scan_hz
                if scan_gap is None or not (scan_gap[0] <= t <= scan_gap[1]):
                    arrival = stamp + int(scan_latency * SEC)
                    events.append((arrival, 'scan', (stamp, raycast(truth))))
    events.sort(key=lambda e: e[0])
    for arrival, kind, data in events:
        if kind == 'wheels':
            out = loc.on_wheels(arrival, *data)
        else:
            out = loc.on_scan(*data)
        if out:
            refs.extend(out)
    return truth, refs


def assert_pose(actual, expected, pos=1e-6, ang=1e-6):
    assert math.hypot(actual[0] - expected[0], actual[1] - expected[1]) < pos
    assert abs(se2.normalize_angle(actual[2] - expected[2])) < ang


# ---------------------------------------------------------------- kinematics

def test_integrate_straight_and_spin():
    assert_pose(integrate_diff_drive((0, 0, 0), 0.5, 0.5, B), (0.5, 0.0, 0.0))
    assert_pose(integrate_diff_drive((1, 2, math.pi / 2), 0.5, 0.5, B), (1.0, 2.5, math.pi / 2))
    assert_pose(integrate_diff_drive((0, 0, 0), -0.08 * math.pi / 2, 0.08 * math.pi / 2, B),
                (0.0, 0.0, math.pi / 2))


def test_integrate_arc_matches_closed_form():
    # Quarter circle of radius 1 m in many small steps.
    pose = (0.0, 0.0, 0.0)
    n = 200
    arc = math.pi / 2 / n
    for _ in range(n):
        pose = integrate_diff_drive(pose, (1 - B / 2) * arc, (1 + B / 2) * arc, B)
    assert_pose(pose, (1.0, 1.0, math.pi / 2), pos=1e-4, ang=1e-9)


def test_interpolate_wraps_angle():
    p = interpolate((0, 0, 3.0), (1, 2, -3.0), 0.5)
    assert p[:2] == pytest.approx((0.5, 1.0))
    assert abs(se2.normalize_angle(p[2] - math.pi)) < 1e-9


# ------------------------------------------------------------------ encoders

def test_encoders_first_message_creates_reference_zero():
    loc = Localizer('encoders')
    refs = loc.on_wheels(7, 1.0, 2.0)
    assert len(refs) == 1
    assert refs[0].stamp == 7
    assert refs[0].pose == se2.IDENTITY
    assert refs[0].points is None
    assert loc.started and loc.rel == se2.IDENTITY


def test_encoders_straight_line_references_every_tau_d():
    loc = Localizer('encoders', tau_d=0.30, tau_theta=0.30)
    truth, refs = drive(loc, [(0.2, 0.0, 5.0)], scans=False)   # 1 m
    assert_pose(loc.odom_pose, truth, pos=1e-9)
    assert len(refs) == 1 + 3
    step = 0.2 / 30   # a reference is created on the first wheel step past tau_d
    for a, b in zip(refs, refs[1:]):
        assert 0.30 <= b.pose[0] - a.pose[0] <= 0.30 + step + 1e-9
    assert_pose(se2.compose(loc.ref_pose, loc.rel), loc.odom_pose, pos=1e-12)
    assert se2.norm(loc.rel) < 0.30
    assert [r.stamp for r in refs] == sorted(r.stamp for r in refs)


def test_encoders_spin_in_place_uses_tau_theta():
    loc = Localizer('encoders', tau_d=0.30, tau_theta=0.30)
    truth, refs = drive(loc, [(0.0, 0.5, 2.0)], scans=False)   # 1 rad
    assert_pose(loc.odom_pose, truth, pos=1e-9, ang=1e-9)
    assert len(refs) == 1 + 3


def test_encoders_odom_pose_does_not_jump_at_references():
    loc = Localizer('encoders')
    poses = []
    _, _ = drive(loc, [(0.0, 0.0, 0.1)], scans=False)
    left = right = 0.0
    t = 10 * SEC
    for i in range(300):
        left += 0.2 / 30 / R
        right += 0.25 / 30 / R
        t += SEC // 30
        loc.on_wheels(t, left, right)
        poses.append(loc.odom_pose)
    steps = [se2.norm(se2.between(a, b)) for a, b in zip(poses, poses[1:])]
    assert max(steps) < 0.01
    assert loc.num_references > 5


def test_encoders_ignore_scans_and_old_stamps():
    loc = Localizer('encoders')
    loc.on_wheels(10, 0.0, 0.0)
    assert loc.on_scan(11, np.zeros((50, 2))) is None
    assert loc.on_wheels(10, 1.0, 1.0) is None
    assert loc.rel == se2.IDENTITY


# ------------------------------------------------------------- encoder noise

def test_noise_disabled_by_default():
    assert not EncoderNoise().enabled
    assert EncoderNoise()(0.1, 0.2) == (0.1, 0.2)


def test_noise_bias_turns_left_and_seed_is_reproducible():
    loc = Localizer('encoders', encoder_noise=EncoderNoise(bias=0.02))
    _, _ = drive(loc, [(0.2, 0.0, 5.0)], scans=False)
    assert loc.odom_pose[2] > 0.1   # believes it turned left on a straight line

    a = EncoderNoise(std=0.01, seed=3)
    b = EncoderNoise(std=0.01, seed=3)
    assert [a(0.01, 0.01) for _ in range(5)] == [b(0.01, 0.01) for _ in range(5)]


def test_noise_std_grows_with_sqrt_of_distance():
    noise = EncoderNoise(std=0.05, seed=0)
    totals = []
    for _ in range(400):
        totals.append(sum(noise(0.001, 0.001)[0] for _ in range(1000)) - 1.0)   # 1 m
    assert np.std(totals) == pytest.approx(0.05, rel=0.15)


# --------------------------------------------------------------------- lidar

ARC = [(0.2, 0.0, 6.0), (0.15, 0.4, 5.0), (0.2, -0.1, 5.0)]


def test_lidar_follows_ground_truth():
    loc = Localizer('lidar')
    truth, refs = drive(loc, ARC)
    assert loc.num_references == len(refs) > 5
    assert all(r.points is not None and len(r.points) > 100 for r in refs)
    assert_pose(loc.odom_pose, truth, pos=0.05, ang=0.03)


def test_lidar_ignores_wheels():
    loc = Localizer('lidar')
    assert loc.on_wheels(1, 0.0, 0.0) is None
    assert loc.on_scan(2, raycast((0, 0, 0)))[0].stamp == 2


# --------------------------------------------------------------------- mixed

def test_mixed_corrects_biased_encoders():
    bias = EncoderNoise(bias=0.03)
    enc = Localizer('encoders', encoder_noise=bias)
    truth, _ = drive(enc, ARC, scans=False)
    mixed = Localizer('mixed', encoder_noise=EncoderNoise(bias=0.03))
    truth_m, refs = drive(mixed, ARC)
    assert truth_m == truth
    enc_err = se2.norm(se2.between(enc.odom_pose, truth))
    mixed_err = se2.norm(se2.between(mixed.odom_pose, truth))
    assert enc_err > 0.2
    assert mixed_err < 0.05
    assert all(r.points is not None for r in refs)


def test_mixed_scan_dropout_uses_wheels_then_refixes():
    loc = Localizer('mixed', scan_timeout=0.5)
    truth, refs = drive(loc, [(0.2, 0.0, 10.0)], scan_gap=(2.0, 6.0))
    without = [r for r in refs if r.points is None]
    assert len(without) >= 2                       # ~0.8 m travelled blind
    first_after = refs.index(without[-1]) + 1
    assert refs[first_after].points is not None     # the next scan fixes it
    first_scan_after_gap = (refs[first_after].stamp - 5 * SEC) / SEC
    assert 6.0 <= first_scan_after_gap <= 6.1 + 1e-6
    assert_pose(loc.odom_pose, truth, pos=0.03, ang=0.02)


def test_mixed_waits_for_first_scan_then_times_out():
    loc = Localizer('mixed', scan_timeout=0.5)
    assert loc.on_wheels(0, 0.0, 0.0) is None
    assert loc.on_wheels(SEC // 10, 0.0, 0.0) is None
    assert not loc.started
    refs = loc.on_wheels(SEC, 0.0, 0.0)
    assert refs[0].points is None and loc.started


def test_mixed_starts_with_first_scan_and_needs_wheels():
    loc = Localizer('mixed')
    assert loc.on_scan(1, raycast((0, 0, 0))) is None   # no wheels yet
    loc.on_wheels(2, 0.0, 0.0)
    refs = loc.on_scan(3, raycast((0, 0, 0)))
    assert refs[0].stamp == 3 and refs[0].points is not None


def test_mixed_late_scans_never_make_references_go_back_in_time():
    # Scans arrive later than the timeout: wheel references are created while
    # a scan stamped before them is still in flight.
    loc = Localizer('mixed', scan_timeout=0.15)
    truth, refs = drive(loc, ARC, scan_hz=5, scan_latency=0.18)
    stamps = [r.stamp for r in refs]
    assert stamps == sorted(stamps) and len(set(stamps)) == len(stamps)
    assert any(r.points is None for r in refs)
    assert_pose(loc.odom_pose, truth, pos=0.05, ang=0.03)


@pytest.mark.parametrize('mode', ['encoders', 'lidar', 'mixed'])
def test_reference_stamps_increase(mode):
    loc = Localizer(mode)
    _, refs = drive(loc, ARC, scans=mode != 'encoders')
    stamps = [r.stamp for r in refs]
    assert stamps == sorted(stamps) and len(set(stamps)) == len(stamps)
    assert loc.ref_stamp == stamps[-1]


def test_mixed_reference_pose_is_at_scan_stamp():
    # Scans arrive late; the reference must be where the robot was at the scan stamp.
    loc = Localizer('mixed')
    truth, refs = drive(loc, [(0.2, 0.0, 4.0)], scan_latency=0.08)
    for ref in refs:
        t = (ref.stamp - 5 * SEC) / SEC
        assert ref.pose[0] == pytest.approx(0.2 * t, abs=0.01)
