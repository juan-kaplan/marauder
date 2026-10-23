"""Tiny TB3 simulator for the localizer tests: wheel angles and ray-cast scans."""

import math

import numpy as np

from rota_common import se2
from rota_localization.localizer import integrate_diff_drive

R = 0.033   # wheel radius
B = 0.160   # wheel separation
SEC = 1_000_000_000

# A 6 x 4 m room with a few boxes, so ICP has corners to lock onto. The test
# trajectories stay clear of them.
ROOM = np.array([
    [(-1.0, -2.0), (5.0, -2.0)], [(5.0, -2.0), (5.0, 2.0)], [(5.0, 2.0), (-1.0, 2.0)],
    [(-1.0, 2.0), (-1.0, -2.0)],
    [(3.8, 1.2), (4.4, 1.2)], [(3.8, 1.2), (3.8, 2.0)],
    [(3.0, -2.0), (3.0, -1.4)], [(3.0, -1.4), (3.5, -1.4)],
    [(0.2, -1.0), (0.5, -1.0)], [(0.5, -1.0), (0.5, -0.7)],
])

# A 1.5 m wide corridor, much longer than the LiDAR range: nothing tells ICP
# how far the robot moved along it.
CORRIDOR = np.array([
    [(-50.0, -0.75), (50.0, -0.75)], [(-50.0, 0.75), (50.0, 0.75)],
])


def raycast(pose, segments=ROOM, beams=360, max_range=3.5, noise=0.0, rng=None):
    """Points in the robot frame hit by a 360-beam LiDAR at ``pose``; ``noise`` is the range std in m."""
    x, y, th = pose
    a = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    d = np.stack([np.cos(th + a), np.sin(th + a)], axis=1)          # (N, 2)
    p, e = segments[:, 0], segments[:, 1] - segments[:, 0]           # (M, 2)
    w = p - np.array([x, y])
    den = d[:, None, 0] * -e[None, :, 1] - d[:, None, 1] * -e[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (w[None, :, 0] * -e[None, :, 1] - w[None, :, 1] * -e[None, :, 0]) / den
        u = (d[:, None, 0] * w[None, :, 1] - d[:, None, 1] * w[None, :, 0]) / den
    s = np.where((np.abs(den) > 1e-12) & (s > 0.12) & (u >= 0) & (u <= 1), s, np.inf)
    r = s.min(axis=1)
    keep = r < max_range
    r, a = r[keep], a[keep]
    if noise > 0.0:
        r = r + rng.normal(0.0, noise, r.shape)
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def drive(loc, commands, wheel_hz=30, scan_hz=10, scan_latency=0.02, scans=True,
          scan_gap=None, start=(0.0, 0.0, 0.0), t0=5 * SEC, segments=ROOM,
          scan_noise=0.0, seed=0, trace=None):
    """
    Feed ``loc`` wheels and scans for a list of ``(v, omega, seconds)``.

    Returns the ground truth pose at the end and all references created, in
    order. ``scan_gap=(a, b)`` drops the scans stamped between a and b seconds.
    If ``trace`` is a list, ``(truth, estimate)`` pairs at the estimate's stamp
    are appended to it after every update, both relative to the start.
    """
    rng = np.random.default_rng(seed)
    truth = start
    left = right = 0.0
    refs = []
    events = []
    truths = {t0: truth}
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
            truths[stamp] = truth
            events.append((stamp, 'wheels', (left, right)))
            if scans and t >= next_scan - 1e-9:
                next_scan += 1.0 / scan_hz
                if scan_gap is None or not (scan_gap[0] <= t <= scan_gap[1]):
                    arrival = stamp + int(scan_latency * SEC)
                    pts = raycast(truth, segments, noise=scan_noise, rng=rng)
                    events.append((arrival, 'scan', (stamp, pts)))
    events.sort(key=lambda e: e[0])
    origin = None
    for arrival, kind, data in events:
        if kind == 'wheels':
            out = loc.on_wheels(arrival, *data)
        else:
            out = loc.on_scan(*data)
        if out:
            refs.extend(out)
        if trace is not None and out is not None and loc.started:
            now = truths[loc.stamp]
            if origin is None:
                # The estimate's origin is wherever the robot was when it started.
                origin = se2.compose(truths[loc.ref_stamp], se2.inverse(loc.ref_pose))
            trace.append((se2.between(origin, now), loc.odom_pose))
    return truth, refs
