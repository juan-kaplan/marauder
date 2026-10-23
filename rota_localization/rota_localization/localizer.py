"""
Layer 1 of the Teach and Repeat stack, without ROS: how far the robot moved
since its last reference (see ``docs/tp1/arquitectura.md``).

The estimate is ``T_odom_ref ⊕ T_ref_robot``. When ``T_ref_robot`` passes
``tau_d`` or ``tau_theta`` a new reference is created where the robot is, so
the odom -> robot pose never jumps.

Modes:

* ``encoders``: integrate the wheel deltas.
* ``lidar``: ICP of each scan against the reference scan, with a constant
  velocity guess.
* ``mixed``: integrate the wheels between scans and refine with ICP against
  the reference scan on each scan. After ``scan_timeout`` without scans,
  references are created from the wheels alone, and the next scan creates a
  reference that carries a scan again.

Stamps are integer nanoseconds, so references keep the exact stamp of the
sensor message that created them.
"""

import bisect
import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from rota_common import icp, se2
from rota_common.se2 import Pose2D

MODES = ('encoders', 'lidar', 'mixed')
NS = 1e-9

Twist2D = Tuple[float, float, float]  # (vx, vy, omega) in the robot frame
ZERO_TWIST: Twist2D = (0.0, 0.0, 0.0)


@dataclass
class Reference:
    stamp: int                     # ns, stamp of the message that created it
    pose: Pose2D                   # T_odom_ref
    points: Optional[np.ndarray]   # reference scan in the robot frame, None if made from wheels


def integrate_diff_drive(pose: Sequence[float], d_left: float, d_right: float,
                         wheel_separation: float) -> Pose2D:
    """One differential-drive step (midpoint rule); ``d_*`` are wheel distances in m."""
    x, y, theta = pose
    d_lin = 0.5 * (d_right + d_left)
    d_ang = (d_right - d_left) / wheel_separation
    return (x + d_lin * math.cos(theta + 0.5 * d_ang),
            y + d_lin * math.sin(theta + 0.5 * d_ang),
            se2.normalize_angle(theta + d_ang))


def interpolate(a: Sequence[float], b: Sequence[float], alpha: float) -> Pose2D:
    return (a[0] + alpha * (b[0] - a[0]),
            a[1] + alpha * (b[1] - a[1]),
            se2.normalize_angle(a[2] + alpha * se2.normalize_angle(b[2] - a[2])))


def twist_between(a: Sequence[float], b: Sequence[float], dt: float) -> Twist2D:
    """Constant body twist that takes pose ``a`` to pose ``b`` in ``dt`` seconds."""
    if dt <= 0.0:
        return ZERO_TWIST
    d = se2.between(a, b)
    return (d[0] / dt, d[1] / dt, d[2] / dt)


def advance(pose: Sequence[float], twist: Sequence[float], dt: float) -> Pose2D:
    return se2.compose(pose, (twist[0] * dt, twist[1] * dt, twist[2] * dt))


class EncoderNoise:
    """
    Simulated wheel encoder errors.

    ``std``: random error per wheel, in m per sqrt(m) travelled (the variance
    grows with the distance, not with the message rate). ``bias``: relative
    difference between the wheel radii; positive makes the robot believe it
    turns left.
    """

    def __init__(self, std: float = 0.0, bias: float = 0.0, seed: Optional[int] = None):
        self.std = std
        self.bias = bias
        self._rng = np.random.default_rng(seed)

    @property
    def enabled(self) -> bool:
        return self.std > 0.0 or self.bias != 0.0

    def __call__(self, d_left: float, d_right: float) -> Tuple[float, float]:
        d_left *= 1.0 - 0.5 * self.bias
        d_right *= 1.0 + 0.5 * self.bias
        if self.std > 0.0:
            d_left += self._rng.normal(0.0, self.std * math.sqrt(abs(d_left)))
            d_right += self._rng.normal(0.0, self.std * math.sqrt(abs(d_right)))
        return d_left, d_right


class Localizer:

    def __init__(self, mode: str, tau_d: float = 0.30, tau_theta: float = 0.30,
                 wheel_radius: float = 0.033, wheel_separation: float = 0.160,
                 icp_params: icp.IcpParams = icp.IcpParams(), scan_timeout: float = 0.5,
                 encoder_noise: Optional[EncoderNoise] = None):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got '{mode}'")
        self.mode = mode
        self.tau_d = tau_d
        self.tau_theta = tau_theta
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.icp_params = icp_params
        self.scan_timeout_ns = int(scan_timeout * 1e9)
        self.encoder_noise = encoder_noise or EncoderNoise()

        # Estimate, valid once started.
        self.stamp: Optional[int] = None    # ns of the latest update
        self.ref_pose: Pose2D = se2.IDENTITY  # T_odom_ref
        self.rel: Pose2D = se2.IDENTITY       # T_ref_robot at self.stamp
        self.twist: Twist2D = ZERO_TWIST
        self.ref_points: Optional[np.ndarray] = None
        self.ref_stamp: Optional[int] = None
        self.num_references = 0
        self.last_icp: Optional[icp.IcpResult] = None

        # Wheels: last joint angles and the raw integrated wheel odometry,
        # whose history lets mixed mode find the wheel pose at a scan stamp.
        self._last_joints: Optional[Tuple[int, float, float]] = None
        self._wheels: deque = deque(maxlen=300)  # (stamp, pose)
        self._first_wheel_stamp: Optional[int] = None
        self._last_scan_stamp: Optional[int] = None

    @property
    def started(self) -> bool:
        return self.stamp is not None

    @property
    def odom_pose(self) -> Pose2D:
        """T_odom_robot."""
        return se2.compose(self.ref_pose, self.rel)

    # ------------------------------------------------------------------ inputs

    def on_wheels(self, stamp: int, left: float, right: float) -> Optional[List[Reference]]:
        """
        Wheel joint angles (rad). Returns ``None`` if the estimate did not
        change, otherwise the references created (usually none).
        """
        if self.mode == 'lidar':
            return None
        if self._last_joints is None:
            self._last_joints = (stamp, left, right)
            self._wheels.append((stamp, se2.IDENTITY))
            self._first_wheel_stamp = stamp
            if self.mode == 'encoders':
                return [self._start(stamp, None)]
            return None

        last_stamp, last_left, last_right = self._last_joints
        if stamp <= last_stamp:
            return None
        d_left, d_right = self.encoder_noise(self.wheel_radius * (left - last_left),
                                             self.wheel_radius * (right - last_right))
        self._last_joints = (stamp, left, right)
        prev = self._wheels[-1][1]
        wheel_pose = integrate_diff_drive(prev, d_left, d_right, self.wheel_separation)
        self._wheels.append((stamp, wheel_pose))
        delta = se2.between(prev, wheel_pose)
        twist = twist_between(prev, wheel_pose, (stamp - last_stamp) * NS)

        if not self.started:
            # mixed, still waiting for the first scan
            if stamp - self._first_wheel_stamp <= self.scan_timeout_ns:
                return None
            return [self._start(stamp, None)]

        self.rel = se2.compose(self.rel, delta)
        self.twist = twist
        self.stamp = max(self.stamp, stamp)
        if self.mode == 'mixed' and not self._scan_timed_out(stamp):
            return []  # scans decide when to create references
        return self._maybe_new_reference(stamp, None)

    def on_scan(self, stamp: int, points: np.ndarray) -> Optional[List[Reference]]:
        """A scan in the robot frame. Same return convention as ``on_wheels``."""
        if self.mode == 'encoders':
            return None
        self.last_icp = None
        if self.mode == 'lidar':
            return self._on_scan_lidar(stamp, points)
        return self._on_scan_mixed(stamp, points)

    # ------------------------------------------------------------------- modes

    def _on_scan_lidar(self, stamp, points):
        if not self.started:
            return [self._start(stamp, points)]
        dt = (stamp - self.stamp) * NS
        if dt <= 0.0:
            return None
        guess = advance(self.rel, self.twist, dt)
        rel = self._match(points, guess)
        self.twist = twist_between(self.rel, rel, dt)
        self.rel = rel
        self.stamp = stamp
        self._last_scan_stamp = stamp
        return self._maybe_new_reference(stamp, points)

    def _on_scan_mixed(self, stamp, points):
        if not self._wheels:
            return None  # wait for the wheels
        if self.started and stamp <= self.ref_stamp:
            # Older than the current reference (made from the wheels while this
            # scan was in flight): references must move forward in time.
            return None
        wheel_now = self._wheels[-1][1]
        wheel_at_scan = self._wheel_pose_at(stamp)
        since_scan = se2.between(wheel_at_scan, wheel_now)  # motion from the scan to now
        self._last_scan_stamp = stamp

        if not self.started:
            ref = self._start(stamp, points)
            self.rel = since_scan
            self.stamp = max(stamp, self._wheels[-1][0])
            return [ref]

        # Estimate at the scan stamp, from the latest estimate and the wheels.
        rel_at_scan = se2.compose(self.rel, se2.inverse(since_scan))
        if self.ref_points is not None:
            rel_at_scan = self._match(points, rel_at_scan)
        refs = []
        # A reference made from the wheels alone is replaced by one with this scan.
        if self.ref_points is None or self._exceeds(rel_at_scan):
            refs.append(self._new_reference(stamp, rel_at_scan, points))
            rel_at_scan = se2.IDENTITY
        self.rel = se2.compose(rel_at_scan, since_scan)
        self.stamp = max(self.stamp, stamp)
        return refs

    # ----------------------------------------------------------------- helpers

    def _start(self, stamp, points) -> Reference:
        self.stamp = stamp
        self.ref_pose = se2.IDENTITY
        self.rel = se2.IDENTITY
        self.twist = ZERO_TWIST
        self.ref_points = points
        self.ref_stamp = stamp
        self.num_references = 1
        return Reference(stamp, self.ref_pose, points)

    def _match(self, points, guess) -> Pose2D:
        r = icp.align(points, self.ref_points, guess, self.icp_params)
        self.last_icp = r
        return r.pose if r.ok else tuple(guess)

    def _exceeds(self, rel) -> bool:
        return se2.norm(rel) >= self.tau_d or abs(rel[2]) >= self.tau_theta

    def _maybe_new_reference(self, stamp, points) -> List[Reference]:
        """Create a reference where the robot is now if it passed the thresholds."""
        if not self._exceeds(self.rel):
            return []
        ref = self._new_reference(stamp, self.rel, points)
        self.rel = se2.IDENTITY
        return [ref]

    def _new_reference(self, stamp, rel, points) -> Reference:
        self.ref_pose = se2.compose(self.ref_pose, rel)
        self.ref_points = points
        self.ref_stamp = stamp
        self.num_references += 1
        return Reference(stamp, self.ref_pose, points)

    def _scan_timed_out(self, stamp) -> bool:
        last = self._last_scan_stamp if self._last_scan_stamp is not None else self._first_wheel_stamp
        return stamp - last > self.scan_timeout_ns

    def _wheel_pose_at(self, stamp) -> Pose2D:
        """Raw wheel odometry at ``stamp``, interpolated (clamped to the history)."""
        hist = self._wheels
        if stamp <= hist[0][0]:
            return hist[0][1]
        if stamp >= hist[-1][0]:
            return hist[-1][1]
        i = bisect.bisect_left([t for t, _ in hist], stamp)  # hist[i - 1] < stamp <= hist[i]
        (t0, p0), (t1, p1) = hist[i - 1], hist[i]
        return interpolate(p0, p1, (stamp - t0) / (t1 - t0))
