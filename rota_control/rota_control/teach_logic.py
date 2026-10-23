"""
Teach side of Teach and Repeat, without ROS.

Every reference of the localizer becomes a keyframe (see
``docs/tp1/arquitectura.md``):

* its scan is the buffered scan closest in time to the reference stamp. A
  reference waits until a scan at or after its stamp has arrived, so that the
  closest one is known; in lidar and mixed mode that scan is the very one that
  created the reference;
* its edge from the previous keyframe is ICP of its scan against the previous
  keyframe's scan, with the odom poses of the two references as the guess. If
  ICP fails, or a keyframe has no points, the guess is kept and
  ``edge_icp_ok`` is False.

References only come every ``tau_d`` or ``tau_theta``, so the Teach usually
ends between two of them. ``end_keyframe`` builds one more keyframe at the
last pose of the localizer, for the map that is saved, so that the end of the
path is not lost.

Stamps are seconds as floats.
"""

from collections import deque
from typing import List, Optional, Sequence, Tuple

import numpy as np

from rota_common import icp, se2
from rota_common.tnr_map import Keyframe, Pose2D


class TeachRecorder:

    def __init__(self, scan_buffer: int = 50, icp_max_distance: float = 0.30,
                 icp_max_iterations: int = 30, icp_min_fitness: float = 0.25,
                 end_min_dist: float = 0.02, end_min_angle: float = 0.02):
        self.icp_max_distance = icp_max_distance
        self.icp_max_iterations = icp_max_iterations
        self.icp_min_fitness = icp_min_fitness
        # The end keyframe is only made if the robot moved this much since the last one.
        self.end_min_dist = end_min_dist
        self.end_min_angle = end_min_angle
        self.keyframes: List[Keyframe] = []
        # Seconds from each keyframe to the scan it got, None if it got none.
        self.scan_offsets: List[Optional[float]] = []
        self.scans: deque = deque(maxlen=scan_buffer)  # (stamp, points), oldest first
        self.pending: deque = deque()                  # references waiting for a scan

    def on_scan(self, stamp: float, points: np.ndarray) -> List[Keyframe]:
        """Buffer a scan (points in the robot frame); return the keyframes it completes."""
        if self.scans and stamp <= self.scans[-1][0]:
            return []  # out of order or repeated
        self.scans.append((float(stamp), np.asarray(points, dtype=np.float64).reshape(-1, 2)))
        return self._resolve(final=False)

    def on_reference(self, stamp: float, pose: Sequence[float]) -> List[Keyframe]:
        """Queue a reference (its pose in odom); return the keyframes that are now complete."""
        self.pending.append((float(stamp), tuple(float(v) for v in pose)))
        return self._resolve(final=False)

    def flush(self) -> List[Keyframe]:
        """Complete every waiting reference with the closest scan there is (or none)."""
        return self._resolve(final=True)

    def end_keyframe(self, stamp: float, pose: Sequence[float]) -> Optional[Keyframe]:
        """
        Build a keyframe at the localizer's last pose, without adding it.

        It gets the closest buffered scan and an ICP edge from the last keyframe,
        like any other. None if there are no keyframes yet, or if the robot is
        within ``end_min_dist`` and ``end_min_angle`` of the last one.
        """
        if not self.keyframes:
            return None
        pose = tuple(float(v) for v in pose)
        d = se2.between(self.keyframes[-1].pose_odom, pose)
        if se2.norm(d) < self.end_min_dist and abs(d[2]) < self.end_min_angle:
            return None
        kf, _ = self._make(float(stamp), pose, self._closest_scan(stamp))
        return kf

    def _resolve(self, final: bool) -> List[Keyframe]:
        done = []
        while self.pending:
            stamp, pose = self.pending[0]
            if not final and (not self.scans or self.scans[-1][0] < stamp):
                break  # a closer scan may still come
            self.pending.popleft()
            kf, offset = self._make(stamp, pose, self._closest_scan(stamp))
            self.keyframes.append(kf)
            self.scan_offsets.append(offset)
            done.append(kf)
        return done

    def _closest_scan(self, stamp: float) -> Optional[Tuple[float, np.ndarray]]:
        if not self.scans:
            return None
        return min(self.scans, key=lambda s: abs(s[0] - stamp))

    def _make(self, stamp: float, pose: Pose2D,
              scan: Optional[Tuple[float, np.ndarray]]) -> Tuple[Keyframe, Optional[float]]:
        """Next keyframe after the last one, and the seconds to its scan (None if no scan)."""
        points = scan[1] if scan is not None else np.empty((0, 2))
        kf = Keyframe(id=len(self.keyframes), stamp=stamp, pose_odom=pose, points=points)
        if self.keyframes:
            prev = self.keyframes[-1]
            guess = se2.between(prev.pose_odom, pose)
            edge, fitness, ok = icp.align(points, prev.points, guess, self.icp_max_distance,
                                          self.icp_max_iterations, self.icp_min_fitness)
            kf.edge_from_prev = edge if ok else guess
            kf.edge_fitness = fitness
            kf.edge_icp_ok = ok
        return kf, None if scan is None else scan[0] - stamp
