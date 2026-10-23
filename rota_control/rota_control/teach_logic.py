"""
Teach without ROS: each reference of the localizer becomes a keyframe.

Its scan is the buffered one closest in time to the reference (a reference
waits until a scan at or after its stamp arrives). Its edge is ICP against the
previous keyframe, guessed from the two reference poses; if ICP fails the guess
is kept. end_keyframe adds one at the last pose, since the Teach usually ends
between two references. Stamps are in seconds.
"""

from collections import deque

import numpy as np

from rota_common import icp, se2
from rota_common.tnr_map import Keyframe


class TeachRecorder:

    def __init__(self, scan_buffer=50, icp_max_distance=0.30, icp_max_iterations=30,
                 icp_min_fitness=0.25, end_min_dist=0.02, end_min_angle=0.02):
        self.icp_max_distance = icp_max_distance
        self.icp_max_iterations = icp_max_iterations
        self.icp_min_fitness = icp_min_fitness
        self.end_min_dist = end_min_dist
        self.end_min_angle = end_min_angle
        self.keyframes = []
        self.scan_offsets = []                  # seconds from each keyframe to its scan, or None
        self.scans = deque(maxlen=scan_buffer)  # (stamp, points)
        self.pending = deque()                  # references waiting for a scan

    def on_scan(self, stamp, points):
        """Return the keyframes this scan completes."""
        if self.scans and stamp <= self.scans[-1][0]:
            return []
        self.scans.append((float(stamp), np.asarray(points, dtype=np.float64).reshape(-1, 2)))
        return self.resolve(final=False)

    def on_reference(self, stamp, pose):
        """Return the keyframes that are complete after this reference."""
        self.pending.append((float(stamp), tuple(float(v) for v in pose)))
        return self.resolve(final=False)

    def flush(self):
        """Complete the waiting references with the closest scan there is."""
        return self.resolve(final=True)

    def end_keyframe(self, stamp, pose):
        """Keyframe at the last pose, not added; None if the robot is still at the last one."""
        if not self.keyframes:
            return None
        pose = tuple(float(v) for v in pose)
        d = se2.between(self.keyframes[-1].pose_odom, pose)
        if se2.norm(d) < self.end_min_dist and abs(d[2]) < self.end_min_angle:
            return None
        return self.make(float(stamp), pose, self.closest_scan(stamp))[0]

    def resolve(self, final):
        done = []
        while self.pending:
            stamp, pose = self.pending[0]
            if not final and (not self.scans or self.scans[-1][0] < stamp):
                break
            self.pending.popleft()
            kf, offset = self.make(stamp, pose, self.closest_scan(stamp))
            self.keyframes.append(kf)
            self.scan_offsets.append(offset)
            done.append(kf)
        return done

    def closest_scan(self, stamp):
        if not self.scans:
            return None
        return min(self.scans, key=lambda s: abs(s[0] - stamp))

    def make(self, stamp, pose, scan):
        points = scan[1] if scan is not None else np.empty((0, 2))
        kf = Keyframe(len(self.keyframes), stamp, pose, points=points)
        if self.keyframes:
            prev = self.keyframes[-1]
            guess = se2.between(prev.pose_odom, pose)
            edge, fitness, ok = icp.align(points, prev.points, guess, self.icp_max_distance,
                                          self.icp_max_iterations, self.icp_min_fitness)
            kf.edge_from_prev = edge if ok else guess
            kf.edge_fitness = fitness
            kf.edge_icp_ok = ok
        return kf, None if scan is None else scan[0] - stamp
