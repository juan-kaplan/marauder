"""
Repeat side of Teach and Repeat, without ROS.

It follows the chain of taught keyframes (see ``docs/tp1/arquitectura.md``).
The state is ``k``, the keyframe being followed, and ``T_kf_robot``, the robot
pose in the frame of keyframe ``k``. On each scan:

* the guess is ``T_kf_robot ⊕ Δodom``, where ``Δodom`` is the odom motion since
  the previous scan. Only odom deltas are used, so the origin of odom does not
  matter;
* with ``use_map_icp``, ICP of the scan against the scan of keyframe ``k``
  replaces the guess when it reaches ``min_fitness``;
* while the robot is within ``advance_dist`` of keyframe ``k+1``, or already
  past it, the state moves to the frame of ``k+1`` without relocalizing. If
  ``k+1`` is mostly a turn (less than ``advance_dist`` from ``k``), the robot
  must also have turned to within ``advance_angle`` of its heading, or beyond.
  Otherwise the keyframes of a turn in place, which share a position, would
  all be skipped at once.

Between scans the pose is extrapolated with odom. The setpoint is keyframe
``k+1`` seen from the robot, or keyframe ``k`` if it is the last one; once the
robot is within the goal tolerances of the last keyframe it is the identity
from then on, which makes the controller stop.
"""

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from rota_common import icp, se2
from rota_common.tnr_map import Keyframe, Pose2D


class RepeatTracker:

    def __init__(self, keyframes: Sequence[Keyframe], use_map_icp: bool = True,
                 advance_dist: float = 0.20, advance_angle: float = 0.15,
                 goal_tolerance: float = 0.08, goal_angle_tolerance: float = 0.15,
                 icp_max_distance: float = 0.30,
                 icp_max_iterations: int = 30, icp_min_fitness: float = 0.25):
        if not keyframes:
            raise ValueError('the map has no keyframes')
        # edges[i] is keyframe i seen from keyframe i-1; edges[0] is unused.
        self.edges = [kf.edge_from_prev for kf in keyframes]
        self.points = [kf.points for kf in keyframes]
        self.use_map_icp = use_map_icp
        self.advance_dist = advance_dist
        self.advance_angle = advance_angle
        self.goal_tolerance = goal_tolerance
        self.goal_angle_tolerance = goal_angle_tolerance
        self.icp_max_distance = icp_max_distance
        self.icp_max_iterations = icp_max_iterations
        self.icp_min_fitness = icp_min_fitness

        # The robot is assumed to start where the Teach started.
        self.k = 0
        self.rel: Pose2D = se2.IDENTITY     # T_kf_robot at the latest scan
        self.odom: Optional[Pose2D] = None  # odom -> robot at the latest scan
        self.done = False
        # (pose, fitness, ok) of the latest ICP, None if the latest scan had none.
        self.last_icp: Optional[Tuple[Pose2D, float, bool]] = None

    @property
    def num_keyframes(self) -> int:
        return len(self.edges)

    @property
    def started(self) -> bool:
        return self.odom is not None

    @property
    def is_last(self) -> bool:
        return self.k == self.num_keyframes - 1

    @property
    def next_edge(self) -> Optional[Pose2D]:
        """Keyframe ``k+1`` seen from keyframe ``k``, or None on the last keyframe."""
        return None if self.is_last else self.edges[self.k + 1]

    def on_scan(self, odom: Sequence[float], points: Optional[np.ndarray]) -> None:
        """
        Update the estimate with a scan.

        ``odom`` is odom -> robot at the scan stamp and ``points`` the scan in
        the robot frame (None to skip ICP).
        """
        odom = tuple(float(v) for v in odom)
        delta = se2.IDENTITY if self.odom is None else se2.between(self.odom, odom)
        guess = se2.compose(self.rel, delta)
        self.odom = odom

        self.last_icp = None
        if self.use_map_icp and points is not None:
            pose, fitness, ok = icp.align(points, self.points[self.k], guess,
                                          self.icp_max_distance, self.icp_max_iterations,
                                          self.icp_min_fitness)
            self.last_icp = (pose, fitness, ok)
            self.rel = pose if ok else guess
        else:
            self.rel = guess

        while not self.is_last and self._reached_next():
            self.rel = se2.between(self.edges[self.k + 1], self.rel)
            self.k += 1

    def _reached_next(self) -> bool:
        edge = self.edges[self.k + 1]
        next_robot = se2.between(edge, self.rel)
        if se2.norm(next_robot) >= self.advance_dist and next_robot[0] < 0.0:
            return False
        if se2.norm(edge) >= self.advance_dist:
            return True
        # A turn in place: the position is met from the start, so wait until
        # the robot has turned up to advance_angle short of it, or past it.
        return next_robot[2] * math.copysign(1.0, edge[2]) > -self.advance_angle

    def pose(self, odom_now: Sequence[float]) -> Pose2D:
        """``T_kf_robot`` now: the latest scan estimate plus the odom motion since that scan."""
        if self.odom is None:
            raise RuntimeError('no scan yet')
        return se2.compose(self.rel, se2.between(self.odom, odom_now))

    def setpoint(self, odom_now: Sequence[float]) -> Pose2D:
        """Target pose in the robot frame; the identity once the goal is reached."""
        if self.done:
            return se2.IDENTITY
        target = se2.IDENTITY if self.is_last else self.edges[self.k + 1]
        setpoint = se2.between(self.pose(odom_now), target)
        if (self.is_last and se2.norm(setpoint) < self.goal_tolerance
                and abs(setpoint[2]) < self.goal_angle_tolerance):
            self.done = True
            return se2.IDENTITY
        return setpoint
