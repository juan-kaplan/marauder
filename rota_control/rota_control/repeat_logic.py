"""
Repeat without ROS: follow the chain of keyframes of the map.

The state is k, the keyframe being followed, and rel, the robot pose in its
frame. On each scan rel moves with the odom delta and is relocalized by ICP
against keyframe k; k advances when the robot is near keyframe k+1 or past it.
The setpoint is keyframe k+1 seen from the robot, and the identity once the
last keyframe is reached.
"""

import math

from rota_common import icp, se2


class RepeatTracker:

    def __init__(self, keyframes, use_map_icp=True, advance_dist=0.20, advance_angle=0.15,
                 goal_tolerance=0.08, goal_angle_tolerance=0.15, icp_max_distance=0.30,
                 icp_max_iterations=30, icp_min_fitness=0.25):
        if not keyframes:
            raise ValueError('the map has no keyframes')
        self.edges = [kf.edge_from_prev for kf in keyframes]  # edges[0] is unused
        self.points = [kf.points for kf in keyframes]
        self.use_map_icp = use_map_icp
        self.advance_dist = advance_dist
        self.advance_angle = advance_angle
        self.goal_tolerance = goal_tolerance
        self.goal_angle_tolerance = goal_angle_tolerance
        self.icp_max_distance = icp_max_distance
        self.icp_max_iterations = icp_max_iterations
        self.icp_min_fitness = icp_min_fitness

        self.k = 0                # the robot starts where the Teach started
        self.rel = se2.IDENTITY   # keyframe k -> robot, at the latest scan
        self.odom = None          # odom -> robot, at the latest scan
        self.done = False
        self.last_icp = None      # (pose, fitness, ok), None if the latest scan had no ICP

    @property
    def num_keyframes(self):
        return len(self.edges)

    @property
    def started(self):
        return self.odom is not None

    @property
    def is_last(self):
        return self.k == self.num_keyframes - 1

    @property
    def next_edge(self):
        return None if self.is_last else self.edges[self.k + 1]

    def on_scan(self, odom, points):
        """odom: odom -> robot at the scan stamp; points: the scan in robot_frame, or None."""
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

        while not self.is_last and self.reached_next():
            self.rel = se2.between(self.edges[self.k + 1], self.rel)
            self.k += 1

    def reached_next(self):
        edge = self.edges[self.k + 1]
        next_robot = se2.between(edge, self.rel)
        if se2.norm(next_robot) >= self.advance_dist and next_robot[0] < 0.0:
            return False
        if se2.norm(edge) >= self.advance_dist:
            return True
        # Keyframe k+1 is a turn in place, so its position is met from the start:
        # also wait until the robot has turned to within advance_angle of it, or past it.
        return next_robot[2] * math.copysign(1.0, edge[2]) > -self.advance_angle

    def pose(self, odom_now):
        """Keyframe k -> robot now, extrapolated from the latest scan with odom."""
        if self.odom is None:
            raise RuntimeError('no scan yet')
        return se2.compose(self.rel, se2.between(self.odom, odom_now))

    def setpoint(self, odom_now):
        if self.done:
            return se2.IDENTITY
        target = se2.IDENTITY if self.is_last else self.edges[self.k + 1]
        setpoint = se2.between(self.pose(odom_now), target)
        if (self.is_last and se2.norm(setpoint) < self.goal_tolerance
                and abs(setpoint[2]) < self.goal_angle_tolerance):
            self.done = True
            return se2.IDENTITY
        return setpoint
