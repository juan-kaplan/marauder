#!/usr/bin/env python3
"""
Repeat phase of Teach and Repeat.

Follows the chain of keyframes in map_dir. The state is k, the keyframe being
followed, and rel, the robot pose in its frame. On each scan rel moves with the
odom delta and is relocalized by ICP against keyframe k; k advances when the
robot is near keyframe k+1 or past it. Publishes /repeat/setpoint (keyframe k+1 seen
from the robot, the last keyframe itself at the end, the identity when the localizer's
TF is lost) and TF robot_frame -> map_current_keyframe -> map_next_keyframe and
robot_frame -> setpoint. The controller stops the robot at the last keyframe.
"""

import math
import os
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rota_common import icp, se2, tnr_map
from rota_common.msgs import pose_from_transform, pose_stamped, transform_stamped
from rota_common.scan import project

TF_WAIT = 0.2     # a queued scan is dropped if its TF has not arrived after this
TF_MAX_AGE = 0.5  # stop if the newest odom -> robot_frame TF is older than this


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RepeatNode(Node):

    def __init__(self):
        super().__init__('repeat_node')

        self.declare_parameter('map_dir', 'data/map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('advance_dist', 0.20)
        self.declare_parameter('advance_angle', 0.15)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('icp_max_distance', 0.30)
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_min_fitness', 0.25)

        self.map_dir = os.path.abspath(os.path.expanduser(self.get_parameter('map_dir').value))
        self.robot_frame = self.get_parameter('robot_frame').value
        self.advance_dist = self.get_parameter('advance_dist').value
        self.advance_angle = self.get_parameter('advance_angle').value
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.icp_args = (self.get_parameter('icp_max_distance').value,
                         self.get_parameter('icp_max_iterations').value,
                         self.get_parameter('icp_min_fitness').value)

        meta, keyframes = tnr_map.load(self.map_dir)
        if meta.get('robot_frame', self.robot_frame) != self.robot_frame:
            self.get_logger().warn(f"Map taught with robot_frame={meta['robot_frame']}, "
                                   f"repeating with {self.robot_frame}")
        self.taught = tnr_map.chain_poses(keyframes)
        self.edges = [kf.edge_from_prev for kf in keyframes]  # edges[0] is unused
        self.points = [kf.points for kf in keyframes]
        self.last = len(keyframes) - 1
        self.get_logger().info(f"Loaded {len(keyframes)} keyframes from {self.map_dir} "
                               f"(mode {meta.get('mode', '?')})")

        self.k = 0                # the robot starts where the Teach started
        self.rel = se2.IDENTITY   # keyframe k -> robot, at the latest scan
        self.odom = None          # odom -> robot, at the latest scan
        self.scans = deque(maxlen=10)  # (stamp, points) waiting for their TF
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'
        self.last_path_pose = None

        # Scans are not processed in on_scan: waiting there for their TF starves the
        # listener. control() takes the ones whose TF is already in the buffer.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_setpoint = self.create_publisher(PoseStamped, '/repeat/setpoint', 10)
        self.pub_taught = self.create_publisher(Path, '/repeat/taught_path', 10)
        self.pub_actual = self.create_publisher(Path, '/repeat/actual_path', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter('control_frequency').value, self.control)

    def on_scan(self, msg):
        points = project(msg, self.tf_buffer, self.robot_frame, self.min_range, self.max_range)
        if points is None:
            self.get_logger().warn(f"No TF {self.robot_frame} -> {msg.header.frame_id} yet",
                                   throttle_duration_sec=2.0)
            return
        self.scans.append((msg.header.stamp, points))

    def process_scans(self):
        now = seconds(self.get_clock().now().to_msg())
        while self.scans:
            stamp, points = self.scans[0]
            try:
                tf = self.tf_buffer.lookup_transform('odom', self.robot_frame,
                                                     Time.from_msg(stamp))
            except TransformException as ex:
                if now - seconds(stamp) < TF_WAIT:
                    return
                self.scans.popleft()
                self.get_logger().warn(f"Dropping a scan, no TF odom -> {self.robot_frame} "
                                       f"at its stamp: {ex}", throttle_duration_sec=2.0)
                continue
            self.scans.popleft()
            self.update(pose_from_transform(tf), points)

    def update(self, odom, points):
        """Move rel with the odom delta, relocalize it against keyframe k, advance k."""
        delta = se2.IDENTITY if self.odom is None else se2.between(self.odom, odom)
        guess = se2.compose(self.rel, delta)
        self.odom = odom
        self.rel = guess
        pose, fitness, ok = icp.align(points, self.points[self.k], guess, *self.icp_args)
        if ok:
            self.rel = pose
        else:
            self.get_logger().warn(f"ICP against keyframe #{self.k} failed "
                                   f"(fitness {fitness:.2f}), using odom",
                                   throttle_duration_sec=2.0)
        while self.k < self.last and self.reached_next():
            self.rel = se2.between(self.edges[self.k + 1], self.rel)
            self.k += 1
            self.get_logger().info(f"Following keyframe #{self.k} of {self.last}")

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

    def setpoint(self, odom_now):
        """Keyframe k+1 (or the end of the map) as seen from the robot now."""
        pose = se2.compose(self.rel, se2.between(self.odom, odom_now))  # keyframe k -> robot
        target = self.edges[self.k + 1] if self.k < self.last else se2.IDENTITY
        setpoint = se2.between(pose, target)
        return pose, setpoint

    def latest_odom(self):
        """Newest odom -> robot_frame pose and stamp, None if missing or too old."""
        try:
            tf = self.tf_buffer.lookup_transform('odom', self.robot_frame, Time())
        except TransformException:
            return None
        if seconds(self.get_clock().now().to_msg()) - seconds(tf.header.stamp) > TF_MAX_AGE:
            return None
        return pose_from_transform(tf), tf.header.stamp

    def control(self):
        self.process_scans()
        if self.odom is None:
            self.get_logger().info("Waiting for the first scan and the localizer's TF",
                                   throttle_duration_sec=5.0)
            return
        latest = self.latest_odom()
        if latest is None:
            self.get_logger().warn(f"Lost the TF odom -> {self.robot_frame}, stopping",
                                   throttle_duration_sec=2.0)
            self.pub_setpoint.publish(pose_stamped(self.get_clock().now().to_msg(),
                                                   self.robot_frame, se2.IDENTITY))
            return
        odom, stamp = latest

        pose, setpoint = self.setpoint(odom)
        self.tf_broadcaster.sendTransform([
            transform_stamped(stamp, self.robot_frame, 'map_current_keyframe',
                              se2.inverse(pose)),
            transform_stamped(stamp, 'map_current_keyframe', 'map_next_keyframe',
                              self.edges[self.k + 1] if self.k < self.last else se2.IDENTITY),
            transform_stamped(stamp, self.robot_frame, 'setpoint', setpoint)])
        self.pub_setpoint.publish(pose_stamped(stamp, self.robot_frame, setpoint))
        self.publish_paths(stamp, odom)

    def publish_paths(self, stamp, odom):
        current = self.taught[self.k]
        taught = Path()
        taught.header.stamp = stamp
        taught.header.frame_id = 'map_current_keyframe'
        taught.poses = [pose_stamped(stamp, 'map_current_keyframe', se2.between(current, p))
                        for p in self.taught]
        self.pub_taught.publish(taught)

        if self.last_path_pose is not None:
            d = se2.between(self.last_path_pose, odom)
            if se2.norm(d) < 0.02 and abs(d[2]) < 0.05:
                return
        self.last_path_pose = odom
        self.actual_path.header.stamp = stamp
        self.actual_path.poses.append(pose_stamped(stamp, 'odom', odom))
        self.pub_actual.publish(self.actual_path)


def main(args=None):
    rclpy.init(args=args)
    node = RepeatNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
