#!/usr/bin/env python3
"""
Repeat phase of Teach and Repeat.

Loads the map from map_dir and follows its chain of keyframes (see
repeat_logic). Each /scan is queued, and once the localizer's TF
odom -> robot_frame at its stamp is in the buffer, it relocalizes the robot
with ICP against the current keyframe. Nothing blocks waiting for a TF: the
queue is checked at control_frequency, which also publishes /repeat/setpoint, the next
keyframe in robot_frame (the identity once the goal is reached, or when the
pose is not known, so that diff_control_node stops), and the TFs
robot_frame -> map_current_keyframe -> map_next_keyframe and
robot_frame -> setpoint. Also /repeat/taught_path (in map_current_keyframe),
/repeat/actual_path (in odom) and /repeat/markers.
"""

import math
import os
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rota_common import se2, tnr_map
from rota_common.msgs import pose_from_transform, pose_stamped, transform_stamped
from rota_common.scan import scan_to_points
from rota_control.repeat_logic import RepeatTracker

# How long a queued scan waits for the localizer's TF at its stamp before it is dropped.
TF_WAIT = 0.2
# The robot pose is considered lost when the newest odom -> robot TF is older than this.
TF_MAX_AGE = 0.5


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RepeatNode(Node):

    def __init__(self):
        super().__init__('repeat_node')

        self.declare_parameter('map_dir', 'data/map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('use_map_icp', True)
        self.declare_parameter('advance_dist', 0.20)
        self.declare_parameter('advance_angle', 0.15)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('goal_angle_tolerance', 0.15)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('icp_max_distance', 0.30)
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_min_fitness', 0.25)

        self.map_dir = os.path.abspath(os.path.expanduser(self.get_parameter('map_dir').value))
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.active = self.get_parameter('auto_start').value

        meta, keyframes = tnr_map.load(self.map_dir)
        if meta.get('robot_frame', self.robot_frame) != self.robot_frame:
            self.get_logger().warn(f"The map was taught with robot_frame={meta['robot_frame']}, "
                                   f"repeating with {self.robot_frame}")
        self.taught = tnr_map.chain_poses(keyframes)
        self.tracker = RepeatTracker(
            keyframes,
            use_map_icp=self.get_parameter('use_map_icp').value,
            advance_dist=self.get_parameter('advance_dist').value,
            advance_angle=self.get_parameter('advance_angle').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value,
            goal_angle_tolerance=self.get_parameter('goal_angle_tolerance').value,
            icp_max_distance=self.get_parameter('icp_max_distance').value,
            icp_max_iterations=self.get_parameter('icp_max_iterations').value,
            icp_min_fitness=self.get_parameter('icp_min_fitness').value)
        self.get_logger().info(
            f"Loaded {len(keyframes)} keyframes from {self.map_dir} (taught in mode "
            f"{meta.get('mode', '?')}); use_map_icp={self.tracker.use_map_icp} "
            f"active={self.active}")

        self.mount = None      # robot_frame -> LiDAR, as (R, t)
        self.k_logged = 0
        self.done_logged = False
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'
        self.last_path_pose = None

        # Scans waiting for the TF at their stamp: (stamp, points in robot_frame).
        self.scans = deque(maxlen=10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_setpoint = self.create_publisher(PoseStamped, '/repeat/setpoint', 10)
        self.pub_taught = self.create_publisher(Path, '/repeat/taught_path', 10)
        self.pub_actual = self.create_publisher(Path, '/repeat/actual_path', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/repeat/markers', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(1.0 / self.get_parameter('control_frequency').value, self.control)

    def on_scan(self, msg):
        if not self.active:
            return
        points = self.project(msg)
        if points is not None:
            self.scans.append((msg.header.stamp, points))

    def process_scans(self):
        """Feed the tracker the queued scans whose odom -> robot_frame TF has arrived."""
        now = seconds(self.get_clock().now().to_msg())
        while self.scans:
            stamp, points = self.scans[0]
            try:
                tf = self.tf_buffer.lookup_transform('odom', self.robot_frame,
                                                     Time.from_msg(stamp))
            except TransformException as ex:
                if now - seconds(stamp) < TF_WAIT:
                    return  # the localizer has not published it yet
                self.scans.popleft()
                self.get_logger().warn(f"Dropping a scan, no TF odom -> {self.robot_frame} "
                                       f"at its stamp: {ex}", throttle_duration_sec=2.0)
                continue
            self.scans.popleft()
            self.tracker.on_scan(pose_from_transform(tf), points)
            if self.tracker.last_icp is not None and not self.tracker.last_icp[2]:
                self.get_logger().warn(
                    f"ICP against keyframe #{self.tracker.k} failed (fitness "
                    f"{self.tracker.last_icp[1]:.2f}), using odom", throttle_duration_sec=2.0)
            if self.tracker.k != self.k_logged:
                self.k_logged = self.tracker.k
                self.get_logger().info(f"Following keyframe #{self.tracker.k} of "
                                       f"{self.tracker.num_keyframes - 1}")

    def project(self, msg):
        """Scan points in robot_frame, using the static LiDAR mount."""
        if self.mount is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.robot_frame, msg.header.frame_id, Time())
            except TransformException as ex:
                self.get_logger().warn(
                    f"No TF {self.robot_frame} -> {msg.header.frame_id} yet: {ex}",
                    throttle_duration_sec=2.0)
                return None
            x, y, theta = pose_from_transform(tf)
            c, s = math.cos(theta), math.sin(theta)
            self.mount = (np.array([[c, -s], [s, c]]), np.array([x, y]))
        R, t = self.mount
        return scan_to_points(msg, self.min_range, self.max_range) @ R.T + t

    def latest_odom(self):
        """Newest odom -> robot pose and its stamp, or None if missing or too old."""
        try:
            tf = self.tf_buffer.lookup_transform('odom', self.robot_frame, Time())
        except TransformException:
            return None
        age = seconds(self.get_clock().now().to_msg()) - seconds(tf.header.stamp)
        if age > TF_MAX_AGE:
            return None
        return pose_from_transform(tf), tf.header.stamp

    def control(self):
        if not self.active:
            return
        self.process_scans()
        stamp = self.get_clock().now().to_msg()
        if not self.tracker.started:
            self.get_logger().info("Waiting for the first scan and the localizer's TF",
                                   throttle_duration_sec=5.0)
            return
        latest = self.latest_odom()
        if latest is None:
            self.get_logger().warn(f"Lost the TF odom -> {self.robot_frame}, stopping",
                                   throttle_duration_sec=2.0)
            self.pub_setpoint.publish(pose_stamped(stamp, self.robot_frame, se2.IDENTITY))
            return
        odom, odom_stamp = latest

        setpoint = self.tracker.setpoint(odom)
        kf_robot = self.tracker.pose(odom)
        next_edge = self.tracker.next_edge or se2.IDENTITY
        self.tf_broadcaster.sendTransform([
            transform_stamped(odom_stamp, self.robot_frame, 'map_current_keyframe',
                              se2.inverse(kf_robot)),
            transform_stamped(odom_stamp, 'map_current_keyframe', 'map_next_keyframe', next_edge),
            transform_stamped(odom_stamp, self.robot_frame, 'setpoint', setpoint)])
        self.pub_setpoint.publish(pose_stamped(odom_stamp, self.robot_frame, setpoint))
        if self.tracker.done and not self.done_logged:
            self.done_logged = True
            self.get_logger().info(f"Reached the last keyframe (#{self.tracker.k}), stopping")
        self.publish_paths(odom_stamp, odom)
        self.publish_markers(odom_stamp)

    def publish_paths(self, stamp, odom):
        # The taught chain seen from the current keyframe, so it moves with the relocalization.
        current = self.taught[self.tracker.k]
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

    def publish_markers(self, stamp):
        m = Marker()
        m.header.frame_id = self.robot_frame
        m.header.stamp = stamp
        m.ns = 'repeat_status'
        m.type = Marker.TEXT_VIEW_FACING
        m.pose.position.z = 0.4
        m.scale.z = 0.12
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f"KF {self.tracker.k}/{self.tracker.num_keyframes - 1}"
        if self.tracker.done:
            m.text += ' [DONE]'
        elif self.tracker.last_icp is not None and not self.tracker.last_icp[2]:
            m.text += ' [ODOM]'
        self.pub_markers.publish(MarkerArray(markers=[m]))

    def on_start(self, request, response):
        self.active = True
        response.success = True
        response.message = f"Repeat running, following keyframe #{self.tracker.k}"
        self.get_logger().info(response.message)
        return response

    def on_stop(self, request, response):
        self.active = False
        stamp = self.get_clock().now().to_msg()
        self.pub_setpoint.publish(pose_stamped(stamp, self.robot_frame, se2.IDENTITY))
        response.success = True
        response.message = f"Repeat stopped at keyframe #{self.tracker.k}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RepeatNode()
    except (FileNotFoundError, ValueError) as ex:
        rclpy.logging.get_logger('repeat_node').fatal(f"Cannot load the map: {ex}")
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
