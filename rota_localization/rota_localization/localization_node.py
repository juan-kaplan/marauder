#!/usr/bin/env python3
"""
Layer 1 of Teach and Repeat. Publishes /rel_odom (reference -> robot),
/localization/odom (odom -> robot), /localization/reference (a pose in odom
each time a new reference is made) and TF odom -> reference -> robot_frame.

mode: encoders (wheels), lidar (ICP against the reference scan) or mixed
(wheels between scans, ICP against the reference scan on each scan).
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rota_common import icp, se2
from rota_common.msgs import pose_stamped, pose_to_msg, transform_stamped
from rota_common.scan import project

# Reliable and transient local so teach_node gets every reference, even the first.
REFERENCE_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')

        self.declare_parameter('mode', 'encoders')
        self.declare_parameter('tau_d', 0.30)
        self.declare_parameter('tau_theta', 0.30)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('left_wheel_joint', 'wheel_left_joint')
        self.declare_parameter('right_wheel_joint', 'wheel_right_joint')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('icp_max_distance', 0.30)
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_min_fitness', 0.25)

        self.mode = self.get_parameter('mode').value
        if self.mode not in ('encoders', 'lidar', 'mixed'):
            raise ValueError(f"mode must be encoders, lidar or mixed, got '{self.mode}'")
        self.tau_d = self.get_parameter('tau_d').value
        self.tau_theta = self.get_parameter('tau_theta').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.left_joint = self.get_parameter('left_wheel_joint').value
        self.right_joint = self.get_parameter('right_wheel_joint').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.icp_max_distance = self.get_parameter('icp_max_distance').value
        self.icp_max_iterations = self.get_parameter('icp_max_iterations').value
        self.icp_min_fitness = self.get_parameter('icp_min_fitness').value
        self.get_logger().info(f"mode={self.mode} robot_frame={self.robot_frame} "
                               f"tau_d={self.tau_d} tau_theta={self.tau_theta}")

        self.started = False
        self.stamp = None            # stamp of the latest update
        self.ref_pose = se2.IDENTITY  # odom -> reference
        self.rel = se2.IDENTITY       # reference -> robot
        self.twist = (0.0, 0.0, 0.0)  # robot frame: vx, vy, omega
        self.ref_points = None        # reference scan, in robot_frame
        self.num_references = 0
        self.last_joints = None       # (time, left, right)

        self.pub_rel = self.create_publisher(Odometry, '/rel_odom', 10)
        self.pub_odom = self.create_publisher(Odometry, '/localization/odom', 10)
        self.pub_ref = self.create_publisher(PoseStamped, '/localization/reference', REFERENCE_QOS)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.mode in ('encoders', 'mixed'):
            self.create_subscription(JointState, '/joint_states', self.on_joint_states,
                                     qos_profile_sensor_data)
        if self.mode in ('lidar', 'mixed'):
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_joint_states(self, msg):
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            self.get_logger().warn(f"Wheel joints not in {list(msg.name)}", throttle_duration_sec=5.0)
            return
        t = seconds(msg.header.stamp)
        last, self.last_joints = self.last_joints, (t, left, right)
        if last is None or t <= last[0]:
            if self.mode == 'encoders' and not self.started:
                self.start(msg.header.stamp, None)
                self.publish()
            return

        d_left = self.wheel_radius * (left - last[1])
        d_right = self.wheel_radius * (right - last[2])
        d_lin = 0.5 * (d_left + d_right)
        d_ang = (d_right - d_left) / self.wheel_separation
        delta = (d_lin * math.cos(0.5 * d_ang), d_lin * math.sin(0.5 * d_ang), d_ang)
        if not self.started:
            return  # mixed: the first scan starts the estimate
        dt = t - last[0]
        self.rel = se2.compose(self.rel, delta)
        self.twist = (delta[0] / dt, delta[1] / dt, d_ang / dt)
        self.stamp = msg.header.stamp
        if self.mode == 'encoders':
            self.new_reference_if_needed(msg.header.stamp, None)
        self.publish()

    def on_scan(self, msg):
        points = project(msg, self.tf_buffer, self.robot_frame, self.min_range, self.max_range)
        if points is None:
            self.get_logger().warn(f"No TF {self.robot_frame} -> {msg.header.frame_id} yet",
                                   throttle_duration_sec=2.0)
            return
        if not self.started:
            if self.mode == 'mixed' and self.last_joints is None:
                return  # wait for the wheels
            self.start(msg.header.stamp, points)
            self.publish()
            return

        if self.mode == 'lidar':
            dt = seconds(msg.header.stamp) - seconds(self.stamp)
            if dt <= 0.0:
                return
            guess = se2.compose(self.rel, tuple(v * dt for v in self.twist))  # constant velocity
        else:
            guess = self.rel  # the wheels have moved it since the last scan
        pose, fitness, ok = icp.align(points, self.ref_points, guess, self.icp_max_distance,
                                      self.icp_max_iterations, self.icp_min_fitness)
        if not ok:
            self.get_logger().warn(f"ICP against the reference failed (fitness {fitness:.2f}), "
                                   "using the guess", throttle_duration_sec=2.0)
            pose = guess
        if self.mode == 'lidar':
            d = se2.between(self.rel, pose)
            self.twist = (d[0] / dt, d[1] / dt, d[2] / dt)
            self.stamp = msg.header.stamp
        self.rel = pose
        self.new_reference_if_needed(msg.header.stamp, points)
        self.publish()

    def start(self, stamp, points):
        self.started = True
        self.stamp = stamp
        self.ref_points = points
        self.publish_reference(stamp)

    def new_reference_if_needed(self, stamp, points):
        if se2.norm(self.rel) < self.tau_d and abs(self.rel[2]) < self.tau_theta:
            return
        self.ref_pose = se2.compose(self.ref_pose, self.rel)
        self.rel = se2.IDENTITY
        self.ref_points = points
        self.publish_reference(stamp)

    def publish_reference(self, stamp):
        self.pub_ref.publish(pose_stamped(stamp, 'odom', self.ref_pose))
        self.get_logger().info(f"Reference #{self.num_references} at ({self.ref_pose[0]:.3f}, "
                               f"{self.ref_pose[1]:.3f}, {self.ref_pose[2]:.3f} rad)")
        self.num_references += 1

    def publish(self):
        self.tf_broadcaster.sendTransform([
            transform_stamped(self.stamp, 'odom', 'reference', self.ref_pose),
            transform_stamped(self.stamp, 'reference', self.robot_frame, self.rel)])
        self.pub_rel.publish(self.odometry('reference', self.rel))
        self.pub_odom.publish(self.odometry('odom', se2.compose(self.ref_pose, self.rel)))

    def odometry(self, frame, pose):
        msg = Odometry()
        msg.header.stamp = self.stamp
        msg.header.frame_id = frame
        msg.child_frame_id = self.robot_frame
        msg.pose.pose = pose_to_msg(pose)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = self.twist
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
