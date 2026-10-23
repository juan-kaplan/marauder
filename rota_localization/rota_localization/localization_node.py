#!/usr/bin/env python3
"""
localization_node: layer 1 of Teach and Repeat (``docs/tp1/arquitectura.md``).

Publishes
  /rel_odom                 Odometry, reference -> robot_frame
  /localization/reference   PoseStamped in odom, once per new reference
  /localization/odom        Odometry, odom -> robot_frame
  TF odom -> reference -> robot_frame

The odom -> base TF of Gazebo or the Create 3 must be off: robot_frame can
only have one parent.
"""

from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rota_common.icp import IcpParams
from rota_common.msgs import pose_stamped, pose_to_msg, transform_stamped
from rota_common.scan import ScanProjector
from rota_localization.localizer import MODES, EncoderNoise, Localizer, Reference

ODOM_FRAME = 'odom'
REFERENCE_FRAME = 'reference'

# Reliable so no reference is lost, and transient local so a node that starts
# a bit later (teach_node) still gets the first ones.
REFERENCE_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)


def to_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def to_msg(ns: int):
    return Time(nanoseconds=ns).to_msg()


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')

        self.declare_parameter('mode', 'encoders')
        self.declare_parameter('tau_d', 0.30)
        self.declare_parameter('tau_theta', 0.30)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('left_wheel_joint', 'wheel_left_joint')
        self.declare_parameter('right_wheel_joint', 'wheel_right_joint')
        self.declare_parameter('wheel_radius', 0.033)       # TurtleBot3 burger (m)
        self.declare_parameter('wheel_separation', 0.160)   # TurtleBot3 burger (m)
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('icp_max_correspondence_distance', 0.30)
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_min_fitness', 0.25)
        self.declare_parameter('icp_method', 'point_to_plane')
        self.declare_parameter('icp_degeneracy_ratio', 0.02)
        self.declare_parameter('scan_timeout', 0.5)         # s, mixed only
        self.declare_parameter('encoder_noise_std', 0.0)    # m per sqrt(m) per wheel, simulation only
        self.declare_parameter('encoder_noise_bias', 0.0)   # relative wheel radius difference
        self.declare_parameter('encoder_noise_seed', -1)    # -1: random

        def param(name):
            return self.get_parameter(name).value

        mode = str(param('mode')).strip().lower()
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got '{mode}'")
        self.robot_frame = str(param('robot_frame'))
        self.left_joint = str(param('left_wheel_joint'))
        self.right_joint = str(param('right_wheel_joint'))
        seed = int(param('encoder_noise_seed'))
        noise = EncoderNoise(float(param('encoder_noise_std')), float(param('encoder_noise_bias')),
                             None if seed < 0 else seed)
        icp_params = IcpParams(
            max_correspondence_distance=float(param('icp_max_correspondence_distance')),
            max_iterations=int(param('icp_max_iterations')),
            min_fitness=float(param('icp_min_fitness')),
            method=str(param('icp_method')),
            degeneracy_ratio=float(param('icp_degeneracy_ratio')))
        self.loc = Localizer(mode, float(param('tau_d')), float(param('tau_theta')),
                             float(param('wheel_radius')), float(param('wheel_separation')),
                             icp_params, float(param('scan_timeout')), noise)

        self.pub_rel = self.create_publisher(Odometry, '/rel_odom', 10)
        self.pub_odom = self.create_publisher(Odometry, '/localization/odom', 10)
        self.pub_ref = self.create_publisher(PoseStamped, '/localization/reference', REFERENCE_QOS)
        self.tf_broadcaster = TransformBroadcaster(self)

        if mode in ('encoders', 'mixed'):
            self.create_subscription(JointState, '/joint_states', self.on_joint_states,
                                     qos_profile_sensor_data)
        if mode in ('lidar', 'mixed'):
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.project = ScanProjector(self, self.tf_buffer, self.robot_frame,
                                         float(param('min_scan_range')), float(param('max_scan_range')))
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f"LocalizationNode: mode='{mode}', robot_frame='{self.robot_frame}', "
            f"tau_d={self.loc.tau_d} m, tau_theta={self.loc.tau_theta} rad"
            + (f", wheels '{self.left_joint}'/'{self.right_joint}' r={self.loc.wheel_radius} "
               f"b={self.loc.wheel_separation}" if mode != 'lidar' else '')
            + (f", ICP {icp_params.method}" if mode != 'encoders' else '')
            + (f", encoder noise std={noise.std} bias={noise.bias}" if noise.enabled else ''))

    def on_joint_states(self, msg: JointState):
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            self.get_logger().warn(
                f"Wheel joints '{self.left_joint}'/'{self.right_joint}' not in {list(msg.name)}",
                throttle_duration_sec=5.0)
            return
        self.publish(self.loc.on_wheels(to_ns(msg.header.stamp), left, right))

    def on_scan(self, msg: LaserScan):
        points = self.project(msg)
        if points is None:
            return
        refs = self.loc.on_scan(to_ns(msg.header.stamp), points)
        r = self.loc.last_icp
        if refs is not None and r is not None and not r.ok:
            self.get_logger().warn(
                f"ICP against the reference failed (fitness {r.fitness:.2f}), using the guess",
                throttle_duration_sec=2.0)
        elif refs is not None and r is not None and r.degenerate:
            self.get_logger().info(
                f"ICP poorly constrained in {r.degenerate} direction(s) (corridor?), "
                f"keeping the guess there", throttle_duration_sec=5.0)
        self.publish(refs)

    def publish(self, refs: Optional[List[Reference]]):
        if refs is None:
            return
        for ref in refs:
            self.pub_ref.publish(pose_stamped(to_msg(ref.stamp), ODOM_FRAME, ref.pose))
            self.get_logger().info(
                f"Reference #{self.loc.num_references - 1} at "
                f"({ref.pose[0]:.3f}, {ref.pose[1]:.3f}, {ref.pose[2]:.3f} rad)"
                + ('' if ref.points is not None or self.loc.mode == 'encoders' else ', without scan'))

        stamp = to_msg(self.loc.stamp)
        self.tf_broadcaster.sendTransform([
            transform_stamped(stamp, ODOM_FRAME, REFERENCE_FRAME, self.loc.ref_pose),
            transform_stamped(stamp, REFERENCE_FRAME, self.robot_frame, self.loc.rel),
        ])
        self.pub_rel.publish(self.odometry(stamp, REFERENCE_FRAME, self.loc.rel))
        self.pub_odom.publish(self.odometry(stamp, ODOM_FRAME, self.loc.odom_pose))

    def odometry(self, stamp, frame: str, pose) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.child_frame_id = self.robot_frame
        msg.pose.pose = pose_to_msg(pose)
        vx, vy, omega = self.loc.twist
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.angular.z = float(omega)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
