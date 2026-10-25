#!/usr/bin/env python3
"""
Wheel odometry: integrates /joint_states (or /wheel_ticks with wheel_source:=ticks) into /wheel_odom (odom -> robot_frame).
The odom TF is only published with publish_tf:=true, so it does not clash with
localization_node or lidar_odometry_node.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from rota_common import se2
from rota_common.msgs import pose_to_msg, transform_stamped


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('wheel_source', 'joint_states')
        self.declare_parameter('left_wheel_joint', 'wheel_left_joint')
        self.declare_parameter('right_wheel_joint', 'wheel_right_joint')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('meters_per_tick', 4.37e-4)
        self.declare_parameter('publish_tf', False)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.wheel_source = self.get_parameter('wheel_source').value
        if self.wheel_source not in ('joint_states', 'ticks'):
            raise ValueError(f"wheel_source must be joint_states or ticks, got '{self.wheel_source}'")
        self.left_joint = self.get_parameter('left_wheel_joint').value
        self.right_joint = self.get_parameter('right_wheel_joint').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.meters_per_tick = self.get_parameter('meters_per_tick').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.pose = se2.IDENTITY
        self.last = None  # (time, left, right), travelled by each wheel in metres

        self.pub_odom = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.wheel_source == 'ticks':
            from irobot_create_msgs.msg import WheelTicks
            self.create_subscription(WheelTicks, '/wheel_ticks', self.on_ticks, qos_profile_sensor_data)
        else:
            self.create_subscription(JointState, '/joint_states', self.on_joint_states,
                                     qos_profile_sensor_data)

    def on_joint_states(self, msg):
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            self.get_logger().warn(f"Wheel joints not in {list(msg.name)}", throttle_duration_sec=5.0)
            return
        self.on_wheels(msg.header.stamp, self.wheel_radius * left, self.wheel_radius * right)

    def on_ticks(self, msg):
        self.on_wheels(msg.header.stamp, self.meters_per_tick * msg.ticks_left,
                       self.meters_per_tick * msg.ticks_right)

    def on_wheels(self, stamp, left, right):
        t = seconds(stamp)
        last, self.last = self.last, (t, left, right)
        if last is None or t <= last[0]:
            return

        d_left = left - last[1]
        d_right = right - last[2]
        d_lin = 0.5 * (d_left + d_right)
        d_ang = (d_right - d_left) / self.wheel_separation
        self.pose = se2.compose(self.pose, (d_lin * math.cos(0.5 * d_ang), d_lin * math.sin(0.5 * d_ang), d_ang))

        dt = t - last[0]
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.robot_frame
        odom.pose.pose = pose_to_msg(self.pose)
        odom.twist.twist.linear.x = d_lin / dt
        odom.twist.twist.angular.z = d_ang / dt
        self.pub_odom.publish(odom)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                transform_stamped(stamp, self.odom_frame, self.robot_frame, self.pose))


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
