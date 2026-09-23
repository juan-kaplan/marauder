
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import numpy as np


class DifferentialOdometryNode(Node):

    def __init__(self):
        super().__init__('differential_odometry')

        self.get_logger().info(f"Parameters:")

        self.declare_parameter("wheel_radius", 0.05)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.get_logger().info(f"wheel_radius: {self.wheel_radius}")

        self.declare_parameter("wheel_separation", 0.50)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.get_logger().info(f"wheel_separation: {self.wheel_separation}")

        self.declare_parameter("parent_frame_id", "odom")
        self.parent_frame_id = self.get_parameter("parent_frame_id").value
        self.get_logger().info(f"parent_frame_id: {self.parent_frame_id}")

        self.declare_parameter("frame_id", "base_link")
        self.frame_id = self.get_parameter("frame_id").value
        self.get_logger().info(f"frame_id: {self.frame_id}")

        self.declare_parameter("left_wheel_joint_idx", 1)
        self.left_wheel_joint_idx = self.get_parameter("left_wheel_joint_idx").value
        self.get_logger().info(f"left_wheel_joint_idx: {self.left_wheel_joint_idx}")

        self.declare_parameter("right_wheel_joint_idx", 0)
        self.right_wheel_joint_idx = self.get_parameter("right_wheel_joint_idx").value
        self.get_logger().info(f"right_wheel_joint_idx: {self.right_wheel_joint_idx}")

        self.timer_startup = self.create_timer(0.1, self.timer_startup_callback_)
        self.initialized = False

    def timer_startup_callback_(self):
        current_time = self.get_clock().now()
        if 0 == current_time.nanoseconds:
            self.get_logger().info("Waiting for valid clock...", throttle_duration_sec=2.0)
            return
        self.get_logger().info(f'Clock received! Time: {current_time.seconds_nanoseconds()}')
        self.timer_startup.cancel()
        self.start_node_()
    
    def start_node_(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.stamp = self.get_clock().now().to_msg()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

        self.get_logger().info("Init OK")


    def on_joint_states(self, msg):
        if not self.initialized:
            self.last_left_pos = msg.position[self.left_wheel_joint_idx]
            self.last_right_pos = msg.position[self.right_wheel_joint_idx]
            self.last_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.initialized = True
            return

        delta_left = msg.position[self.left_wheel_joint_idx] - self.last_left_pos
        delta_right = msg.position[self.right_wheel_joint_idx] - self.last_right_pos
        delta_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.last_time

        d_left = self.wheel_radius * delta_left
        d_right = self.wheel_radius * delta_right

        delta_lin= (d_right + d_left) / 2
        delta_ang = (d_right - d_left) / self.wheel_separation

        self.x = self.x + delta_lin * np.cos(self.theta + delta_ang/2)
        self.y = self.y + delta_lin * np.sin(self.theta + delta_ang/2)
        self.theta = self.theta + delta_ang

        if delta_time > 0:
            self.v = delta_lin / delta_time
            self.omega = delta_ang / delta_time
        else:
            self.v = 0
            self.omega = 0

        self.last_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_left_pos = msg.position[self.left_wheel_joint_idx]
        self.last_right_pos = msg.position[self.right_wheel_joint_idx]
        self.stamp = msg.header.stamp

        self.publish_odometry()
        self.publish_tf()

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.stamp
        msg.header.frame_id = self.parent_frame_id
        msg.child_frame_id = self.frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.w = np.cos(self.theta / 2)
        msg.pose.pose.orientation.z = np.sin(self.theta / 2)
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.omega

        self.pub_odom.publish(msg)

    def publish_tf(self):
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.stamp
        tf_stamped.header.frame_id = self.parent_frame_id
        tf_stamped.child_frame_id = self.frame_id
        tf_stamped.transform.translation.x = float(self.x)
        tf_stamped.transform.translation.y = float(self.y)
        tf_stamped.transform.translation.z = 0.0
        tf_stamped.transform.rotation.w = float(np.cos(self.theta / 2))
        tf_stamped.transform.rotation.z = float(np.sin(self.theta / 2))
        self.tf_broadcaster.sendTransform(tf_stamped)



def main(args=None):
    rclpy.init(args=args)
    node = DifferentialOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
