#!/usr/bin/env python3
"""
LiDAR odometry: ICP of each scan against a reference scan, published as
/lidar_odom (odom -> robot_frame). The reference is replaced by the current
scan when the robot has moved max_distance or turned max_angle from it.
The odom TF is only published with publish_tf:=true.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rota_common import icp, se2
from rota_common.msgs import pose_to_msg, transform_stamped
from rota_common.scan import project


class LidarOdometryNode(Node):

    def __init__(self):
        super().__init__('lidar_odometry_node')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('max_distance', 0.50)
        self.declare_parameter('max_angle', 0.26)
        self.declare_parameter('icp_max_distance', 0.30)
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_min_fitness', 0.25)
        self.declare_parameter('publish_tf', False)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.max_distance = self.get_parameter('max_distance').value
        self.max_angle = self.get_parameter('max_angle').value
        self.icp_max_distance = self.get_parameter('icp_max_distance').value
        self.icp_max_iterations = self.get_parameter('icp_max_iterations').value
        self.icp_min_fitness = self.get_parameter('icp_min_fitness').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.ref_pose = se2.IDENTITY  # odom -> reference
        self.rel = se2.IDENTITY       # reference -> robot
        self.ref_points = None

        self.pub_odom = self.create_publisher(Odometry, '/lidar_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        points = project(msg, self.tf_buffer, self.robot_frame, self.min_range, self.max_range)
        if points is None:
            self.get_logger().warn(f"No TF {self.robot_frame} -> {msg.header.frame_id} yet",
                                   throttle_duration_sec=2.0)
            return
        if self.ref_points is None:
            self.ref_points = points
        else:
            pose, fitness, ok = icp.align(points, self.ref_points, self.rel, self.icp_max_distance,
                                          self.icp_max_iterations, self.icp_min_fitness)
            if not ok:
                self.get_logger().warn(f"ICP against the reference failed (fitness {fitness:.2f}), "
                                       "keeping the last pose", throttle_duration_sec=2.0)
                pose = self.rel
            self.rel = pose

        pose = se2.compose(self.ref_pose, self.rel)
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.robot_frame
        odom.pose.pose = pose_to_msg(pose)
        self.pub_odom.publish(odom)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                transform_stamped(msg.header.stamp, self.odom_frame, self.robot_frame, pose))

        if se2.norm(self.rel) > self.max_distance or abs(self.rel[2]) > self.max_angle:
            self.ref_pose = pose
            self.rel = se2.IDENTITY
            self.ref_points = points


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
