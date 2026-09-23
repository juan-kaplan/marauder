#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_matrix
from tf_transformations import quaternion_matrix

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

import open3d as o3d
import numpy as np


def laser_scan_to_open3d(scan_msg: LaserScan, range_min=None, range_max=None) -> o3d.geometry.PointCloud:
    points = []
    angle = scan_msg.angle_min

    range_min = max(range_min, scan_msg.range_min) if range_min else scan_msg.range_min
    range_max = min(range_max, scan_msg.range_max) if range_max else scan_msg.range_max

    for r in scan_msg.ranges:
        if range_min <= r <= range_max and not math.isnan(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0  # Plano 2D
            points.append([x, y, z])
        angle += scan_msg.angle_increment

    pcd = o3d.geometry.PointCloud()
    if points:
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    return pcd

def transform_to_matrix(t: TransformStamped) -> np.ndarray:
    q = t.transform.rotation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = t.transform.translation.x
    matrix[1, 3] = t.transform.translation.y
    matrix[2, 3] = t.transform.translation.z
    return matrix


def get_2D_coordinates(T):
    x = T[0, 3]
    y = T[1, 3]
    _, _, yaw = euler_from_matrix(T, axes='sxyz')
    return x, y, yaw


class LidarOdometryNode(Node):

    def __init__(self):
        super().__init__('lidar_odometry')

        self.get_logger().info(f"Parameters:")

        # Robot
        self.declare_parameter("min_range", 0.10)
        self.min_range = float(self.get_parameter("min_range").value)
        self.get_logger().info(f"min_range: {self.min_range}")

        self.declare_parameter("max_range", 5.00)
        self.max_range = float(self.get_parameter("max_range").value)
        self.get_logger().info(f"max_range: {self.max_range}")

        self.declare_parameter("max_distance", 0.50)
        self.max_distance = float(self.get_parameter("max_distance").value)
        self.get_logger().info(f"max_distance: {self.max_distance}")

        self.declare_parameter("max_angle", 15.0)
        self.max_angle = float(self.get_parameter("max_angle").value)
        self.get_logger().info(f"max_angle: {self.max_angle}")

        self.parent_frame_id = 'odom'
        self.reference_frame_id = 'reference'
        self.frame_id = 'base_link'

        self.timer_startup = self.create_timer(0.1, self.timer_startup_callback_)

    def timer_startup_callback_(self):
        current_time = self.get_clock().now()
        if 0 == current_time.nanoseconds:
            self.get_logger().info("Waiting for valid clock...", throttle_duration_sec=2.0)
            return
        self.get_logger().info(f'Clock received! Time: {current_time.seconds_nanoseconds()}')
        self.timer_startup.cancel()
        self.start_node_()
    
    def start_node_(self):
        self.initialized = False
        self.reference_pcd = None
        self.T_odom_reference = np.eye(4)
        self.T_guess = np.eye(4)
        self.map_pcd = o3d.geometry.PointCloud()
        self.map_voxel_size = 0.03
        self.lidar_to_base = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_map = self.create_publisher(PointCloud2, '/map_points', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.get_logger().info("Init OK")

    def publish_map_(self, stamp, pcd_odom):
        self.map_pcd += pcd_odom
        self.map_pcd = self.map_pcd.voxel_down_sample(self.map_voxel_size)

        header = Header()
        header.stamp = stamp
        header.frame_id = self.parent_frame_id
        map_msg = point_cloud2.create_cloud_xyz32(header, np.asarray(self.map_pcd.points))
        self.pub_map.publish(map_msg)

    def do_icp(self, source_pcd, target_pcd, T_init):
        threshold = 0.2 

        reg_p2p = o3d.pipelines.registration.registration_icp(
            source=source_pcd,
            target=target_pcd,
            max_correspondence_distance=threshold,
            init=T_init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )

        return reg_p2p.transformation

    def on_scan(self, msg):
        if self.lidar_to_base is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.frame_id, msg.header.frame_id, Time())
            except TransformException as ex:
                self.get_logger().warn(
                    f"Could not get static transform {msg.header.frame_id} -> {self.frame_id}: {ex}",
                    throttle_duration_sec=2.0)
                return
            self.lidar_to_base = transform_to_matrix(tf)
            self.get_logger().info(
                f"Got static transform {msg.header.frame_id} -> {self.frame_id}, "
                f"scans will be aligned to {self.frame_id} before ICP.")

        if not self.initialized:
            self.reference_pcd = laser_scan_to_open3d(msg, self.min_range, self.max_range)
            self.reference_pcd.transform(self.lidar_to_base)
            self.publish_map_(msg.header.stamp, self.reference_pcd)

            self.initialized = True
            self.get_logger().info("Reference point cloud initialized.")
            return

        current_pcd = laser_scan_to_open3d(msg, self.min_range, self.max_range)
        current_pcd.transform(self.lidar_to_base)
        T = self.do_icp(current_pcd, self.reference_pcd, self.T_guess)
        T_odom_base = self.T_odom_reference @ T

        x, y, yaw = get_2D_coordinates(T_odom_base)
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.parent_frame_id
        odom_msg.child_frame_id = self.frame_id

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.pub_odom.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        current_pcd_odom = o3d.geometry.PointCloud(current_pcd)
        current_pcd_odom.transform(T_odom_base)
        self.publish_map_(msg.header.stamp, current_pcd_odom)

        dx, dy, dyaw = get_2D_coordinates(T)
        distance = math.hypot(dx, dy)
        angle_deg = abs(math.degrees(dyaw))

        if distance > self.max_distance or angle_deg > self.max_angle:
            self.reference_pcd = current_pcd
            self.T_odom_reference = T_odom_base
            self.T_guess = np.eye(4)
            self.get_logger().info("Reference point cloud updated.")
        else:
            self.T_guess = T

def main(args=None):
    rclpy.init(args=args)
    node = LidarOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
