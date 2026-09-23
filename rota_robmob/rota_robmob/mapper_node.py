import math

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MapperNode(Node):

    def __init__(self):
        super().__init__('mapper_node')

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
        self.occupancy_grid = np.ones((100, 100), dtype=np.int8) * 127

        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub_map = self.create_publisher(OccupancyGrid, 'map', 10)

    def on_odom(self, msg):
        if not self.initialized:
            self.initialized = True

        self.odom = msg

    def on_scan(self, msg):
        if not self.initialized:
            return

        scan_x = msg.ranges * np.cos(msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges)))
        scan_y = msg.ranges * np.sin(msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges)))
        scan = np.vstack((scan_x, scan_y)).T

        q = self.odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        T_lidar_to_robot = np.array([
            [1.0, 0.0,  0.032],
            [0.0, 1.0,  0.000],
            [0.0, 0.0,  1.000]
        ])

        T_robot_to_map = np.array([
            [np.cos(yaw), -np.sin(yaw), self.odom.pose.pose.position.x],
            [np.sin(yaw),  np.cos(yaw), self.odom.pose.pose.position.y],
            [0.0, 0.0, 1.0]
        ])
        
        T_lidar_to_map = T_robot_to_map @ T_lidar_to_robot
        scan_hom = np.hstack((scan, np.ones((len(scan), 1))))
        scan_map = (T_lidar_to_map @ scan_hom.T).T
        self.scan = scan_map[:, :2]
        self.publish_map()

    def publish_map(self):
        msg = OccupancyGrid()     
        self.pub_map.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
