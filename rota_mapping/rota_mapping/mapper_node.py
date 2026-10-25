#!/usr/bin/env python3
"""
Occupancy grid from /scan and the robot pose in odom_topic (default
/localization/odom), published on /map in the frame of that pose. Log-odds
grid: each scan marks the cells along every beam as free and the cell at its
end as occupied. The grid is fixed in size and starts centred on the origin.
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rota_common.msgs import pose_from_msg
from rota_common.scan import project

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MapperNode(Node):

    def __init__(self):
        super().__init__('mapper_node')

        self.declare_parameter('odom_topic', '/localization/odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('size', 20.0)
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('l_free', -0.4)
        self.declare_parameter('l_occupied', 0.85)
        self.declare_parameter('l_limit', 5.0)
        self.declare_parameter('publish_rate', 2.0)

        self.robot_frame = self.get_parameter('robot_frame').value
        self.resolution = self.get_parameter('resolution').value
        self.cells = int(self.get_parameter('size').value / self.resolution)
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.l_free = self.get_parameter('l_free').value
        self.l_occupied = self.get_parameter('l_occupied').value
        self.l_limit = self.get_parameter('l_limit').value

        self.pose = None
        self.frame = None
        self.log_odds = np.zeros((self.cells, self.cells))
        self.origin = -0.5 * self.cells * self.resolution

        self.pub_map = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_map)

    def on_odom(self, msg):
        self.pose = pose_from_msg(msg)
        self.frame = msg.header.frame_id

    def cell(self, xy):
        return np.floor((xy - self.origin) / self.resolution).astype(int)

    def on_scan(self, msg):
        if self.pose is None:
            return
        points = project(msg, self.tf_buffer, self.robot_frame, self.min_range, self.max_range)
        if points is None or len(points) == 0:
            return
        x, y, theta = self.pose
        c, s = np.cos(theta), np.sin(theta)
        ends = points @ np.array([[c, s], [-s, c]]) + (x, y)
        start = np.array([x, y])

        # sample every beam at half a cell, stopping a cell before its end point
        steps = np.arange(0.0, self.max_range, 0.5 * self.resolution)
        lengths = np.maximum(np.linalg.norm(ends - start, axis=1), 1e-6)
        along = start + (ends - start)[:, None, :] * (steps[None, :] / lengths[:, None])[:, :, None]
        free = self.cell(along[steps[None, :] < lengths[:, None] - self.resolution])
        occupied = self.cell(ends)
        self.update(free, self.l_free)
        self.update(occupied, self.l_occupied)

    def update(self, ij, value):
        ij = np.unique(ij, axis=0)
        ij = ij[np.all((ij >= 0) & (ij < self.cells), axis=1)]
        cell = self.log_odds[ij[:, 1], ij[:, 0]]
        self.log_odds[ij[:, 1], ij[:, 0]] = np.clip(cell + value, -self.l_limit, self.l_limit)

    def publish_map(self):
        if self.frame is None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame
        grid.info.resolution = self.resolution
        grid.info.width = grid.info.height = self.cells
        grid.info.origin.position.x = grid.info.origin.position.y = self.origin
        grid.info.origin.orientation.w = 1.0
        probability = 100.0 / (1.0 + np.exp(-self.log_odds))
        data = np.where(self.log_odds == 0.0, -1, np.round(probability)).astype(np.int8)
        grid.data = data.ravel().tolist()
        self.pub_map.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
