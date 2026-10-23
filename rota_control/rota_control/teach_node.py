#!/usr/bin/env python3
"""
Teach phase of Teach and Repeat.

Each /localization/reference becomes a keyframe: the closest scan to it, and an
edge to the previous keyframe from ICP (the pose difference if ICP fails). The
map is saved to map_dir on ~/save and on shutdown, with one more keyframe at the
last pose. Publishes /teach/path and /teach/keyframes.
"""

import os
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rota_common import icp, se2, tnr_map
from rota_common.msgs import pose_from_msg, pose_stamped, pose_to_msg
from rota_common.scan import project
from rota_common.tnr_map import Keyframe

# Same QoS as localization_node, so the first reference is not lost.
REFERENCE_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TeachNode(Node):

    def __init__(self):
        super().__init__('teach_node')

        self.declare_parameter('map_dir', 'data/map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('icp_max_distance', 0.30)
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_min_fitness', 0.25)
        # Only written to map.json, should match localization_node's.
        self.declare_parameter('mode', 'encoders')
        self.declare_parameter('tau_d', 0.30)
        self.declare_parameter('tau_theta', 0.30)

        self.map_dir = os.path.abspath(os.path.expanduser(self.get_parameter('map_dir').value))
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.icp_args = (self.get_parameter('icp_max_distance').value,
                         self.get_parameter('icp_max_iterations').value,
                         self.get_parameter('icp_min_fitness').value)
        self.meta = {
            'robot_frame': self.robot_frame,
            'mode': self.get_parameter('mode').value,
            'tau_d': self.get_parameter('tau_d').value,
            'tau_theta': self.get_parameter('tau_theta').value,
        }
        self.get_logger().info(f"map_dir={self.map_dir} robot_frame={self.robot_frame}")

        self.keyframes = []
        self.scans = deque(maxlen=10)  # (stamp, points)
        self.pending = []              # (stamp, pose) of references waiting for a scan
        self.last_odom = None
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_path_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_path = self.create_publisher(Path, '/teach/path', 10)
        self.pub_keyframes = self.create_publisher(MarkerArray, '/teach/keyframes', 10)
        self.create_subscription(PoseStamped, '/localization/reference', self.on_reference,
                                 REFERENCE_QOS)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odom', self.on_odom, 10)
        self.create_service(Trigger, '~/save', self.on_save)

    def on_reference(self, msg):
        self.pending.append((seconds(msg.header.stamp), pose_from_msg(msg)))
        self.add_keyframes()

    def on_scan(self, msg):
        points = project(msg, self.tf_buffer, self.robot_frame, self.min_range, self.max_range)
        if points is None:
            self.get_logger().warn(f"No TF {self.robot_frame} -> {msg.header.frame_id} yet",
                                   throttle_duration_sec=2.0)
            return
        self.scans.append((seconds(msg.header.stamp), points))
        self.add_keyframes()

    def on_odom(self, msg):
        p = pose_from_msg(msg)
        self.last_odom = p
        if self.last_path_pose is not None:
            d = se2.between(self.last_path_pose, p)
            if se2.norm(d) < 0.02 and abs(d[2]) < 0.05:
                return
        self.last_path_pose = p
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose_stamped(msg.header.stamp, 'odom', p))
        self.pub_path.publish(self.path)

    def add_keyframes(self):
        """Make the keyframes of the references whose scan (at or after their stamp) is in."""
        while self.pending and self.scans and self.scans[-1][0] >= self.pending[0][0]:
            stamp, pose = self.pending.pop(0)
            scan = min(self.scans, key=lambda s: abs(s[0] - stamp))
            kf = self.make_keyframe(stamp, pose, scan[1])
            self.keyframes.append(kf)
            self.get_logger().info(
                f"Keyframe #{kf.id}: {len(kf.points)} points, edge "
                f"({kf.edge_from_prev[0]:.3f}, {kf.edge_from_prev[1]:.3f}, "
                f"{kf.edge_from_prev[2]:.3f} rad), ICP "
                f"{'ok' if kf.edge_icp_ok else 'failed'} (fitness {kf.edge_fitness:.2f})")
            self.publish_keyframes()

    def make_keyframe(self, stamp, pose, points):
        kf = Keyframe(len(self.keyframes), stamp, pose, points=points)
        if self.keyframes:
            prev = self.keyframes[-1]
            guess = se2.between(prev.pose_odom, pose)
            edge, kf.edge_fitness, kf.edge_icp_ok = icp.align(points, prev.points, guess,
                                                              *self.icp_args)
            kf.edge_from_prev = edge if kf.edge_icp_ok else guess
        return kf

    def publish_keyframes(self):
        anchor = self.keyframes[0].pose_odom
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for kf, pose in zip(self.keyframes, tnr_map.chain_poses(self.keyframes)):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns = 'keyframes'
            m.id = kf.id
            m.type = Marker.ARROW
            m.pose = pose_to_msg(se2.compose(anchor, pose))
            m.scale.x, m.scale.y, m.scale.z = 0.15, 0.03, 0.03
            m.color.g, m.color.a = 0.8, 1.0
            markers.markers.append(m)
        self.pub_keyframes.publish(markers)

    def save(self):
        if not self.keyframes:
            return 0
        keyframes = list(self.keyframes)
        # The Teach usually ends between two references: one more keyframe at the last pose.
        if self.last_odom is not None and self.scans:
            d = se2.between(keyframes[-1].pose_odom, self.last_odom)
            if se2.norm(d) >= 0.02 or abs(d[2]) >= 0.02:
                keyframes.append(self.make_keyframe(self.scans[-1][0], self.last_odom,
                                                    self.scans[-1][1]))
        tnr_map.save(self.map_dir, keyframes, self.meta)
        self.get_logger().info(f"Saved {len(keyframes)} keyframes to {self.map_dir}")
        return len(keyframes)

    def on_save(self, request, response):
        saved = self.save()
        response.success = saved > 0
        response.message = f"{saved} keyframes in {self.map_dir}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TeachNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.save()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
