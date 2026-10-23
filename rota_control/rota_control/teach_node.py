#!/usr/bin/env python3
"""
Teach phase of Teach and Repeat.

Each /localization/reference becomes a keyframe with its scan, and the edge
from the previous keyframe is refined by ICP (see teach_logic). The map is
written to map_dir (map.json + scans.npz) on ~/save and on shutdown, with one
more keyframe at the last /localization/odom pose so that the stretch after
the last reference is kept. Publishes
/teach/path (the localizer's path in odom) and /teach/keyframes (the ICP chain
of keyframes, anchored at the odom pose of keyframe 0).
"""

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rota_common import se2, tnr_map
from rota_common.msgs import pose_from_msg, pose_from_transform, pose_stamped, pose_to_msg
from rota_common.scan import scan_to_points
from rota_control.teach_logic import TeachRecorder

# Same QoS as localization_node, so the first reference is not lost if this
# node subscribes a little late.
REFERENCE_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
# A keyframe whose scan is further than this from its reference gets a warning.
MAX_SCAN_OFFSET = 0.1


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
        # Only written to map.json; they should match localization_node's.
        self.declare_parameter('mode', 'encoders')
        self.declare_parameter('tau_d', 0.30)
        self.declare_parameter('tau_theta', 0.30)

        self.map_dir = os.path.abspath(os.path.expanduser(self.get_parameter('map_dir').value))
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.meta = {
            'robot_frame': self.robot_frame,
            'mode': self.get_parameter('mode').value,
            'tau_d': self.get_parameter('tau_d').value,
            'tau_theta': self.get_parameter('tau_theta').value,
        }
        self.recorder = TeachRecorder(
            icp_max_distance=self.get_parameter('icp_max_distance').value,
            icp_max_iterations=self.get_parameter('icp_max_iterations').value,
            icp_min_fitness=self.get_parameter('icp_min_fitness').value)
        self.get_logger().info(f"map_dir={self.map_dir} robot_frame={self.robot_frame}")

        self.mount = None  # robot_frame -> LiDAR, as (R, t)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_path_pose = None
        self.last_odom = None  # (stamp in seconds, pose) of the latest /localization/odom

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
        self.added(self.recorder.on_reference(seconds(msg.header.stamp), pose_from_msg(msg)))

    def on_scan(self, msg):
        points = self.project(msg)
        if points is not None:
            self.added(self.recorder.on_scan(seconds(msg.header.stamp), points))

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

    def added(self, keyframes):
        for kf in keyframes:
            offset = self.recorder.scan_offsets[kf.id]
            if kf.id == 0:
                self.get_logger().info(f"Keyframe #0: {len(kf.points)} points")
            else:
                x, y, theta = kf.edge_from_prev
                icp_state = 'ok' if kf.edge_icp_ok else 'failed'
                self.get_logger().info(
                    f"Keyframe #{kf.id}: {len(kf.points)} points, "
                    f"edge ({x:.3f}, {y:.3f}, {theta:.3f} rad), "
                    f"ICP {icp_state} (fitness {kf.edge_fitness:.2f})")
            if kf.id > 0 and not kf.edge_icp_ok:
                self.get_logger().warn(f"Keyframe #{kf.id}: ICP failed, edge from the localizer")
            if offset is None:
                self.get_logger().warn(f"Keyframe #{kf.id} has no scan")
            elif abs(offset) > MAX_SCAN_OFFSET:
                self.get_logger().warn(f"Keyframe #{kf.id}: closest scan is {offset:+.3f} s away")
        if keyframes:
            self.publish_keyframes()

    def on_odom(self, msg):
        p = pose_from_msg(msg)
        self.last_odom = (seconds(msg.header.stamp), p)
        if self.last_path_pose is not None:
            d = se2.between(self.last_path_pose, p)
            if se2.norm(d) < 0.02 and abs(d[2]) < 0.05:
                return
        self.last_path_pose = p
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose_stamped(msg.header.stamp, 'odom', p))
        self.pub_path.publish(self.path)

    def publish_keyframes(self):
        keyframes = self.recorder.keyframes
        anchor = keyframes[0].pose_odom
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for kf, pose in zip(keyframes, tnr_map.chain_poses(keyframes)):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns = 'keyframes'
            m.id = kf.id
            m.type = Marker.ARROW
            m.pose = pose_to_msg(se2.compose(anchor, pose))
            m.scale.x, m.scale.y, m.scale.z = 0.15, 0.03, 0.03
            ok = kf.id == 0 or kf.edge_icp_ok
            m.color.r, m.color.g, m.color.b = (0.1, 0.8, 0.2) if ok else (1.0, 0.6, 0.0)
            m.color.a = 1.0
            markers.markers.append(m)
        self.pub_keyframes.publish(markers)

    def save(self):
        keyframes = list(self.recorder.keyframes)
        if not keyframes:
            self.get_logger().warn("No keyframes yet, nothing to save")
            return False
        end = self.recorder.end_keyframe(*self.last_odom) if self.last_odom else None
        if end is not None:
            keyframes.append(end)
            x, y, theta = end.edge_from_prev
            self.get_logger().info(
                f"End keyframe #{end.id} at the last pose: {len(end.points)} points, "
                f"edge ({x:.3f}, {y:.3f}, {theta:.3f} rad), "
                f"ICP {'ok' if end.edge_icp_ok else 'failed'} (fitness {end.edge_fitness:.2f})")
        tnr_map.save(self.map_dir, keyframes, self.meta)
        failed = sum(1 for kf in keyframes[1:] if not kf.edge_icp_ok)
        self.get_logger().info(f"Saved {len(keyframes)} keyframes ({failed} edges without ICP) "
                               f"to {self.map_dir}")
        return True

    def on_save(self, request, response):
        response.success = self.save()
        response.message = f"{len(self.recorder.keyframes)} keyframes in {self.map_dir}"
        if self.recorder.pending:
            response.message += f", {len(self.recorder.pending)} references waiting for a scan"
        return response

    def shutdown(self):
        self.added(self.recorder.flush())
        self.save()


def main(args=None):
    rclpy.init(args=args)
    node = TeachNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
