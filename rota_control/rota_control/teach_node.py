#!/usr/bin/env python3

import json
import math
import os
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def normalize_angle(angle: float) -> float:
    """Normalizes an angle strictly to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if a <= -math.pi or math.isclose(a, -math.pi, abs_tol=1e-9):
        return math.pi
    return a


def get_yaw_from_quaternion(q) -> float:
    """Extracts yaw from a Quaternion message or list."""
    if hasattr(q, 'x'):
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
    else:
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-6:
        return 0.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
    return yaw


def se2_minus(x_a: Tuple[float, float, float], x_b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    r"""
    Computes relative transform x_a \ominus x_b (pose of A expressed in frame B).
    x_b is reference, x_a is target.
    """
    xb, yb, thetab = x_b
    xa, ya, thetaa = x_a
    dx = xa - xb
    dy = ya - yb
    c = math.cos(thetab)
    s = math.sin(thetab)
    rel_x = c * dx + s * dy
    rel_y = -s * dx + c * dy
    rel_theta = normalize_angle(thetaa - thetab)
    return (rel_x, rel_y, rel_theta)


def laser_scan_to_points(msg: LaserScan, range_min: float, range_max: float) -> np.ndarray:
    """Converts a LaserScan message into a 2D array of (x, y) coordinates."""
    points = []
    angle = msg.angle_min
    rmin = max(range_min, msg.range_min)
    rmax = min(range_max, msg.range_max)

    for r in msg.ranges:
        if rmin <= r <= rmax and not math.isnan(r) and not math.isinf(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])
        angle += msg.angle_increment

    if not points:
        return np.empty((0, 2), dtype=np.float64)
    return np.array(points, dtype=np.float64)


class TeachNode(Node):
    """
    Node responsible for the Teach phase in Teach-and-Repeat navigation.
    Subscribes to odometry and LiDAR scans, performs spatial/angular sampling,
    and records keyframes (anchors) with relative transformations.
    """

    def __init__(self):
        super().__init__('teach_node')

        # Parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('tau_d', 0.30)           # linear distance threshold (meters)
        self.declare_parameter('tau_theta', 0.30)       # angular threshold (radians, ~17 deg)
        self.declare_parameter('min_scan_range', 0.12)  # LiDAR minimum range
        self.declare_parameter('max_scan_range', 3.50)  # LiDAR maximum range
        self.declare_parameter('output_file', 'data/trajectory.json')
        self.declare_parameter('auto_record', True)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.tau_d = float(self.get_parameter('tau_d').value)
        self.tau_theta = float(self.get_parameter('tau_theta').value)
        self.min_scan_range = float(self.get_parameter('min_scan_range').value)
        self.max_scan_range = float(self.get_parameter('max_scan_range').value)
        self.output_file = str(self.get_parameter('output_file').value)
        self.is_recording = bool(self.get_parameter('auto_record').value)

        # State
        self.current_pose = None            # (x, y, theta)
        self.last_keyframe_pose = None      # (x, y, theta)
        self.latest_scan_msg = None
        self.keyframes: List[dict] = []     # List of keyframe metadata & poses
        self.keyframe_points = []           # List of 2D numpy arrays of points

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)

        # Publishers
        self.pub_markers = self.create_publisher(MarkerArray, '/teach/markers', 10)
        self.pub_path = self.create_publisher(Path, '/teach/path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'


        # Services
        self.srv_start = self.create_service(Trigger, '~/start_recording', self.on_start_recording)
        self.srv_stop = self.create_service(Trigger, '~/stop_recording', self.on_stop_recording)
        self.srv_save = self.create_service(Trigger, '~/save_trajectory', self.on_save_trajectory)

        # Marker timer (1 Hz)
        self.timer_markers = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(
            f"TeachNode initialized:\n"
            f"  odom: '{self.odom_topic}', scan: '{self.scan_topic}'\n"
            f"  tau_d={self.tau_d}m, tau_theta={self.tau_theta:.2f}rad\n"
            f"  output_file: '{self.output_file}'\n"
            f"  recording active: {self.is_recording}"
        )

    def on_odom(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (px, py, yaw)

        if not self.is_recording:
            return

        # Check if we should drop a new keyframe
        if self.last_keyframe_pose is None:
            # First keyframe
            if self.latest_scan_msg is not None:
                self.record_keyframe()
        else:
            dx = px - self.last_keyframe_pose[0]
            dy = py - self.last_keyframe_pose[1]
            dist = math.hypot(dx, dy)
            dtheta = abs(normalize_angle(yaw - self.last_keyframe_pose[2]))

            if dist >= self.tau_d or dtheta >= self.tau_theta:
                self.record_keyframe()

    def on_scan(self, msg: LaserScan):
        self.latest_scan_msg = msg

    def record_keyframe(self):
        if self.current_pose is None or self.latest_scan_msg is None:
            return

        kf_id = len(self.keyframes)
        px, py, yaw = self.current_pose
        stamp = self.get_clock().now().nanoseconds * 1e-9

        # Relative transform from previous keyframe
        if kf_id == 0:
            rel_transform = (0.0, 0.0, 0.0)
        else:
            rel_transform = se2_minus(self.current_pose, self.last_keyframe_pose)

        # Convert scan to 2D Cartesian points
        pts = laser_scan_to_points(self.latest_scan_msg, self.min_scan_range, self.max_scan_range)

        kf_data = {
            'id': kf_id,
            'stamp': stamp,
            'pose': [float(px), float(py), float(yaw)],
            'rel_transform': [float(rel_transform[0]), float(rel_transform[1]), float(rel_transform[2])],
            'num_points': int(len(pts))
        }

        self.keyframes.append(kf_data)
        self.keyframe_points.append(pts)
        self.last_keyframe_pose = self.current_pose

        # Add to visualization Path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = float(px)
        pose_stamped.pose.position.y = float(py)
        pose_stamped.pose.position.z = 0.02
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.path_msg.poses.append(pose_stamped)
        self.path_msg.header.stamp = pose_stamped.header.stamp
        self.pub_path.publish(self.path_msg)

        self.get_logger().info(
            f"Anchor keyframe #{kf_id} recorded: pose=({px:.3f}, {py:.3f}, {yaw:.3f} rad), "
            f"rel=({rel_transform[0]:.3f}, {rel_transform[1]:.3f}, {rel_transform[2]:.3f}), "
            f"points={len(pts)}"
        )

        self.publish_markers()


    def save_trajectory_to_disk(self) -> bool:
        if not self.keyframes:
            self.get_logger().warn("No keyframes recorded to save!")
            return False

        # Resolve output file path
        out_path = self.output_file
        if not os.path.isabs(out_path):
            out_path = os.path.abspath(out_path)

        out_dir = os.path.dirname(out_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        # Save metadata and poses JSON
        metadata = {
            'tau_d': self.tau_d,
            'tau_theta': self.tau_theta,
            'total_keyframes': len(self.keyframes),
            'keyframes': self.keyframes
        }
        with open(out_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        # Save scan point clouds in matching .npz
        npz_path = os.path.splitext(out_path)[0] + '_scans.npz'
        scans_dict = {f'scan_{i}': pts for i, pts in enumerate(self.keyframe_points)}
        np.savez_compressed(npz_path, **scans_dict)

        self.get_logger().info(
            f"Trajectory successfully saved: {len(self.keyframes)} anchors.\n"
            f"  Metadata: '{out_path}'\n"
            f"  Point clouds: '{npz_path}'"
        )
        return True

    def on_start_recording(self, request, response):
        self.is_recording = True
        response.success = True
        response.message = f"Recording started. Currently {len(self.keyframes)} keyframes recorded."
        self.get_logger().info(response.message)
        return response

    def on_stop_recording(self, request, response):
        self.is_recording = False
        saved = self.save_trajectory_to_disk()
        response.success = saved
        response.message = f"Recording stopped and saved ({len(self.keyframes)} keyframes)."
        self.get_logger().info(response.message)
        return response

    def on_save_trajectory(self, request, response):
        saved = self.save_trajectory_to_disk()
        response.success = saved
        response.message = f"Trajectory saved ({len(self.keyframes)} keyframes)." if saved else "Failed to save trajectory."
        return response

    def publish_markers(self):
        if not self.keyframes:
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. LineStrip connecting keyframe positions
        line_marker = Marker()
        line_marker.header.frame_id = 'odom'
        line_marker.header.stamp = stamp
        line_marker.ns = 'teach_path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.04  # line width
        line_marker.color.r = 0.0
        line_marker.color.g = 0.8
        line_marker.color.b = 0.2
        line_marker.color.a = 1.0

        for kf in self.keyframes:
            p = Point()
            p.x = kf['pose'][0]
            p.y = kf['pose'][1]
            p.z = 0.02
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)

        # 2. Arrows for each keyframe orientation
        for i, kf in enumerate(self.keyframes):
            arrow = Marker()
            arrow.header.frame_id = 'odom'
            arrow.header.stamp = stamp
            arrow.ns = 'teach_anchors'
            arrow.id = i + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = kf['pose'][0]
            arrow.pose.position.y = kf['pose'][1]
            arrow.pose.position.z = 0.02

            q = quaternion_from_euler(0.0, 0.0, kf['pose'][2])
            arrow.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            arrow.scale.x = 0.15  # arrow length
            arrow.scale.y = 0.03  # arrow width
            arrow.scale.z = 0.03  # arrow height

            arrow.color.r = 0.9
            arrow.color.g = 0.6
            arrow.color.b = 0.1
            arrow.color.a = 0.9
            marker_array.markers.append(arrow)

        self.pub_markers.publish(marker_array)
        if self.path_msg.poses:
            self.path_msg.header.stamp = stamp
            self.pub_path.publish(self.path_msg)



def main(args=None):
    rclpy.init(args=args)
    node = TeachNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.is_recording and node.keyframes:
            node.save_trajectory_to_disk()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
