#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from typing import Optional, Tuple

import numpy as np

# Robust import for open3d with automatic .venv detection
try:
    import open3d as o3d
except ImportError:
    import sys
    candidates = [
        os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../.venv/lib/python3.12/site-packages')),
        os.path.abspath(os.path.join(os.getcwd(), '.venv/lib/python3.12/site-packages')),
        os.path.expanduser('~/Documents/robotica_ws/src/marauder/.venv/lib/python3.12/site-packages'),
    ]
    for c in candidates:
        if os.path.isdir(c) and c not in sys.path:
            sys.path.insert(0, c)
    try:
        import open3d as o3d
    except ImportError:
        o3d = None

import rclpy

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_matrix, euler_from_quaternion, quaternion_from_euler
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


def se2_to_matrix4(x: float, y: float, theta: float) -> np.ndarray:
    """Creates a 4x4 homogeneous transformation matrix from SE(2) coordinates."""
    c = math.cos(theta)
    s = math.sin(theta)
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = c
    T[0, 1] = -s
    T[0, 3] = x
    T[1, 0] = s
    T[1, 1] = c
    T[1, 3] = y
    return T


def matrix4_to_se2(T: np.ndarray) -> Tuple[float, float, float]:
    """Extracts (x, y, yaw) from a 4x4 homogeneous matrix."""
    x = float(T[0, 3])
    y = float(T[1, 3])
    _, _, yaw = euler_from_matrix(T, axes='sxyz')
    return (x, y, normalize_angle(yaw))


def points_to_open3d_pcd(points_2d: np.ndarray):
    """Converts a 2D numpy array [[x, y], ...] into an Open3D PointCloud (z=0)."""
    if o3d is None:
        return None
    pcd = o3d.geometry.PointCloud()
    if len(points_2d) > 0:
        pts_3d = np.zeros((len(points_2d), 3), dtype=np.float64)
        pts_3d[:, :2] = points_2d
        pcd.points = o3d.utility.Vector3dVector(pts_3d)
    return pcd


def laser_scan_to_open3d(msg: LaserScan, range_min: float, range_max: float):
    """Converts a LaserScan message into an Open3D PointCloud."""
    if o3d is None:
        return None
    points = []
    angle = msg.angle_min
    rmin = max(range_min, msg.range_min)
    rmax = min(range_max, msg.range_max)

    for r in msg.ranges:
        if rmin <= r <= rmax and not math.isnan(r) and not math.isinf(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y, 0.0])
        angle += msg.angle_increment

    pcd = o3d.geometry.PointCloud()
    if points:
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    return pcd



class RepeatNode(Node):
    """
    Node responsible for the Repeat phase in Teach-and-Repeat navigation.
    Loads a taught trajectory, performs relative localization via Open3D ICP scan matching,
    and guides the differential-drive robot using a polar kinematic controller.
    """

    def __init__(self):
        super().__init__('repeat_node')

        # Parameters
        self.declare_parameter('trajectory_file', 'data/trajectory.json')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('use_stamped_vel', True)
        self.declare_parameter('control_frequency', 20.0)

        # Controller gains & limits
        self.declare_parameter('k_rho', 2.5)
        self.declare_parameter('k_alpha', 6.0)
        self.declare_parameter('k_beta', -1.2)
        self.declare_parameter('k_angle', 1.5)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('scale_linear_by_alpha', True)

        # Tolerances & thresholds
        self.declare_parameter('dist_tolerance', 0.08)          # final goal position tolerance (m)
        self.declare_parameter('angle_tolerance', 0.10)         # final goal heading tolerance (rad)
        self.declare_parameter('keyframe_advance_dist', 0.25)   # distance to next anchor to advance (m)
        self.declare_parameter('icp_max_distance', 0.30)        # max correspondence distance for ICP (m)
        self.declare_parameter('icp_min_fitness', 0.25)         # minimum fitness score to accept ICP
        self.declare_parameter('min_scan_range', 0.12)
        self.declare_parameter('max_scan_range', 3.50)
        self.declare_parameter('auto_start', True)

        # Retrieve parameters
        self.trajectory_file = str(self.get_parameter('trajectory_file').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.use_stamped_vel = bool(self.get_parameter('use_stamped_vel').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)

        self.k_rho = float(self.get_parameter('k_rho').value)
        self.k_alpha = float(self.get_parameter('k_alpha').value)
        self.k_beta = float(self.get_parameter('k_beta').value)
        self.k_angle = float(self.get_parameter('k_angle').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.scale_linear_by_alpha = bool(self.get_parameter('scale_linear_by_alpha').value)

        self.dist_tolerance = float(self.get_parameter('dist_tolerance').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.keyframe_advance_dist = float(self.get_parameter('keyframe_advance_dist').value)
        self.icp_max_distance = float(self.get_parameter('icp_max_distance').value)
        self.icp_min_fitness = float(self.get_parameter('icp_min_fitness').value)
        self.min_scan_range = float(self.get_parameter('min_scan_range').value)
        self.max_scan_range = float(self.get_parameter('max_scan_range').value)
        self.is_active = bool(self.get_parameter('auto_start').value)

        # Trajectory data
        self.keyframes = []
        self.anchor_pcds = []  # List of Open3D PointCloud objects for each keyframe

        # State variables
        self.current_kf_idx = 0
        self.current_odom_pose: Optional[Tuple[float, float, float]] = None
        self.latest_scan_msg: Optional[LaserScan] = None
        self.mission_completed = False

        # Load trajectory
        self.load_trajectory()

        # Publishers & Subscribers
        if self.use_stamped_vel:
            self.pub_cmd_vel = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.pub_markers = self.create_publisher(MarkerArray, '/repeat/markers', 10)
        self.pub_taught_path = self.create_publisher(Path, '/repeat/taught_path', 10)
        self.pub_actual_path = self.create_publisher(Path, '/repeat/actual_path', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.taught_path_msg = Path()
        self.taught_path_msg.header.frame_id = 'odom'
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = 'odom'
        self.last_path_pose = None

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)


        # Services
        self.srv_start = self.create_service(Trigger, '~/start_repeat', self.on_start_repeat)
        self.srv_stop = self.create_service(Trigger, '~/stop_repeat', self.on_stop_repeat)

        # Control loop timer
        timer_period = 1.0 / max(1.0, self.control_frequency)
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"RepeatNode initialized:\n"
            f"  trajectory: '{self.trajectory_file}' ({len(self.keyframes)} anchors)\n"
            f"  cmd_vel: '{self.cmd_vel_topic}' (use_stamped_vel={self.use_stamped_vel})\n"
            f"  gains: k_rho={self.k_rho}, k_alpha={self.k_alpha}, k_beta={self.k_beta}\n"
            f"  active: {self.is_active}"
        )

    def load_trajectory(self):
        traj_path = self.trajectory_file
        if not os.path.isabs(traj_path):
            traj_path = os.path.abspath(traj_path)

        if not os.path.exists(traj_path):
            self.get_logger().error(f"Trajectory metadata file '{traj_path}' not found!")
            return

        npz_path = os.path.splitext(traj_path)[0] + '_scans.npz'
        if not os.path.exists(npz_path):
            self.get_logger().error(f"Trajectory scans file '{npz_path}' not found!")
            return

        with open(traj_path, 'r') as f:
            meta = json.load(f)

        self.keyframes = meta.get('keyframes', [])
        scans_data = np.load(npz_path)

        self.anchor_pcds = []
        for i in range(len(self.keyframes)):
            key = f'scan_{i}'
            if key in scans_data:
                pts = scans_data[key]
                pcd = points_to_open3d_pcd(pts)
                self.anchor_pcds.append(pcd)
            else:
                self.get_logger().warn(f"Key '{key}' missing from '{npz_path}', using empty point cloud.")
                self.anchor_pcds.append(o3d.geometry.PointCloud())

        # Build taught path for RViz
        self.taught_path_msg = Path()
        self.taught_path_msg.header.frame_id = 'odom'
        for kf in self.keyframes:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = float(kf['pose'][0])
            ps.pose.position.y = float(kf['pose'][1])
            ps.pose.position.z = 0.02
            q = quaternion_from_euler(0.0, 0.0, float(kf['pose'][2]))
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.taught_path_msg.poses.append(ps)

        self.get_logger().info(f"Loaded {len(self.keyframes)} keyframes with point clouds.")

    def on_odom(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_odom_pose = (px, py, yaw)

        # Update and publish actual repeat path in RViz
        if self.is_active and not self.mission_completed:
            if self.last_path_pose is None or math.hypot(px - self.last_path_pose[0], py - self.last_path_pose[1]) >= 0.03:
                self.last_path_pose = (px, py)
                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = 'odom'
                ps.pose.position.x = float(px)
                ps.pose.position.y = float(py)
                ps.pose.position.z = 0.02
                ps.pose.orientation = msg.pose.pose.orientation
                self.actual_path_msg.poses.append(ps)
                self.actual_path_msg.header.stamp = msg.header.stamp
                self.pub_actual_path.publish(self.actual_path_msg)


    def on_scan(self, msg: LaserScan):
        self.latest_scan_msg = msg

    def do_icp(self, live_pcd: o3d.geometry.PointCloud, anchor_pcd: o3d.geometry.PointCloud,
               T_init: np.ndarray) -> Tuple[np.ndarray, bool]:
        """
        Aligns live scan (source) to anchor scan (target) using Open3D Point-to-Point ICP.
        Returns the 4x4 relative transformation matrix and a boolean indicating success.
        """
        if len(live_pcd.points) < 10 or len(anchor_pcd.points) < 10:
            return T_init, False

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source=live_pcd,
            target=anchor_pcd,
            max_correspondence_distance=self.icp_max_distance,
            init=T_init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=criteria
        )

        if reg_p2p.fitness >= self.icp_min_fitness:
            return reg_p2p.transformation, True
        else:
            return T_init, False

    def control_loop(self):
        if not self.is_active or self.mission_completed or not self.keyframes:
            return

        if self.current_odom_pose is None or self.latest_scan_msg is None:
            self.get_logger().warn("Waiting for odometry and scan data...", throttle_duration_sec=2.0)
            return

        total_kf = len(self.keyframes)
        k = self.current_kf_idx
        anchor_data = self.keyframes[k]
        anchor_odom_pose = tuple(anchor_data['pose'])
        anchor_pcd = self.anchor_pcds[k]

        # 1. Odometry-based relative displacement guess from anchor k
        # T_guess represents relative displacement: current_odom \ominus anchor_odom
        odom_guess_se2 = se2_minus(self.current_odom_pose, anchor_odom_pose)
        T_guess_mat = se2_to_matrix4(*odom_guess_se2)

        # 2. Open3D ICP Scan Matching
        live_pcd = laser_scan_to_open3d(self.latest_scan_msg, self.min_scan_range, self.max_scan_range)
        T_measured, icp_ok = self.do_icp(live_pcd, anchor_pcd, T_guess_mat)

        if icp_ok:
            s_k = matrix4_to_se2(T_measured)  # measured robot pose relative to anchor k
        else:
            s_k = odom_guess_se2  # fallback to wheel odometry delta
            self.get_logger().warn(
                f"[ICP Fallback] Keyframe #{k}: ICP fitness below threshold, using odometry guess.",
                throttle_duration_sec=2.0
            )

        now_msg = self.get_clock().now().to_msg()

        # Dynamic TF 1: Broadcast current anchor frame (odom -> current_anchor)
        tf_anchor = TransformStamped()
        tf_anchor.header.stamp = now_msg
        tf_anchor.header.frame_id = 'odom'
        tf_anchor.child_frame_id = 'current_anchor'
        tf_anchor.transform.translation.x = float(anchor_odom_pose[0])
        tf_anchor.transform.translation.y = float(anchor_odom_pose[1])
        tf_anchor.transform.translation.z = 0.0
        q_a = quaternion_from_euler(0.0, 0.0, float(anchor_odom_pose[2]))
        tf_anchor.transform.rotation = Quaternion(x=q_a[0], y=q_a[1], z=q_a[2], w=q_a[3])
        self.tf_broadcaster.sendTransform(tf_anchor)

        # Dynamic TF 2: Broadcast measured ICP robot pose in anchor frame (current_anchor -> base_link_icp)
        tf_icp = TransformStamped()
        tf_icp.header.stamp = now_msg
        tf_icp.header.frame_id = 'current_anchor'
        tf_icp.child_frame_id = 'base_link_icp'
        tf_icp.transform.translation.x = float(s_k[0])
        tf_icp.transform.translation.y = float(s_k[1])
        tf_icp.transform.translation.z = 0.0
        q_i = quaternion_from_euler(0.0, 0.0, float(s_k[2]))
        tf_icp.transform.rotation = Quaternion(x=q_i[0], y=q_i[1], z=q_i[2], w=q_i[3])
        self.tf_broadcaster.sendTransform(tf_icp)

        # Publish taught reference path periodically for RViz
        if self.taught_path_msg.poses:
            self.taught_path_msg.header.stamp = now_msg
            self.pub_taught_path.publish(self.taught_path_msg)

        # 3. Target setpoint determination
        is_last_kf = (k >= total_kf - 1)

        if not is_last_kf:
            # Setpoint is next keyframe k+1
            # In metadata, keyframe k+1 has relative transform rel_transform from keyframe k:
            # T_(k, k+1) = pose_(k+1) \ominus pose_k
            T_k_next = tuple(self.keyframes[k + 1]['rel_transform'])

            # Dynamic TF 3: Broadcast target anchor in current anchor frame (current_anchor -> target_anchor)
            tf_target = TransformStamped()
            tf_target.header.stamp = now_msg
            tf_target.header.frame_id = 'current_anchor'
            tf_target.child_frame_id = 'target_anchor'
            tf_target.transform.translation.x = float(T_k_next[0])
            tf_target.transform.translation.y = float(T_k_next[1])
            tf_target.transform.translation.z = 0.0
            q_t = quaternion_from_euler(0.0, 0.0, float(T_k_next[2]))
            tf_target.transform.rotation = Quaternion(x=q_t[0], y=q_t[1], z=q_t[2], w=q_t[3])
            self.tf_broadcaster.sendTransform(tf_target)

            # Target pose in current robot frame:
            # T_(target, robot) = T_(k, k+1) \ominus s_k(t)
            target_rel_robot = se2_minus(T_k_next, s_k)
        else:
            # Final keyframe reached: target is anchor k itself (or pose 0 relative to anchor)
            target_rel_robot = se2_minus((0.0, 0.0, 0.0), s_k)


        dx_r, dy_r, dtheta_r = target_rel_robot
        rho = math.hypot(dx_r, dy_r)

        # 4. Keyframe Transition Check
        if not is_last_kf:
            # Advance condition: within keyframe_advance_dist
            # or passed the perpendicular plane of keyframe k+1
            advance = False
            if rho <= self.keyframe_advance_dist:
                advance = True
            elif dx_r < 0.0 and abs(dy_r) < 0.2:
                # Robot has driven past the waypoint along the track
                advance = True

            if advance:
                self.current_kf_idx += 1
                self.get_logger().info(
                    f"-> Advanced to Keyframe #{self.current_kf_idx}/{total_kf - 1} "
                    f"(previous anchor rho={rho:.3f}m)"
                )
                self.publish_markers(target_rel_robot)
                return

        # 5. Final Goal Check
        if is_last_kf and rho <= self.dist_tolerance:
            heading_err = normalize_angle(dtheta_r)
            if abs(heading_err) <= self.angle_tolerance:
                self.mission_completed = True
                self.publish_zero_velocity()
                self.get_logger().info(
                    f"SUCCESS: Final Keyframe #{k} reached within tolerances! "
                    f"(rho={rho:.3f}m, heading_err={math.degrees(heading_err):.1f}°)"
                )
                self.publish_markers(target_rel_robot)
                return
            else:
                # Rotate in place to match orientation
                v = 0.0
                omega = float(np.clip(self.k_angle * heading_err, -self.max_angular_speed, self.max_angular_speed))
                self.publish_velocity(v, omega)
                self.publish_markers(target_rel_robot)
                return

        # 6. Kinematic Polar Controller (Siegwart)
        alpha = normalize_angle(math.atan2(dy_r, dx_r))
        beta = normalize_angle(dtheta_r - alpha)

        v = self.k_rho * rho
        omega = self.k_alpha * alpha + self.k_beta * beta

        if self.scale_linear_by_alpha and v > 0.0:
            v = v * max(0.0, math.cos(alpha))

        v = float(np.clip(v, -self.max_linear_speed, self.max_linear_speed))
        omega = float(np.clip(omega, -self.max_angular_speed, self.max_angular_speed))

        self.publish_velocity(v, omega)
        self.publish_markers(target_rel_robot)

        self.get_logger().info(
            f"[Repeat KF #{k}/{total_kf - 1}] rho={rho:.2f}m, alpha={math.degrees(alpha):.1f}°, "
            f"cmd: v={v:.2f}m/s, w={omega:.2f}rad/s (ICP: {'OK' if icp_ok else 'ODOM'})",
            throttle_duration_sec=1.0
        )

    def publish_velocity(self, v: float, omega: float):
        if self.use_stamped_vel:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(omega)
            self.pub_cmd_vel.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(omega)
            self.pub_cmd_vel.publish(msg)

    def publish_zero_velocity(self):
        self.publish_velocity(0.0, 0.0)

    def on_start_repeat(self, request, response):
        self.is_active = True
        self.mission_completed = False
        response.success = True
        response.message = f"Repeat started from Keyframe #{self.current_kf_idx}."
        self.get_logger().info(response.message)
        return response

    def on_stop_repeat(self, request, response):
        self.is_active = False
        self.publish_zero_velocity()
        response.success = True
        response.message = "Repeat stopped."
        self.get_logger().info(response.message)
        return response

    def publish_markers(self, target_rel_robot: Optional[Tuple[float, float, float]] = None):
        if not self.keyframes or self.current_odom_pose is None:
            return

        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        # 1. Target waypoint arrow in robot frame
        if target_rel_robot is not None:
            tx, ty, tth = target_rel_robot
            wp_marker = Marker()
            wp_marker.header.frame_id = 'base_link'
            wp_marker.header.stamp = stamp
            wp_marker.ns = 'repeat_target'
            wp_marker.id = 100
            wp_marker.type = Marker.ARROW
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = float(tx)
            wp_marker.pose.position.y = float(ty)
            wp_marker.pose.position.z = 0.05
            q = quaternion_from_euler(0.0, 0.0, tth)
            wp_marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            wp_marker.scale.x = 0.20
            wp_marker.scale.y = 0.04
            wp_marker.scale.z = 0.04
            wp_marker.color.r = 1.0
            wp_marker.color.g = 0.0
            wp_marker.color.b = 0.2
            wp_marker.color.a = 1.0
            marker_array.markers.append(wp_marker)

        # 2. Text status overlay
        text_marker = Marker()
        text_marker.header.frame_id = 'base_link'
        text_marker.header.stamp = stamp
        text_marker.ns = 'repeat_status'
        text_marker.id = 200
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 0.4
        text_marker.scale.z = 0.12  # text size
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        total_kf = len(self.keyframes)
        status_str = f"KF #{self.current_kf_idx}/{total_kf - 1}"
        if self.mission_completed:
            status_str += " [COMPLETED]"
        elif not self.is_active:
            status_str += " [PAUSED]"
        text_marker.text = status_str
        marker_array.markers.append(text_marker)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RepeatNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_zero_velocity()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
