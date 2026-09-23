#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion


def normalize_angle(angle: float) -> float:
    """
    Normalizes an angle strictly to the half-open interval (-pi, pi]
    """
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if a <= -math.pi or math.isclose(a, -math.pi, abs_tol=1e-9):
        return math.pi
    return a


def get_yaw_from_quaternion(q) -> float:
    """Extracts yaw from a Quaternion object (with x, y, z, w attributes) or iterable."""
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


class DiffControlNode(Node):
    """
    Kinematic Controller for Differential Drive Mobile Robot.
    """

    def __init__(self):
        super().__init__('diff_control_node')

        # Gains
        self.declare_parameter('k_rho', 3.0)
        self.declare_parameter('k_alpha', 8.0)
        self.declare_parameter('k_beta', -1.5)
        self.declare_parameter('k_angle', 2.0)

        # Tolerances
        self.declare_parameter('dist_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)

        # Velocity limits
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.5)

        # Topic & frame configuration
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('use_tf', False)

        # Control loop & behavior options
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('scale_linear_by_alpha', True)

        # Velocity message type (TwistStamped required for ROS 2 Jazzy Gazebo bridge)
        self.declare_parameter('use_stamped_vel', True)

        # Load parameter values
        self.k_rho = float(self.get_parameter('k_rho').value)
        self.k_alpha = float(self.get_parameter('k_alpha').value)
        self.k_beta = float(self.get_parameter('k_beta').value)
        self.k_angle = float(self.get_parameter('k_angle').value)
        self.dist_tolerance = float(self.get_parameter('dist_tolerance').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.reference_frame = str(self.get_parameter('reference_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.use_tf = bool(self.get_parameter('use_tf').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.scale_linear_by_alpha = bool(self.get_parameter('scale_linear_by_alpha').value)
        self.use_stamped_vel = bool(self.get_parameter('use_stamped_vel').value)

        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.goal_active = False

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers & Subscribers
        if self.use_stamped_vel:
            self.pub_cmd_vel = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub_goal = self.create_subscription(PoseStamped, self.goal_topic, self.on_goal_pose, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        # Periodic control loop timer
        timer_period = 1.0 / max(1.0, self.control_frequency)
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"DiffControlNode initialized:\n"
            f"  k_rho={self.k_rho}, k_alpha={self.k_alpha}, k_beta={self.k_beta}\n"
            f"  subscribing goal: '{self.goal_topic}'\n"
            f"  subscribing odom: '{self.odom_topic}'\n"
            f"  publishing cmd_vel: '{self.cmd_vel_topic}'"
        )

    def on_goal_pose(self, msg: PoseStamped):
        """Callback for incoming goal pose targets."""
        target_pose = msg
        # Transform goal if received in a different coordinate frame
        if msg.header.frame_id and msg.header.frame_id != self.reference_frame:
            try:
                target_pose = self.tf_buffer.transform(
                    msg,
                    self.reference_frame,
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
            except Exception as ex:
                self.get_logger().warn(
                    f"Could not transform goal from '{msg.header.frame_id}' to '{self.reference_frame}': {ex}. Using coordinates as given."
                )

        x_g = target_pose.pose.position.x
        y_g = target_pose.pose.position.y
        theta_g = get_yaw_from_quaternion(target_pose.pose.orientation)

        self.goal_pose = (x_g, y_g, theta_g)
        self.goal_active = True

        self.get_logger().info(
            f"New goal setpoint received: x={x_g:.3f}, y={y_g:.3f}, theta={theta_g:.3f} rad ({math.degrees(theta_g):.1f} deg)"
        )

    def on_odom(self, msg: Odometry):
        """Callback for robot odometry updates."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (x, y, theta)

    def get_current_pose(self):
        """Returns the current robot pose (x, y, theta), optionally using TF lookup."""
        if self.use_tf:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.robot_frame,
                    rclpy.time.Time()
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                theta = get_yaw_from_quaternion(t.transform.rotation)
                return (x, y, theta)
            except TransformException as ex:
                self.get_logger().warn(
                    f"TF lookup ({self.reference_frame} -> {self.robot_frame}) failed: {ex}",
                    throttle_duration_sec=2.0
                )
                return self.current_pose
        return self.current_pose

    def control_loop(self):
        """Executes the kinematic control law according to diff.md."""
        if not self.goal_active or self.goal_pose is None:
            return

        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn("Waiting for robot pose to become available...", throttle_duration_sec=2.0)
            return

        x, y, theta = pose
        x_g, y_g, theta_g = self.goal_pose

        dx = x_g - x
        dy = y_g - y
        rho = math.hypot(dx, dy)

        twist = Twist()

        # Check position tolerance
        if rho <= self.dist_tolerance:
            heading_error = normalize_angle(theta_g - theta)
            if abs(heading_error) <= self.angle_tolerance:
                # Goal position and orientation achieved
                self.goal_active = False
                self.publish_zero_velocity()
                self.get_logger().info(f"Goal pose reached successfully! (x={x:.3f}, y={y:.3f}, theta={theta:.3f})")
                return
            else:
                # Arrived at position; rotate in place to match target orientation
                v = 0.0
                omega = self.k_angle * heading_error
                omega = float(np.clip(omega, -self.max_angular_speed, self.max_angular_speed))
                self.get_logger().info(
                    f"At target position (rho={rho:.3f}m <= {self.dist_tolerance}m). Rotating in place: error={math.degrees(heading_error):.1f}° | w={omega:.2f} rad/s",
                    throttle_duration_sec=1.0
                )
                self.publish_velocity(v, omega)
                return

        alpha = normalize_angle(-theta + math.atan2(dy, dx))
        beta = normalize_angle(theta_g - theta - alpha)

        v = self.k_rho * rho
        omega = self.k_alpha * alpha + self.k_beta * beta

        if self.scale_linear_by_alpha and v > 0.0:
            v = v * max(0.0, math.cos(alpha))

        # Clamp velocities within bounds
        v = float(np.clip(v, -self.max_linear_speed, self.max_linear_speed))
        omega = float(np.clip(omega, -self.max_angular_speed, self.max_angular_speed))

        self.get_logger().info(
            f"Navigating: rho={rho:.3f}m, alpha={math.degrees(alpha):.1f}°, beta={math.degrees(beta):.1f}° | cmd: v={v:.2f}m/s, w={omega:.2f}rad/s",
            throttle_duration_sec=1.0
        )
        self.publish_velocity(v, omega)

    def publish_velocity(self, v: float, omega: float):
        """Publishes velocity commands as either TwistStamped or Twist."""
        if self.use_stamped_vel:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = self.robot_frame
            twist_stamped.twist.linear.x = float(v)
            twist_stamped.twist.angular.z = float(omega)
            self.pub_cmd_vel.publish(twist_stamped)
        else:
            twist = Twist()
            twist.linear.x = float(v)
            twist.angular.z = float(omega)
            self.pub_cmd_vel.publish(twist)

    def publish_zero_velocity(self):
        """Publishes zero velocity to stop the robot."""
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DiffControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_zero_velocity()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
