"""LaserScan to 2D points, in the LiDAR frame or in the robot frame."""

import math
from typing import Optional

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from rota_common.msgs import pose_from_transform


def scan_to_points(msg: LaserScan, rmin: float, rmax: float) -> np.ndarray:
    """
    Return the valid returns of ``msg`` as an ``(N, 2)`` array in the LiDAR frame.

    A return is kept when it is finite and inside both ``[rmin, rmax]`` and the
    sensor's own ``[range_min, range_max]``.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
    lo = max(rmin, msg.range_min)
    hi = min(rmax, msg.range_max)
    keep = np.isfinite(ranges) & (ranges >= lo) & (ranges <= hi)
    r, a = ranges[keep], angles[keep]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


class ScanProjector:
    """
    Turns scans into ``(N, 2)`` points in ``robot_frame``.

    The LiDAR mount is a static TF, so it is looked up once and cached. Until
    it is available, calls return ``None``.
    """

    def __init__(self, node, tf_buffer, robot_frame: str, rmin: float, rmax: float):
        self._node = node
        self._tf_buffer = tf_buffer
        self._robot_frame = robot_frame
        self._rmin = rmin
        self._rmax = rmax
        self._lidar_frame = None
        self._R = None
        self._t = None

    def __call__(self, msg: LaserScan) -> Optional[np.ndarray]:
        if self._lidar_frame != msg.header.frame_id:
            if not self._lookup(msg.header.frame_id):
                return None
        points = scan_to_points(msg, self._rmin, self._rmax)
        return points @ self._R.T + self._t

    def _lookup(self, lidar_frame: str) -> bool:
        try:
            tf = self._tf_buffer.lookup_transform(self._robot_frame, lidar_frame, Time())
        except TransformException as ex:
            self._node.get_logger().warn(
                f"No TF {self._robot_frame} -> {lidar_frame} yet, dropping scan: {ex}",
                throttle_duration_sec=2.0)
            return False
        x, y, theta = pose_from_transform(tf)
        c, s = math.cos(theta), math.sin(theta)
        self._R = np.array([[c, -s], [s, c]])
        self._t = np.array([x, y])
        self._lidar_frame = lidar_frame
        self._node.get_logger().info(
            f"LiDAR mount {self._robot_frame} -> {lidar_frame}: "
            f"({x:.3f}, {y:.3f}, {theta:.3f} rad)")
        return True
