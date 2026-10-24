import math

import numpy as np
from rclpy.time import Time
from tf2_ros import TransformException

from rota_common.msgs import pose_from_transform


def scan_to_points(msg, rmin, rmax):
    """Valid returns of a LaserScan as an (N, 2) array in the LiDAR frame."""
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
    keep = np.isfinite(ranges) & (ranges >= max(rmin, msg.range_min)) & (ranges <= min(rmax, msg.range_max))
    r, a = ranges[keep], angles[keep]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def project(msg, tf_buffer, robot_frame, rmin, rmax):
    """Scan points in robot_frame through the static LiDAR TF, None while it is missing."""
    try:
        tf = tf_buffer.lookup_transform(robot_frame, msg.header.frame_id, Time())
    except TransformException:
        return None
    x, y, theta = pose_from_transform(tf)
    c, s = math.cos(theta), math.sin(theta)
    return scan_to_points(msg, rmin, rmax) @ np.array([[c, s], [-s, c]]) + (x, y)
