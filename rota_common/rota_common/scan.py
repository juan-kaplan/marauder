import numpy as np


def scan_to_points(msg, rmin, rmax):
    """Valid returns of a LaserScan as an (N, 2) array in the LiDAR frame."""
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
    keep = np.isfinite(ranges) & (ranges >= max(rmin, msg.range_min)) & (ranges <= min(rmax, msg.range_max))
    r, a = ranges[keep], angles[keep]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))
