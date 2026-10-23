import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import LookupException

from rota_common import msgs
from rota_common.scan import ScanProjector, scan_to_points


def make_scan(ranges, frame='base_scan', angle_min=0.0, angle_increment=math.pi / 2):
    msg = LaserScan()
    msg.header.frame_id = frame
    msg.angle_min = angle_min
    msg.angle_increment = angle_increment
    msg.range_min = 0.12
    msg.range_max = 3.5
    msg.ranges = [float(r) for r in ranges]
    return msg


def test_scan_to_points_geometry():
    pts = scan_to_points(make_scan([1.0, 2.0, 1.5, 0.5]), 0.0, 10.0)
    assert pts == pytest.approx(np.array([[1.0, 0.0], [0.0, 2.0], [-1.5, 0.0], [0.0, -0.5]]), abs=1e-12)


def test_scan_to_points_filters():
    # inf, nan, below sensor min, above caller max, above sensor max
    scan = make_scan([1.0, math.inf, math.nan, 0.05, 3.0, 4.0, 0.2], angle_increment=0.1)
    pts = scan_to_points(scan, 0.15, 2.5)
    assert len(pts) == 2
    assert pts[0] == pytest.approx([1.0, 0.0])
    assert np.hypot(*pts[1]) == pytest.approx(0.2)
    assert math.atan2(pts[1][1], pts[1][0]) == pytest.approx(0.6)


def test_scan_to_points_empty():
    assert scan_to_points(make_scan([math.inf] * 4), 0.0, 10.0).shape == (0, 2)


class FakeLogger:
    def warn(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass


class FakeNode:
    def get_logger(self):
        return FakeLogger()


class FakeBuffer:
    def __init__(self, tf=None):
        self.tf = tf
        self.calls = 0

    def lookup_transform(self, target, source, time):
        self.calls += 1
        if self.tf is None:
            raise LookupException('not yet')
        assert (target, source) == (self.tf.header.frame_id, self.tf.child_frame_id)
        return self.tf


def test_projector_applies_mount_and_caches():
    # LiDAR 0.1 m ahead of the robot, rotated 90 degrees.
    buffer = FakeBuffer()
    project = ScanProjector(FakeNode(), buffer, 'base_footprint', 0.0, 10.0)
    scan = make_scan([1.0, 2.0, 1.5, 0.5])
    assert project(scan) is None

    buffer.tf = msgs.transform_stamped(Time(), 'base_footprint', 'base_scan', (0.1, 0.0, math.pi / 2))
    pts = project(scan)
    expected = np.array([[0.1, 1.0], [-1.9, 0.0], [0.1, -1.5], [0.6, 0.0]])
    assert pts == pytest.approx(expected, abs=1e-12)

    project(scan)
    assert buffer.calls == 2
