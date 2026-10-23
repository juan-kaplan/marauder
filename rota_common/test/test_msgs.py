import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

from rota_common import msgs

P = (1.5, -0.25, 2.0)


def assert_pose(actual, expected):
    assert actual == pytest.approx(expected)


def test_pose_roundtrip():
    msg = msgs.pose_to_msg(P)
    assert msg.position.z == 0.0
    assert_pose(msgs.pose_from_msg(msg), P)


def test_pose_from_wrapped_messages():
    stamped = msgs.pose_stamped(Time(sec=3, nanosec=7), 'odom', P)
    assert stamped.header.frame_id == 'odom'
    assert (stamped.header.stamp.sec, stamped.header.stamp.nanosec) == (3, 7)
    assert_pose(msgs.pose_from_msg(stamped), P)

    odom = Odometry()
    odom.pose = PoseWithCovariance(pose=msgs.pose_to_msg(P))
    assert_pose(msgs.pose_from_msg(odom), P)
    assert_pose(msgs.pose_from_msg(odom.pose), P)


def test_transform_roundtrip():
    tf = msgs.transform_stamped(Time(sec=1), 'odom', 'reference', P)
    assert (tf.header.frame_id, tf.child_frame_id) == ('odom', 'reference')
    assert tf.transform.translation.z == 0.0
    assert_pose(msgs.pose_from_transform(tf), P)
    assert_pose(msgs.pose_from_transform(tf.transform), P)
