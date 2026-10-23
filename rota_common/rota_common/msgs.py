"""Conversions between ``(x, y, theta)`` poses and geometry_msgs messages."""

from typing import Sequence

from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped

from rota_common.se2 import Pose2D, quaternion_from_yaw, yaw_from_quaternion


def _quaternion(theta: float) -> Quaternion:
    x, y, z, w = quaternion_from_yaw(theta)
    return Quaternion(x=x, y=y, z=z, w=w)


def pose_to_msg(p: Sequence[float]) -> Pose:
    msg = Pose()
    msg.position.x = float(p[0])
    msg.position.y = float(p[1])
    msg.orientation = _quaternion(float(p[2]))
    return msg


def pose_from_msg(msg) -> Pose2D:
    """
    Planar pose of a ``Pose``.

    Also takes ``PoseStamped``, ``PoseWithCovariance`` and ``Odometry``: the
    nested ``.pose`` fields are unwrapped until a ``Pose`` is reached.
    """
    while not hasattr(msg, 'position'):
        msg = msg.pose
    return (float(msg.position.x), float(msg.position.y), yaw_from_quaternion(msg.orientation))


def pose_stamped(stamp, frame: str, p: Sequence[float]) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose = pose_to_msg(p)
    return msg


def transform_stamped(stamp, parent: str, child: str, p: Sequence[float]) -> TransformStamped:
    """TF from ``parent`` to ``child``: ``p`` is the pose of ``child`` in ``parent``."""
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(p[0])
    msg.transform.translation.y = float(p[1])
    msg.transform.rotation = _quaternion(float(p[2]))
    return msg


def pose_from_transform(msg) -> Pose2D:
    """Planar pose of a ``TransformStamped`` or ``Transform`` (child seen from parent)."""
    if hasattr(msg, 'transform'):
        msg = msg.transform
    return (float(msg.translation.x), float(msg.translation.y), yaw_from_quaternion(msg.rotation))
