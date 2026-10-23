"""Conversions between (x, y, theta) poses and geometry_msgs."""

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

from rota_common.se2 import quaternion_from_yaw, yaw_from_quaternion


def pose_to_msg(p):
    msg = Pose()
    msg.position.x = float(p[0])
    msg.position.y = float(p[1])
    q = quaternion_from_yaw(float(p[2]))
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
    return msg


def pose_from_msg(msg):
    """Pose of a Pose, PoseStamped or Odometry."""
    while not hasattr(msg, 'position'):
        msg = msg.pose
    return (msg.position.x, msg.position.y, yaw_from_quaternion(msg.orientation))


def pose_stamped(stamp, frame, p):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose = pose_to_msg(p)
    return msg


def transform_stamped(stamp, parent, child, p):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(p[0])
    msg.transform.translation.y = float(p[1])
    q = quaternion_from_yaw(float(p[2]))
    r = msg.transform.rotation
    r.x, r.y, r.z, r.w = q
    return msg


def pose_from_transform(msg):
    t = msg.transform
    return (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
