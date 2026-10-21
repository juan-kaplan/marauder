"""
Velocity command publisher that works with both ``Twist`` and ``TwistStamped``.

Which type ``/cmd_vel`` expects depends on the setup, not on our code:

* Humble Gazebo Classic / TurtleBot3 / Create 3 (TB4): ``geometry_msgs/Twist``
* Jazzy Gazebo (gz) bridge: ``geometry_msgs/TwistStamped``

With ``cmd_vel_type='auto'`` the type is taken from whoever is subscribed to the
topic (the robot base or the simulator bridge) the first time we publish.
"""

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

TWIST = 'geometry_msgs/msg/Twist'
TWIST_STAMPED = 'geometry_msgs/msg/TwistStamped'
_ALIASES = {'twist': TWIST, 'twist_stamped': TWIST_STAMPED}


class CmdVelPublisher:

    def __init__(self, node: Node, topic: str, cmd_vel_type: str = 'auto',
                 frame_id: str = 'base_link', qos: int = 10):
        mode = cmd_vel_type.strip().lower()
        if mode != 'auto' and mode not in _ALIASES:
            raise ValueError(
                f"cmd_vel_type must be 'auto', 'twist' or 'twist_stamped', got '{cmd_vel_type}'")
        self._node = node
        self._topic = topic
        self._frame_id = frame_id
        self._qos = qos
        self._pub = None
        self._stamped = None
        if mode != 'auto':
            self._create(_ALIASES[mode])

    @property
    def resolved_type(self) -> str:
        """Message type in use, or 'unresolved' while waiting for a subscriber."""
        if self._pub is None:
            return 'unresolved'
        return TWIST_STAMPED if self._stamped else TWIST

    def _create(self, type_name: str):
        self._stamped = type_name == TWIST_STAMPED
        msg_type = TwistStamped if self._stamped else Twist
        self._pub = self._node.create_publisher(msg_type, self._topic, self._qos)
        self._node.get_logger().info(f"Publishing velocity commands on '{self._topic}' as {type_name}")

    def _resolve(self) -> bool:
        if self._pub is not None:
            return True
        types = {info.topic_type for info in self._node.get_subscriptions_info_by_topic(self._topic)}
        if TWIST_STAMPED in types:
            self._create(TWIST_STAMPED)
        elif TWIST in types:
            self._create(TWIST)
        else:
            self._node.get_logger().warn(
                f"No Twist/TwistStamped subscriber on '{self._topic}' yet; "
                f"not sending velocity commands (set cmd_vel_type to force a type).",
                throttle_duration_sec=2.0)
            return False
        if len(types & {TWIST, TWIST_STAMPED}) > 1:
            self._node.get_logger().warn(
                f"'{self._topic}' has both Twist and TwistStamped subscribers; using TwistStamped.")
        return True

    def publish(self, v: float, omega: float):
        if not self._resolve():
            return
        if self._stamped:
            msg = TwistStamped()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(omega)
        else:
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(omega)
        self._pub.publish(msg)
