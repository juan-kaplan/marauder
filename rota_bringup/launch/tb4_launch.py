from launch import LaunchDescription
from launch_ros.actions import Node

# What the TurtleBot 4 needs next to our nodes: the LiDAR mount on our own /tf_static.
# Our nodes publish their TF on /tf and never read the robot's (/tb4_1/tf), so the Create 3's
# odom -> base_link does not clash with localization_node. Values from the robot's tf_static
# (base_link -> shell_link -> rplidar_link). Start it once, then teach/repeat with robot:=tb4.
#   ros2 launch rota_bringup tb4_launch.py


def generate_launch_description():
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['--x', '-0.04', '--y', '0', '--z', '0.1929', '--yaw', '1.5708',
                        '--frame-id', 'base_link', '--child-frame-id', 'rplidar_link']),
    ])
