import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Wheel odometry on /wheel_odom. Start the simulation (or the robot) first:
#   ros2 launch rota_bringup wheel_odometry_launch.py publish_tf:=true
#   ros2 launch rota_bringup wheel_odometry_launch.py robot:=tb4 ns:=/tb4_1   (TurtleBot 4, ticks)


def generate_launch_description():
    robot, ns = LaunchConfiguration('robot'), LaunchConfiguration('ns')
    profile = [os.path.join(get_package_share_directory('rota_bringup'), 'config', ''), robot, '.yaml']
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tb3_sim'),
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(package='rota_localization', executable='wheel_odometry_node', output='screen',
             parameters=[profile, {'publish_tf': LaunchConfiguration('publish_tf')}],
             remappings=[('/wheel_ticks', [ns, '/wheel_ticks'])]),
    ])
