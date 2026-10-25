import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Occupancy grid on /map from /scan and the pose in odom_topic. Start the simulation
# (or the robot) and something that publishes the pose first, for example
#   ros2 launch rota_bringup wheel_odometry_launch.py publish_tf:=true
#   ros2 launch rota_bringup mapper_launch.py odom_topic:=/wheel_odom
# On the TurtleBot 4 add robot:=tb4 ns:=/tb4_1 to both.


def generate_launch_description():
    robot, ns = LaunchConfiguration('robot'), LaunchConfiguration('ns')
    profile = [os.path.join(get_package_share_directory('rota_bringup'), 'config', ''), robot, '.yaml']
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tb3_sim'),
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('odom_topic', default_value='/localization/odom'),
        Node(package='rota_mapping', executable='mapper_node', output='screen',
             parameters=[profile, {'odom_topic': LaunchConfiguration('odom_topic')}],
             remappings=[('/scan', [ns, '/scan'])]),
    ])
