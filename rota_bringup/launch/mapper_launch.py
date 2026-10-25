from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Occupancy grid on /map from /scan and the pose in odom_topic. Start the simulation
# (or the robot) and something that publishes the pose first, for example
#   ros2 launch rota_bringup wheel_odometry_launch.py publish_tf:=true
#   ros2 launch rota_bringup mapper_launch.py odom_topic:=/wheel_odom


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/localization/odom'),
        Node(package='rota_mapping', executable='mapper_node', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'odom_topic': LaunchConfiguration('odom_topic')}]),
    ])
