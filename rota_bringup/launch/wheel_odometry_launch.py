from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Wheel odometry on /wheel_odom. Start the simulation (or the robot) first:
#   ros2 launch rota_bringup wheel_odometry_launch.py publish_tf:=true


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(package='rota_localization', executable='wheel_odometry_node', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'publish_tf': LaunchConfiguration('publish_tf')}]),
    ])
