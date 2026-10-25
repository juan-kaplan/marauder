from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# LiDAR (ICP) odometry on /lidar_odom. Start the simulation (or the robot) first:
#   ros2 launch rota_bringup lidar_odometry_launch.py publish_tf:=true


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(package='rota_localization', executable='lidar_odometry_node', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'publish_tf': LaunchConfiguration('publish_tf')}]),
    ])
