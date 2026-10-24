import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Localizer and Teach. Start the simulation (or the robot) first:
#   ros2 launch rota_common sim_launch.py
#   ros2 launch rota_control teach_launch.py mode:=mixed map_dir:=data/map rviz:=true


def generate_launch_description():
    mode, tau_d, tau_theta = (LaunchConfiguration(n) for n in ('mode', 'tau_d', 'tau_theta'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = os.path.join(get_package_share_directory('rota_control'), 'config', 'teach.rviz')
    return LaunchDescription([
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        DeclareLaunchArgument('mode', default_value='encoders'),
        DeclareLaunchArgument('tau_d', default_value='0.30'),
        DeclareLaunchArgument('tau_theta', default_value='0.30'),
        DeclareLaunchArgument('map_dir', default_value='data/map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='rota_localization', executable='localization_node', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'mode': mode,
                          'tau_d': tau_d, 'tau_theta': tau_theta}]),
        Node(package='rota_control', executable='teach_node', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'mode': mode,
                          'tau_d': tau_d, 'tau_theta': tau_theta,
                          'map_dir': LaunchConfiguration('map_dir')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
