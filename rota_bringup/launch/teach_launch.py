import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Localizer and Teach. Start the simulation (or the robot) first:
#   ros2 launch rota_bringup sim_launch.py
#   ros2 launch rota_bringup teach_launch.py mode:=mixed map_dir:=data/map rviz:=true
# On the TurtleBot 4 (after tb4_launch.py):
#   ros2 launch rota_bringup teach_launch.py robot:=tb4 ns:=/tb4_1 mode:=mixed map_dir:=data/map


def generate_launch_description():
    mode, tau_d, tau_theta = (LaunchConfiguration(n) for n in ('mode', 'tau_d', 'tau_theta'))
    robot, ns = LaunchConfiguration('robot'), LaunchConfiguration('ns')
    profile = [os.path.join(get_package_share_directory('rota_bringup'), 'config', ''), robot, '.yaml']
    remaps = [('/scan', [ns, '/scan']), ('/wheel_ticks', [ns, '/wheel_ticks'])]
    rviz_config = os.path.join(get_package_share_directory('rota_bringup'), 'config', 'teach.rviz')
    return LaunchDescription([
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        DeclareLaunchArgument('mode', default_value='encoders'),
        DeclareLaunchArgument('tau_d', default_value='0.30'),
        DeclareLaunchArgument('tau_theta', default_value='0.30'),
        DeclareLaunchArgument('map_dir', default_value='data/map'),
        DeclareLaunchArgument('robot', default_value='tb3_sim'),
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='rota_localization', executable='localization_node', output='screen',
             parameters=[profile, {'mode': mode, 'tau_d': tau_d, 'tau_theta': tau_theta}],
             remappings=remaps),
        Node(package='rota_control', executable='teach_node', output='screen',
             parameters=[profile, {'mode': mode, 'tau_d': tau_d, 'tau_theta': tau_theta,
                                  'map_dir': LaunchConfiguration('map_dir')}],
             remappings=remaps),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             parameters=[profile], remappings=remaps[:1],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
