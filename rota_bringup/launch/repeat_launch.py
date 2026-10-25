import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Localizer, Repeat and the controller. Start the simulation (or the robot) first:
#   ros2 launch rota_bringup sim_launch.py
#   ros2 launch rota_bringup repeat_launch.py mode:=mixed map_dir:=data/map rviz:=true
# The Repeat starts paused; start it with
#   ros2 service call /repeat_node/run std_srvs/srv/SetBool "{data: true}"
# On the TurtleBot 4 (after tb4_launch.py):
#   ros2 launch rota_bringup repeat_launch.py robot:=tb4 ns:=/tb4_1 mode:=mixed map_dir:=data/map
# The controller follows /repeat/setpoint, which is in the robot frame, so the
# robot's own pose is the identity and the goal needs no transform.


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    robot, ns = LaunchConfiguration('robot'), LaunchConfiguration('ns')
    profile = [os.path.join(get_package_share_directory('rota_bringup'), 'config', ''), robot, '.yaml']
    remaps = [('/scan', [ns, '/scan']), ('/wheel_ticks', [ns, '/wheel_ticks'])]
    rviz_config = os.path.join(get_package_share_directory('rota_bringup'), 'config', 'repeat.rviz')
    return LaunchDescription([
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        DeclareLaunchArgument('mode', default_value='encoders'),
        DeclareLaunchArgument('map_dir', default_value='data/map'),
        DeclareLaunchArgument('robot', default_value='tb3_sim'),
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='rota_localization', executable='localization_node', output='screen',
             parameters=[profile, {'mode': mode}], remappings=remaps),
        Node(package='rota_control', executable='repeat_node', output='screen',
             parameters=[profile, {'map_dir': LaunchConfiguration('map_dir')}],
             remappings=remaps),
        Node(package='rota_control', executable='diff_control_node', output='screen',
             parameters=[profile, {'goal_topic': '/repeat/setpoint',
                                   'cmd_vel_topic': [ns, '/cmd_vel'],
                                   'use_tf': True,
                                   'goal_timeout': 0.5,
                                   'k_rho': 3.0, 'k_alpha': 8.0, 'k_beta': -1.5,
                                   'max_linear_speed': 0.3, 'max_angular_speed': 1.0,
                                   'dist_tolerance': 0.05, 'angle_tolerance': 0.05}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             parameters=[profile], remappings=remaps[:1],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
