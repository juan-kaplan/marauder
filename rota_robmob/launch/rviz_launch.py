#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time_arg = LaunchConfiguration('use_sim_time')

    rota_robmob_pkg_share_dir = get_package_share_directory('rota_robmob')
    rviz_config_default = os.path.join(rota_robmob_pkg_share_dir, 'config', 'config.rviz')

    declare_rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=rviz_config_default)
    rviz_config_arg = LaunchConfiguration('rviz_config')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_arg],
        parameters=[{'use_sim_time': use_sim_time_arg}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(rviz_node)

    return ld