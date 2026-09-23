#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    declare_ros_bag_name_arg = DeclareLaunchArgument('ros_bag_name', default_value='true')
    ros_bag_name_arg = LaunchConfiguration('ros_bag_name')

    ros_bag_play_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--clock', '100', ros_bag_name_arg],
        output='screen'
    )


    declare_rota_odometry_arg = DeclareLaunchArgument('rota_odometry', default_value='true')
    rota_odometry_arg = LaunchConfiguration('rota_odometry')

    rota_robmob_odometry = ExecuteProcess(
        cmd=['ros2', 'launch', 'rota_robmob', 'odometry_launch.py'],
        output='screen',
        condition=IfCondition(rota_odometry_arg)
    )


    declare_rota_mapper_arg = DeclareLaunchArgument('rota_mapper', default_value='true')
    rota_mapper_arg = LaunchConfiguration('rota_mapper')

    rota_robmob_mapper = ExecuteProcess(
        cmd=['ros2', 'launch', 'rota_robmob', 'mapper_launch.py'],
        output='screen',
        condition=IfCondition(rota_mapper_arg)
    )


    ld = LaunchDescription([
        declare_ros_bag_name_arg,
        LogInfo(msg=["The value of ros_bag_name is: ", ros_bag_name_arg]),
        ros_bag_play_proc,
        
        declare_rota_odometry_arg,
        LogInfo(msg=["The value of rota_odometry is: ", rota_odometry_arg]),
        rota_robmob_odometry,

        declare_rota_mapper_arg,
        LogInfo(msg=["The value of rota_mapper is: ", rota_mapper_arg]),
        rota_robmob_mapper,
    ])
    
    return ld
