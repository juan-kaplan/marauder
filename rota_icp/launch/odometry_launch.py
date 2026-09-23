#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import ExecuteProcess
# from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    lidar_odometry_node = Node(
        package='rota_icp',
        executable='lidar_odometry',
        name='lidar_odometry',
        parameters=[{
            'use_sim_time': True,
            }]
    )

    return LaunchDescription([
        lidar_odometry_node,
    ])