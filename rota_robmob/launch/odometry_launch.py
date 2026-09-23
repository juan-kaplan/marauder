#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rota_odometry_node = Node(
        package='rota_robmob',
        executable='odometry',
        name='odometry',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.05,
            'wheel_separation': 0.50,
            'frame_id': 'base_footprint',
            }]
    )

    ld = LaunchDescription()
    ld.add_action(rota_odometry_node)

    return ld