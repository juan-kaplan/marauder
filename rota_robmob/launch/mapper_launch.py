#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rota_mapper_node = Node(
        package='rota_robmob',
        executable='mapper',
        name='mapper',
        parameters=[{
            'use_sim_time': True,
            }]
    )

    ld = LaunchDescription()
    ld.add_action(rota_mapper_node)

    return ld