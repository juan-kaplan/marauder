#!/usr/bin/env python3
"""
The three localization modes side by side on one simulated TB3.

* Gazebo Classic with the scenario's world and the burger model patched so the
  diff-drive plugin does not publish odom -> base_footprint and its odometry
  (the true pose in the world) goes to /ground_truth/odom.
* localization_node in mixed mode owns /tf; lidar and encoders run with their
  topics under /lidar and /encoders.
* All three read /scan_gated, which the drive script fills from /scan except
  while it simulates a blocked LiDAR.

    ros2 launch compare_launch.py scenario:=figure8|corridor gui:=true noise_bias:=0.005 noise_std:=0.002
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_MODEL = 'burger'
TOPICS = ('/tf', '/rel_odom', '/localization/odom', '/localization/reference')


def scenarios():
    tb3_world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world')
    return {  # world, spawn (x, y)
        'figure8': (tb3_world, (-2.0, -0.5)),
        'corridor': (os.path.join(HERE, 'worlds', 'corridor.world'), (0.0, 0.0)),
    }


SDF_PATCHES = [
    ('<publish_odom_tf>true</publish_odom_tf>', '<publish_odom_tf>false</publish_odom_tf>'),
    ('<odometry_frame>odom</odometry_frame>', '<odometry_frame>world</odometry_frame>'),
]
# The ROS 2 diff-drive plugin ignores <odometry_topic>: the topic is renamed with a
# remapping inside its <ros> block.
DIFF_DRIVE_ROS_BLOCK = re.compile(
    r'(<plugin name="turtlebot3_diff_drive"[^>]*>\s*<ros>)(.*?)(</ros>)', re.S)
DIFF_DRIVE_REMAP = '<remapping>odom:=ground_truth/odom</remapping>'


def patched_model():
    model_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                             'models', 'turtlebot3_' + ROBOT_MODEL, 'model.sdf')
    with open(model_sdf) as f:
        sdf = f.read()
    for old, new in SDF_PATCHES:
        if old not in sdf:
            raise RuntimeError(f"Could not patch '{model_sdf}': '{old}' not found")
        sdf = sdf.replace(old, new)
    sdf, n = DIFF_DRIVE_ROS_BLOCK.subn(rf'\g<1>\g<2>{DIFF_DRIVE_REMAP}\g<3>', sdf, count=1)
    if n != 1:
        raise RuntimeError(f"Could not patch '{model_sdf}': diff-drive <ros> block not found")
    patched = tempfile.NamedTemporaryFile('w', prefix='tnr_burger_', suffix='.sdf', delete=False)
    patched.write(sdf)
    patched.close()
    return patched.name


def localization(mode, prefix=None):
    remaps = [('/scan', '/scan_gated')]
    if prefix:
        remaps += [(t, f'/{prefix}{t}') for t in TOPICS]
    return Node(package='rota_localization', executable='localization_node',
                name=f'localization_{mode}', output='screen', remappings=remaps,
                parameters=[{'use_sim_time': True, 'mode': mode, 'encoder_noise_seed': 1,
                             'encoder_noise_bias': LaunchConfiguration('noise_bias'),
                             'encoder_noise_std': LaunchConfiguration('noise_std')}])


def scenario_nodes(context):
    name = LaunchConfiguration('scenario').perform(context)
    if name not in scenarios():
        raise RuntimeError(f"scenario must be one of {list(scenarios())}, got '{name}'")
    world, (x, y) = scenarios()[name]
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    gui = LaunchConfiguration('gui')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo, 'gzserver.launch.py')),
                                 launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo, 'gzclient.launch.py')),
                                 condition=IfCondition(gui)),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', ROBOT_MODEL, '-file', patched_model(),
                        '-x', str(x), '-y', str(y), '-z', '0.01']),
        # odom starts where the robot spawns, so RViz can draw the truth (world) next to the estimates.
        Node(package='tf2_ros', executable='static_transform_publisher', name='world_to_odom',
             arguments=['--x', str(x), '--y', str(y), '--frame-id', 'world', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(HERE, 'rviz', f'{name}.rviz')],
             parameters=[{'use_sim_time': True}], output='log', condition=IfCondition(gui)),
    ]


def generate_launch_description():
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL),
        DeclareLaunchArgument('scenario', default_value='figure8', description='figure8 or corridor'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo client and RViz'),
        DeclareLaunchArgument('noise_bias', default_value='0.005'),
        DeclareLaunchArgument('noise_std', default_value='0.002'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(tb3, 'robot_state_publisher.launch.py')),
                                 launch_arguments={'use_sim_time': 'true'}.items()),
        OpaqueFunction(function=scenario_nodes),
        localization('mixed'),
        localization('lidar', 'lidar'),
        localization('encoders', 'encoders'),
    ])
