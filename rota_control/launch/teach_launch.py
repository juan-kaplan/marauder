#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic name for wheel odometry feedback'
    )
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for 2D LiDAR scan'
    )
    declare_tau_d = DeclareLaunchArgument(
        'tau_d',
        default_value='0.30',
        description='Spatial sampling distance threshold in meters'
    )
    declare_tau_theta = DeclareLaunchArgument(
        'tau_theta',
        default_value='0.30',
        description='Angular sampling threshold in radians (~17 deg)'
    )
    declare_output_file = DeclareLaunchArgument(
        'output_file',
        default_value='data/trajectory.json',
        description='Output path to store trajectory metadata and points'
    )
    declare_start_odometry = DeclareLaunchArgument(
        'start_odometry',
        default_value='false',
        description='Launch rota_robmob wheel odometry node (true if not running Gazebo / if using bag)'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with teach visualization configuration'
    )

    teach_node = Node(
        package='rota_control',
        executable='teach_node',
        name='teach_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'tau_d': LaunchConfiguration('tau_d'),
            'tau_theta': LaunchConfiguration('tau_theta'),
            'output_file': LaunchConfiguration('output_file'),
            'auto_record': True,
        }]
    )

    from launch.conditions import IfCondition
    rota_odometry_node = Node(
        package='rota_robmob',
        executable='odometry',
        name='differential_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_radius': 0.05,
            'wheel_separation': 0.50,
            'frame_id': 'base_footprint',
            'parent_frame_id': 'odom',
        }],
        condition=IfCondition(LaunchConfiguration('start_odometry'))
    )

    import os
    from ament_index_python.packages import get_package_share_directory
    try:
        rviz_config = os.path.join(get_package_share_directory('rota_control'), 'config', 'teach.rviz')
    except Exception:
        rviz_config = 'src/marauder/rota_control/config/teach.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_teach',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_odom_topic,
        declare_scan_topic,
        declare_tau_d,
        declare_tau_theta,
        declare_output_file,
        declare_start_odometry,
        declare_rviz,
        rota_odometry_node,
        teach_node,
        rviz_node,
    ])


