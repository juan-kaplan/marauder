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
    declare_trajectory_file = DeclareLaunchArgument(
        'trajectory_file',
        default_value='data/trajectory.json',
        description='Path to recorded trajectory JSON file'
    )
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic name for output velocity commands'
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic name for wheel odometry'
    )
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for live 2D LiDAR scans'
    )
    declare_use_stamped_vel = DeclareLaunchArgument(
        'use_stamped_vel',
        default_value='true',
        description='Publish TwistStamped instead of Twist (required for ROS 2 Jazzy Gazebo)'
    )
    declare_start_odometry = DeclareLaunchArgument(
        'start_odometry',
        default_value='false',
        description='Launch rota_robmob wheel odometry node (true if not running Gazebo / if using bag)'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with repeat visualization configuration'
    )

    repeat_node = Node(
        package='rota_control',
        executable='repeat_node',
        name='repeat_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'trajectory_file': LaunchConfiguration('trajectory_file'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'use_stamped_vel': LaunchConfiguration('use_stamped_vel'),
            'k_rho': 2.5,
            'k_alpha': 6.0,
            'k_beta': -1.2,
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'keyframe_advance_dist': 0.25,
            'dist_tolerance': 0.08,
            'angle_tolerance': 0.10,
            'icp_max_distance': 0.30,
            'icp_min_fitness': 0.25,
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
        rviz_config = os.path.join(get_package_share_directory('rota_control'), 'config', 'repeat.rviz')
    except Exception:
        rviz_config = 'src/marauder/rota_control/config/repeat.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_repeat',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_trajectory_file,
        declare_cmd_vel_topic,
        declare_odom_topic,
        declare_scan_topic,
        declare_use_stamped_vel,
        declare_start_odometry,
        declare_rviz,
        rota_odometry_node,
        repeat_node,
        rviz_node,
    ])


