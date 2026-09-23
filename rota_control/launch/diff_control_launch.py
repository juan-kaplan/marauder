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
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic name for output velocity commands'
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic name for odometry feedback'
    )
    declare_goal_topic = DeclareLaunchArgument(
        'goal_topic',
        default_value='/goal_pose',
        description='Topic name for target goal pose'
    )
    declare_use_stamped_vel = DeclareLaunchArgument(
        'use_stamped_vel',
        default_value='true',
        description='Publish TwistStamped instead of Twist (required for ROS 2 Jazzy Gazebo)'
    )

    diff_control_node = Node(
        package='rota_control',
        executable='diff_control_node',
        name='diff_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'use_stamped_vel': LaunchConfiguration('use_stamped_vel'),
            'k_rho': 3.0,
            'k_alpha': 8.0,
            'k_beta': -1.5,
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'dist_tolerance': 0.05,
            'angle_tolerance': 0.05,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_cmd_vel_topic,
        declare_odom_topic,
        declare_goal_topic,
        declare_use_stamped_vel,
        diff_control_node,
    ])
