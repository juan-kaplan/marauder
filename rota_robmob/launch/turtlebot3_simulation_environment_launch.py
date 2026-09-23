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


    declare_robot_model_arg = DeclareLaunchArgument('robot_model', default_value='burger')
    robot_model_arg = LaunchConfiguration('robot_model')

    env_turtlebot3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value=robot_model_arg)

    turtle_sim = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
        additional_env={'TURTLEBOT3_MODEL': robot_model_arg},
        output='screen'
    )

    static_tf_map_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_odom',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            ]
            )

    teleop_keyboard_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_arg}],
        prefix=['xterm -e'],
    )


    declare_enable_rviz_arg = DeclareLaunchArgument('enable_rviz', default_value='true')
    enable_rviz_arg = LaunchConfiguration('enable_rviz')

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
        parameters=[{'use_sim_time': use_sim_time_arg}],
        condition=IfCondition(enable_rviz_arg) 
    )


    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)

    ld.add_action(declare_robot_model_arg)
    ld.add_action(env_turtlebot3_model)
    ld.add_action(turtle_sim)
    ld.add_action(static_tf_map_odom_node)
    ld.add_action(teleop_keyboard_node)

    ld.add_action(declare_enable_rviz_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(rviz_node)

    return ld