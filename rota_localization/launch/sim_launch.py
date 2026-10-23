import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# TurtleBot3 in Gazebo with its own odom -> base_footprint TF turned off, so
# localization_node is the only parent of base_footprint. Gazebo's /odom topic
# stays on, and works as the ground truth.
#   ros2 launch rota_localization sim_launch.py mode:=mixed


def patched_model():
    path = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                        'models', 'turtlebot3_burger', 'model.sdf')
    with open(path) as f:
        sdf = f.read().replace('<publish_odom_tf>true</publish_odom_tf>',
                               '<publish_odom_tf>false</publish_odom_tf>')
    patched = tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False)
    patched.write(sdf)
    patched.close()
    return patched.name


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),  # the old model server is gone, Gazebo hangs on it
        DeclareLaunchArgument('mode', default_value='encoders'),
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'gzserver.launch.py')),
            launch_arguments={'world': os.path.join(tb3, 'worlds', 'turtlebot3_world.world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'burger', '-file', patched_model(), '-x', '-2.0', '-y', '-0.5', '-z', '0.01']),
        Node(package='rota_localization', executable='localization_node', output='screen',
             parameters=[{'use_sim_time': True, 'mode': LaunchConfiguration('mode')}]),
    ])
