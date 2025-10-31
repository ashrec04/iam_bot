#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = get_package_share_directory('iam_bot')
    urdf_path = os.path.join(pkg_path, 'urdf', 'iam_bot.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'room.world')

    #~~~ start gazebo server and client
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    #~~~ Nodes ~~~#
    #~~~ iam bot gazebo spawn node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_path, '-name', 'iam_bot', '-x', '5.0', '-y', '-2.0', '-z', '2.0'],
        output='screen'
    )

    #~~~ Robot State Publisher node (converts URDF to TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[urdf_path]
    )

    return LaunchDescription([
        gz_launch,
        robot_state_publisher,
        spawn_entity,
    ])