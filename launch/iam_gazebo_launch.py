#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_path = get_package_share_directory('iam_bot')
    urdf_path = os.path.join(pkg_path, 'urdf', 'iam_bot.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'room.world')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'bot.rviz')
    
    rviz = LaunchConfiguration('rviz')


    declare_rviz = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Open rviz is set to True')

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

    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',)]
    )

    #~~~ Robot State Publisher node (converts URDF to TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=([urdf_path])
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','top_lidar_sensor'],
        output='screen'
    )

    return LaunchDescription([
        declare_rviz,
        rviz2,
        gz_launch,
        spawn_entity,
        robot_state_publisher,
        bridge,
        transform_publisher
    ])