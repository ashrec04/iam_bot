#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    iam_bot_dir = get_package_share_directory('iam_bot')
    mapping_dir = get_package_share_directory('iam_navigation')

    mapping_launch_dir = os.path.join(mapping_dir, 'launch')
    mapping_launch_file = os.path.join(mapping_launch_dir, 'mapping_launch.py')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(iam_bot_dir, 'launch', 'iam_gazebo_launch.py'))
    )
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_file)
        )

    return LaunchDescription([
        gazebo_launch,
        mapping_launch
        ])
