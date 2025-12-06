#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    iam_bot_dir = get_package_share_directory('iam_bot')
    nav_dir = get_package_share_directory('iam_navigation')

    nav_launch_dir = os.path.join(nav_dir, 'launch')
    nav_launch_file = os.path.join(nav_launch_dir, 'navigation_launch.py')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(iam_bot_dir, 'launch', 'iam_gazebo_launch.py'))
    )
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file)
        )

    return LaunchDescription([
        gazebo_launch,
        nav_launch
        ])
