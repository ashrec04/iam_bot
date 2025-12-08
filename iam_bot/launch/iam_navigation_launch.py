#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    iam_bot_dir = get_package_share_directory('iam_bot')
    nav_dir = get_package_share_directory('iam_navigation')

    nav_launch_dir = os.path.join(nav_dir, 'launch')
    nav_launch_file = os.path.join(nav_launch_dir, 'navigation_launch.py')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(iam_bot_dir, 'launch', 'iam_gazebo_launch.py'))
    )

    nav_start_delay_arg = DeclareLaunchArgument(
        'nav_start_delay',
        default_value='6.0',
        description='Delay before starting nav stack to let Gazebo/TF/clock initialize'
    )

    nav_start_delay = LaunchConfiguration('nav_start_delay')

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file)
        )

    delayed_nav_launch = TimerAction(
        period=nav_start_delay,
        actions=[nav_launch],
    )

    return LaunchDescription([
        nav_start_delay_arg,
        gazebo_launch,
        delayed_nav_launch
        ])
