#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    iam_bot_dir = get_package_share_directory('iam_bot')
    mapping_dir = get_package_share_directory('iam_navigation')

    mapping_launch_dir = os.path.join(mapping_dir, 'launch')
    mapping_launch_file = os.path.join(mapping_launch_dir, 'mapping_launch.py')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(iam_bot_dir, 'launch', 'iam_gazebo_launch.py')),
        launch_arguments={'face_detector_enabled': 'false'}.items() # disables face detector for mapping
    )
    
    map_start_delay_arg = DeclareLaunchArgument(
        'mapping_start_delay',
        default_value='6.0',
        description='Delay before starting mapping stack to let Gazebo/TF/clock initialize'
    )
    
    map_start_delay = LaunchConfiguration('mapping_start_delay')


    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_file)
    )

    delayed_map_launch = TimerAction(
        period=map_start_delay,
        actions=[mapping_launch],
    )

    return LaunchDescription([
        map_start_delay_arg,
        gazebo_launch,
        delayed_map_launch
    ])
