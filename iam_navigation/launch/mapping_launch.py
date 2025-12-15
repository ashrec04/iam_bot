import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    iam_navigation_pkg = get_package_share_directory('iam_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(iam_navigation_pkg)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path

    slam_config_file = 'slam_toolbox_mapping_no_map.yaml'

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='mapping.rviz',
        description='rviz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to set use_sim_time'
    )

    # path to interactive marker config file
    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    #path to SLAM toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        iam_navigation_pkg,
        'config',
        slam_config_file
    )

    #launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([iam_navigation_pkg, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    return LaunchDescription([
        rviz_launch_arg,
        rviz_config_arg,
        sim_time_arg,
        rviz_node,
        interactive_marker_twist_server_node,
        slam_toolbox_launch
    ])
