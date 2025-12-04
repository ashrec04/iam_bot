import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

#----------------------#
# Launch file for autonomous navigation
#----------------------#

def generate_launch_description():

    pkg_path = get_package_share_directory('iam_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_path)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open rviz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='navigation.rviz',
        description='rviz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to set use_sim_time'
    )

    #path to config file
    config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    #path to SLAM toolbox launch file
    nav2_localisation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    localisation_params_path = os.path.join(
        get_package_share_directory('iam_navigation'),
        'config',
        'localisation.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('iam_navigation'),
        'config',
        'navigation.yaml'
    )

    map_path = os.path.join(
        get_package_share_directory('iam_navigation'),
        'maps',
        'map_save.yaml'
    )

    #launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_path, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[config_file_path],
        output='screen',
    )

    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localisation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localisation_params_path,
                'map': map_path,
        }.items()
    )

    #launch nav2
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(localisation_launch)
    launchDescriptionObject.add_action(navigation_launch)


    return launchDescriptionObject