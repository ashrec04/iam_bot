import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ~~~~~~~~~~~~~~~~~~~~~~~~~~
# Launch file for autonomous navigation using SLAM toolbox
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~

def generate_launch_description():

    iam_navigation_pkg = get_package_share_directory('iam_navigation')
    gazebo_models_path, ignore_last_dir = os.path.split(iam_navigation_pkg)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path


    #Choose SLAM config based on spawn choice used by iam_gazebo_launch
    spawn_choice = os.environ.get('IAM_SPAWN_CHOICE', '1')
    slam_config_file = 'slam_toolbox_mapping_pose_two.yaml' if spawn_choice == '2' else 'slam_toolbox_mapping_pose_one.yaml'

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='open rviz'
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

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    navigation_params_path = os.path.join(
        iam_navigation_pkg,
        'config',
        'navigation.yaml'
    )

    slam_toolbox_params_path = os.path.join(
        iam_navigation_pkg,
        'config',
        slam_config_file
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([iam_navigation_pkg, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    #path to SLAM Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
                'slam': 'false' 
        }.items()
    )

    return LaunchDescription([
        rviz_launch_arg,
        rviz_config_arg,
        sim_time_arg,
        rviz_node,
        slam_toolbox_launch,
        navigation_launch
        ])