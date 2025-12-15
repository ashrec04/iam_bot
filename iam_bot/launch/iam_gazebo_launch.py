#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def GetSpawnLocation():
    # Keep poses aligned with slam_toolbox map start poses (location 2 needs yaw)
    spawn_options = [['-x', '4.5', '-y', '-2.0', '-z', '0.5'],
                     ['-x', '2.5', '-y', '-8.5', '-z', '0.5', '-Y', '0.5']]
    env_choice = os.environ.get('IAM_SPAWN_CHOICE')

    #if choice already loaded from env use it
    if env_choice in ('1', '2'):
        choice = int(env_choice)

    else:
        while True:
            try:
                choice = int(input(
                    "Location 1: {}\nLocation 2: {}\nEnter a spawn location (1 or 2): "
                    .format(spawn_options[0], spawn_options[1])))
                
            except ValueError:
                choice = 0

            if choice in (1, 2):
                os.environ['IAM_SPAWN_CHOICE'] = str(choice)
                break
            print("Invalid option, please enter 1 or 2")

    return spawn_options[choice - 1]


def generate_launch_description():
    pkg_path = get_package_share_directory('iam_bot')
    urdf_path = os.path.join(pkg_path, 'urdf', 'iam_bot.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'room.world')

    #let Gazebo find meshes
    mesh_pkg_dir = os.path.dirname(pkg_path)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get('GZ_SIM_RESOURCE_PATH', '') + os.pathsep + mesh_pkg_dir

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{pkg_path}:" + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f"{pkg_path}:" + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

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
    spawn_location = GetSpawnLocation()
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'iam_bot', *spawn_location],
        output='screen'
    )

    #~~~ Face detector node
    face_detector_node = Node (
        package='face_detector',
        executable='face_detector_node',
        name='face_detector',
        output='screen',
        remappings=[
            ('/camera/image_raw','/camera/image')
        ]
    )

    #~~~ Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )
    
    lidar_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.1','0','0','0','top_lidar_sensor','iam_bot/base_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        robot_state_publisher,
        bridge,
        lidar_frame_publisher,
        gz_launch,
        spawn_entity,
        face_detector_node,
    ])
