#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


#~~~ read URDF and process meshes~~~#
def getURDFContent(pkg_iam_bot_path, mesh_dir):
    with open(os.path.join(pkg_iam_bot_path, 'urdf', 'iam_bot.urdf'), 'r') as f:
        content = f.read()
    
    #~~~replace all mesh paths with directories~~~#
    content = content.replace('meshes/base_link.stl', f'file://{mesh_dir}/base_link.stl')
    content = content.replace('meshes/base_link_col.stl', f'file://{mesh_dir}/base_link_col.stl')
    content = content.replace('meshes/left_caster_connector.stl', f'file://{mesh_dir}/left_caster_connector.stl')
    content = content.replace('meshes/left_caster_wheel.stl', f'file://{mesh_dir}/left_caster_wheel.stl')
    content = content.replace('meshes/left_wheel.stl', f'file://{mesh_dir}/left_wheel.stl')
    content = content.replace('meshes/right_caster_connector.stl', f'file://{mesh_dir}/right_caster_connector.stl')
    content = content.replace('meshes/right_caster_wheel.stl', f'file://{mesh_dir}/right_caster_wheel.stl')
    content = content.replace('meshes/right_wheel.stl', f'file://{mesh_dir}/right_wheel.stl')
    content = content.replace('meshes/top_lidar_sensor.stl', f'file://{mesh_dir}/top_lidar_sensor.stl')

    return content

def generate_launch_description():

    #~~~ Variable Declarations ~~~#  
    pkg_iam_bot = FindPackageShare('iam_bot')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    urdf_file = PathJoinSubstitution([pkg_iam_bot, 'urdf', 'iam_bot.urdf'])

    #~~~ Launch args
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    declare_x_pos  = DeclareLaunchArgument(
        name = 'x_pose',
        default_value = '0.0',
        description = 'x pos'
    )
    
    declare_y_pos = DeclareLaunchArgument(
        name = 'y_pose', 
        default_value = '0.0',
        description = 'y pos'
    )
    
    declare_z_pos = DeclareLaunchArgument(
        name = 'z_pose',
        default_value = '0.1',
        description = 'z pos'
    )
    
    #~~~ Get robot desc with meshes
    pkg_iam_bot_path = get_package_share_directory('iam_bot')
    mesh_dir = os.path.join(pkg_iam_bot_path, 'urdf', 'meshes')
    
    robot_desc = getURDFContent(pkg_iam_bot_path, mesh_dir)
    

    #~~~ Publishers ~~~#
    #~~~ robot state
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    #~~~ joint state
    joint_state_publisher = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        name = 'joint_state_publisher',
        parameters = [{
            'use_sim_time': use_sim_time
        }]
    )
    

    #~~~ Gazebo config ~~~#
    #~~~ set Gz resource path
    pkg_iam_bot_path = get_package_share_directory('iam_bot') 

    #~~~ Set gazebo enviroment vars to find meshes 
    urdf_path = os.path.join(pkg_iam_bot_path, 'urdf')
    set_gz_resource_path = SetEnvironmentVariable(
        name = 'GZ_SIM_RESOURCE_PATH',
        value = urdf_path
    )

    #~~~ set ros package path
    set_ros_pkg_path = SetEnvironmentVariable(
        name='ROS_PACKAGE_PATH', 
        value=pkg_iam_bot_path + ':' + os.environ.get('ROS_PACKAGE_PATH', '')
    )

    #~~~ launch cmd for gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments = {
            'gz_args': '-v 4 empty.sdf'
        }.items()
    )
    
    #~~~ spawn IAM bot
    spawn_entity = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments = [
            '-entity', 'iam_bot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    #~~~ros2 to gazebo bridge
    bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    

    #~~~Create Launch Desc ~~~#
    ld = LaunchDescription()
    
    #~~~launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pos)
    ld.add_action(declare_y_pos)
    ld.add_action(declare_z_pos)
    
    #~~~environment
    ld.add_action(set_gz_resource_path)
    ld.add_action(set_ros_pkg_path)
    
    #~~~nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(spawn_entity)
    
    return ld