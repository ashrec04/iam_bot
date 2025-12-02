# **IAM-Bot-ROS2-Package**
This is ROS2 Jazzy workspace consisting of multibple packages capabler of launching a URDF robot in Gazebo with visualisation in rviz

# **Robot Features**
- LIDAR - for SLAM mapping and navigation
- Odometry Sensor - for SLAM mapping and navigation
- Camera - for facial recognition

## **Packet Structure**
```
robot_workspace
├── camera_publisher
│   ├── build
│   ├── CMakeLists.txt
│   ├── config
│   ├── include
│   │   └── camera_publisher
│   ├── package.xml
│   └── src
│       └── camera_publisher_node.cpp
├── face_detector
│   ├── build
│   ├── CMakeLists.txt
│   ├── config
│   ├── include
│   ├── package.xml
│   └── src
│       └── face_detector_node.cpp
├── iam_bot
│   ├── build
│   ├── CMakeLists.txt
│   ├── config
│   ├── include
│   │   └── iam_bot
│   ├── install
│   │   └── iam_bot
│   │       └── share
│   │           ├── colcon-core
│   │           │   └── packages
│   │           └── iam_bot
│   ├── launch
│   │   └── iam_gazebo_launch.py
│   ├── meshes
│   │   ├── base_link_col.stl
│   │   ├── base_link.stl
│   │   ├── left_caster_connector.stl
│   │   ├── left_caster_wheel.stl
│   │   ├── left_wheel.stl
│   │   ├── right_caster_connector.stl
│   │   ├── right_caster_wheel.stl
│   │   ├── right_wheel.stl
│   │   └── top_lidar_sensor.stl
│   ├── package.xml
│   ├── urdf
│   │   └── iam_bot.urdf
│   └── worlds
│       ├── materials
│       │   └── textures
│       │       ├── eyebrow001.png
│       │       ├── ...
│       │       └── young_lightskinned_male_diffuse.png
│       ├── meshes
│       │   ├── coffee_table_col.stl
│       │   ├── ...
│       │   └── world_window_two.stl
│       ├── model.config
│       └── room.world
├── iam_navigation
│   ├── CMakeLists.txt
│   ├── config
│   │   └── slam_toolbox_mapping.yaml
│   ├── include
│   │   └── iam_navigation
│   ├── launch
│   │   └── iam_navigation_launch.py
│   ├── LICENSE
│   ├── package.xml
│   ├── rviz
│   │   └── rviz.rviz
│   └── src
└── README.md


```



## **Package Contents**
- Complete Robot structure using stl's and a URDF file
- Python launch file allowing the robot to spawn in the Gazebo environment successfully

## **IAM Bot Tree Structure**
```
base_link
├── top_lidar_sensor
├── left_wheel
├── right_wheel
├── left_caster_connector
│   └── left_caster_wheel
└── right_caster_connector
    └── right_wheel
```

## **Dependencies**
- ROS2 Jazzy
- Gazebo Harmonic
- ros-jazzy-joint-state-publisher
- ros-jazzy-joint-state-publisher-gui
- ros-jazzy-ros-gz
- ros-jazzy-ros-gz-bridge
- ros-jazzy-slam-toolbox
- ros-jazzy-navigation2
- ros-jazzy-teleop-twist-keyboard
- opencv
- ros-jazzy-slam-toolbox
- ros-jazzy-rviz-imu-plugin
- ros-jazzy-nav2-bringup 
- ros-jazzy-nav2-amcl

## **Running the Package**
This will take you through the steps on how to get iam bot running in Gazebo
### 1) Activate the Workspace Environment
```
source /opt/ros/jazzy/setup.bash 
source ~/<insert your workspace name>/install/setup.bash 
```
### 2) Compile the Workspace
```
cd ~/<insert your workspace name>
colcon build --packages-select iam_bot
```
### 3) Run iam bot Launch Script
```
ros2 launch iam_bot iam_gazebo_launch.py 
```
