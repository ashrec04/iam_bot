# **IAM-Bot-ROS2-Package**
This is a ROS2 Jazzy package capable of launching a URDF robot in Gazebo

## **Packet Structure**
```
robot_workspace
├── camera_publisher
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── camera_publisher_node.cpp
├── face_detector
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── face_detector_node.cpp
├── iam_bot
│   ├── build
│   │   └── iam_bot
│   │       └── CMakeFiles
│   │           ├── 3.28.3
│   │           │   ├── CompilerIdC
│   │           │   │   └── tmp
│   │           │   └── CompilerIdCXX
│   │           │       └── tmp
│   │           └── pkgRedirects
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
│   ├── package.xml
│   ├── README.md
│   ├── urdf
│   │   ├── iam_bot.urdf
│   │   └── meshes
│   │       ├── base_link_col.stl
│   │       ├── base_link.stl
│   │       ├── left_caster_connector.stl
│   │       ├── left_caster_wheel.stl
│   │       ├── left_wheel.stl
│   │       ├── right_caster_connector.stl
│   │       ├── right_caster_wheel.stl
│   │       ├── right_wheel.stl
│   │       └── top_lidar_sensor.stl
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
