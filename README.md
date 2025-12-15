# **IAM Bot**
IAM Bot is a ROS2 Jazzy workspace designed to launch and simulate a mobile robot in Gazebo (Harmonic).
It includes environment mapping and navigation using SLAM

# **Robot Features**
- Fully defined **URDF robot model** with differential drive wheels, lidar & camera. :contentReference[oaicite:1]{index=1}
- **SLAM-ready** configuration for mapping, localisation and navigation. :contentReference[oaicite:2]{index=2}
- **Gazebo simulation** with world assets and physics. :contentReference[oaicite:3]{index=3}
- **Camera** for facial recognition
- **Lidar** for environment perception :contentReference[oaicite:4]{index=4}

## **Packet Structure**
```
robot_workspace
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
│   ├── include/
│   ├── install/
│   ├── launch
│   │   └── iam_gazebo_launch.py
│   ├── meshes
│   │   ├── base_link_col.stl
│   │   ├── ...
│   │   └── top_lidar_sensor.stl
│   ├── package.xml
│   ├── urdf
│   │   └── iam_bot.urdf
│   └── worlds
│       ├── materials/
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
- `face_detector` — Uses the robots camera for face detection :contentReference[oaicite:6]{index=6}
- `iam_bot` — core robot model, launch rviz and gazebo files and simulation config :contentReference[oaicite:7]{index=7}
- `iam_navigation` — mapping,navigation SLAM config and launch setups :contentReference[oaicite:8]{index=8}

## **IAM Bot's Tree Structure**
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
- **ROS2 Jazzy** (installed & sourced)
- **Gazebo Harmonic**
- `colcon` build tools
- ROS2 packages:
  - `ros-jazzy-ros-gz`
  - `ros-jazzy-ros-gz-bridge`
  - `ros-jazzy-slam-toolbox`
  - `ros-jazzy-navigation2`
  - `ros-jazzy-joint-state-publisher`
  - `ros-jazzy-joint-state-publisher-gui`
  - OpenCV

## **Running the Package**
This will take you through the steps on how to get iam bot running
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
### 4) Run the iam bot Launch Script
#### Spawn the robot with no existing map
```
ros2 launch iam_bot iam_mapping_launch.py 
```
#### Spawn the robot with an existing map
```
ros2 launch iam_bot iam_navigation_launch.py 
```
