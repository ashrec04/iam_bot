# **IAM-Bot-ROS2-Package**
This is a ROS2 Jazzy package capable of launching a URDF robot in Gazebo
Mousa
## **Packet Structure**
```
iam_bot
├── CMakeLists.txt            #Build Config
├── config
├── include
│   └── iam_bot
├── launch
│   └── iam_gazebo_launch.py
├── package.xml               #Package context
├── src
└── urdf
    ├── iam_bot.urdf          #Robot structure
    └── meshes                #stl meshes refrenced in the URDF
        ├── base_link.stl
        ├── base_link_col.stl
        ├── left_caster_connector.stl
        ├── left_caster_wheel.stl
        ├── left_wheel.stl
        ├── right_caster_connector.stl
        ├── right_caster_wheel.stl
        ├── right_wheel.stl
        └── top_lidar_sensor.stl
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
### 3) Run the Launch Script
```
ros2 launch iam_bot iam_gazebo_launch.py 
```
