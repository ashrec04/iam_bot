cmake_minimum_required(VERSION 3.8)
project(face_detector)

# ================================================================
# Compiler settings
# Add common warning flags when using GCC or Clang compilers.
# These help catch potential bugs and improve code quality.
# ================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================================================================
# Find required ROS2 and external dependencies.
#
# ament_cmake   : ROS2 build system
# rclcpp        : C++ client library for ROS2
# sensor_msgs   : Standard ROS2 sensor message types (Image)
# cv_bridge     : Bridge between ROS Image messages and OpenCV
# OpenCV        : Computer vision library used for face detection
# ================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include OpenCV header directories for compilation
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# ================================================================
# Declare the executable for this package.
# face_detector_node.cpp implements the main face detection logic.
# ================================================================
add_executable(face_detector_node src/face_detector_node.cpp)

# ================================================================
# Link ROS2 dependencies to the executable.
# These ensure that ROS2 message types, node APIs, and cv_bridge
# are correctly linked during compilation.
# ================================================================
ament_target_dependencies(face_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Link OpenCV libraries to enable image processing and visualization
target_link_libraries(face_detector_node
  ${OpenCV_LIBS}
)

# ================================================================
# Install the built executable into the appropriate directory so
# that it can be launched via `ros2 run face_detector face_detector_node`
# ================================================================
install(TARGETS
  face_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================================================================
# Mark this package as an ament package.
# This allows ROS2 tools (colcon, ros2 run, ros2 launch, etc.)
# to detect and use the package.
# ================================================================
ament_package()

