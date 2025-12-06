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

# Include OpenCV header directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Declare the package executable
add_executable(face_detector_node src/face_detector_node.cpp)

# Link ROS2 dependencies
ament_target_dependencies(face_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Link OpenCV libraries
target_link_libraries(face_detector_node
  ${OpenCV_LIBS}
)

install(TARGETS
  face_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

