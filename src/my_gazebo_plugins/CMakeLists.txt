cmake_minimum_required(VERSION 3.8)
project(my_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# THIS IS FOR THE JOINT STATE MIRROR:

# Compile the plugin
add_library(joint_state_mirror SHARED
  src/joint_state_mirror.cpp
)
target_include_directories(joint_state_mirror
  PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
ament_target_dependencies(joint_state_mirror
  gazebo_ros
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
  nav_msgs
)

# Install the plugin library
install(TARGETS joint_state_mirror
  DESTINATION lib
)

# THIS IS FOR THE ODOM MIRROR:

# Compile the plugin
add_library(odom_mirror SHARED
  src/odom_mirror.cpp
)
target_include_directories(odom_mirror
  PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
ament_target_dependencies(odom_mirror
  gazebo_ros
  rclcpp
  nav_msgs
)

# Install the plugin library
install(TARGETS odom_mirror
  DESTINATION lib
)

# Install additional files if needed (e.g., launch files)
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
