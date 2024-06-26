cmake_minimum_required(VERSION 3.8)
project(node_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) ## For interfaces
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
## For the services
find_package(wbr914_package REQUIRED)

include_directories(
include/node_examples
)

# Nodes executables
add_executable(wbr914_velocity_publisher_basic 
src/velocity_publisher_basic.cpp
include/node_examples/velocity_publisher_basic.h) 

add_executable(wbr914_velocity_publisher_continuous_basic 
src/velocity_publisher_continuous_basic.cpp
include/node_examples/velocity_publisher_continuous_basic.h) 

add_executable(wbr914_velocity_publisher_teleop_key 
src/velocity_publisher_teleop_key.cpp
include/node_examples/velocity_publisher_teleop_key.h) 

add_executable(wbr914_wiggle 
src/wiggle_robot.cpp
include/node_examples/wiggle_robot.h) 

# Add needed ROS packages to the executables
ament_target_dependencies(wbr914_velocity_publisher_basic rclcpp geometry_msgs)
ament_target_dependencies(wbr914_velocity_publisher_continuous_basic rclcpp geometry_msgs)
ament_target_dependencies(wbr914_wiggle rclcpp geometry_msgs)
ament_target_dependencies(wbr914_velocity_publisher_teleop_key rclcpp geometry_msgs)

install(TARGETS
 wbr914_velocity_publisher_basic
 wbr914_velocity_publisher_continuous_basic
 wbr914_wiggle
 wbr914_velocity_publisher_teleop_key
  DESTINATION lib/${PROJECT_NAME})


ament_package()
