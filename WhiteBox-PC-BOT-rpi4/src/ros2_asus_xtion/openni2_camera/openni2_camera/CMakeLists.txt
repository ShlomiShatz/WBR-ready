cmake_minimum_required(VERSION 3.5)
project(openni2_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(openni2_camera_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_OPENNI2 libopenni2)
if (NOT PC_OPENNI2_FOUND)
  pkg_check_modules(PC_OPENNI2 REQUIRED openni2)
endif()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PC_OPENNI2_INCLUDE_DIRS}
)

link_directories(${PC_OPENNI2_LIBRARY_DIRS})

add_library(openni2_wrapper SHARED
  src/openni2_convert.cpp
  src/openni2_device.cpp
  src/openni2_device_info.cpp
  src/openni2_timer_filter.cpp
  src/openni2_frame_listener.cpp
  src/openni2_device_manager.cpp
  src/openni2_exception.cpp
  src/openni2_video_mode.cpp
)
target_link_libraries(openni2_wrapper
  ${Boost_LIBRARIES}
  ${PC_OPENNI2_LIBRARIES}
)
ament_target_dependencies(openni2_wrapper
  camera_info_manager
  image_transport
  sensor_msgs
  rclcpp
  rclcpp_components
)

#add_executable(test_wrapper test/test_wrapper.cpp)
#target_link_libraries(test_wrapper openni2_wrapper ${Boost_LIBRARIES})

add_library(openni2_camera SHARED
  src/openni2_driver.cpp
)
target_link_libraries(openni2_camera
  openni2_wrapper
  ${Boost_LIBRARIE}
)
ament_target_dependencies(openni2_camera
  camera_info_manager
  image_transport
  openni2_camera_msgs
  sensor_msgs
  rclcpp
)

rclcpp_components_register_node(openni2_camera
  PLUGIN "openni2_wrapper::OpenNI2Driver"
  EXECUTABLE openni2_camera_driver
)

add_executable(list_devices
   src/list_devices.cpp
)
target_link_libraries(list_devices
  openni2_wrapper
)

if (UNIX AND NOT APPLE)
  add_executable(usb_reset src/usb_reset.c)
  set(ADDITIONAL_EXECUTABLES "usb_reset")
endif()

install(
  TARGETS
    list_devices
    openni2_wrapper
    openni2_camera
    ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(openni2_wrapper)
ament_export_dependencies(
  camera_info_manager
  image_transport
  openni2_camera_msgs
  sensor_msgs
  rclcpp
)
ament_package()

