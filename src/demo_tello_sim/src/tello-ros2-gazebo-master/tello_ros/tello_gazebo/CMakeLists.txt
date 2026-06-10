cmake_minimum_required(VERSION 3.5)
project(tello_gazebo)

#=============
# Setup
#=============

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon
if ($ENV{CLION_IDE})
  message("Running inside CLion")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/tello_msgs/share/tello_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS}
)

#=============
# Tello Gazebo plugin
#=============

add_library(
  TelloPlugin SHARED
  src/tello_plugin.cpp
)

target_link_libraries(
  TelloPlugin
  ignition-plugin1::register
  ignition-gazebo6::core
  ignition-math6::ignition-math6
)

ament_target_dependencies(
  TelloPlugin
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tello_msgs
)

#=============
# Install
#=============

# Install targets
install(
  TARGETS TelloPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install world and launch files
install(
  DIRECTORY models worlds launch
  DESTINATION share/${PROJECT_NAME}
)

## Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#=============
# Run ament macros
#=============

ament_package()
