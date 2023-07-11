cmake_minimum_required(VERSION 2.8.3)
project(explain_bt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  behaviortree_cpp
)

find_package(Boost REQUIRED system filesystem date_time thread)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Explain.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    std_msgs
    behaviortree_cpp
)

include_directories( include ${catkin_INCLUDE_DIRS})
include_directories(include SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} )