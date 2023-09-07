cmake_minimum_required(VERSION 2.8.3)
project(explain_bt)

## Set C++ standard to C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find required packages
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
  std_msgs
)

## Specify package properties and dependencies
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  behaviortree_cpp
)

## Include necessary directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

# add this package's library
add_library(${PROJECT_NAME}
  src/BehaviorTracker.cpp
  src/ExplainableBT.cpp
  src/utils/utils.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION} 
)
install( 
  DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Add executable for example_bt.cpp
add_executable(
  example_bt 
  examples/example_bt.cpp
  src/BehaviorTracker.cpp
  src/ExplainableBT.cpp
  src/utils/utils.cpp
)
add_dependencies(example_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_bt ${catkin_LIBRARIES})