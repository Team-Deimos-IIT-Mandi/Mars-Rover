cmake_minimum_required(VERSION 3.0.2)
project(mars_rover_hw)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  hardware_interface
  controller_manager
  std_msgs
  sensor_msgs
  geometry_msgs
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    controller_manager
    std_msgs
    sensor_msgs
)

## Build configuration
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build hardware interface library
add_library(${PROJECT_NAME}
  src/mars_rover_can_hw.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Build main node executable
add_executable(mars_rover_hw_node
  src/mars_rover_can_hw_node.cpp
)

add_dependencies(mars_rover_hw_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mars_rover_hw_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS ${PROJECT_NAME} mars_rover_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
