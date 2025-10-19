cmake_minimum_required(VERSION 3.0.2)
project(aruco_sim_rover)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(
  DIRECTORY launch meshes urdf materials worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)