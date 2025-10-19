# CMake generated Testfile for 
# Source directory: /Mars-Rover/src/rover_control
# Build directory: /Mars-Rover/build/rover_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rover_control_roslaunch-check_launch "/Mars-Rover/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/Mars-Rover/build/test_results/rover_control/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /Mars-Rover/build/test_results/rover_control" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/Mars-Rover/build/test_results/rover_control/roslaunch-check_launch.xml\" \"/Mars-Rover/src/rover_control/launch\" ")
set_tests_properties(_ctest_rover_control_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/Mars-Rover/src/rover_control/CMakeLists.txt;10;roslaunch_add_file_check;/Mars-Rover/src/rover_control/CMakeLists.txt;0;")
