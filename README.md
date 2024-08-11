# Mars Rover - Team Deimos, IIT Mandi


Welcome to the official repository for the Mars Rover developed by Team    Deimos from IIT Mandi for the University Rover Challenge (URC) 2024.

This repository contains all the files and code used to simulate and build our mars rover using Gazebo, ROS, OpenCV.



## System prerequisites


• Operating System: Ubuntu 20.04

• ROS Distribution: ROS Noetic

• Programming languages : C++, Python 3.0 or newer

• Install the OpenCV standard library with additional modules 

• Firstly install 'pip' if not installed already :

```bash
sudo apt-get update
sudo apt install python3-pip

```
• Then, install OpenCV library with the command given below : 

```bash
pip install opencv-contrib-python==4.2.0.32
```



## Repository Structure

- **rover_description** : Contains the URDF of the rover and launch files for spawning the rover in Gazebo.

- **rover_navigation** : Houses the navigation stack components, including planners, configurations and launch files.

- **rover_control** : Contains launch and configuration files for controlling the rover.

- **pwm_controller** : Contains python scripts for controlling the four PWM motor wheels of the rover.

- **marker_detection_rover** : Contains nodes for Aruco marker detection and spiral search.

- **four_wheel_steering_controller** : Package for launching for four wheel steering control for the rover.

- **four_wheel_steering_msgs** : Defines custom message types for controlling the rover's four-wheel steering system.

- **aruco_sim_rover** : Includes world files, launch files and necessary files for simulating Aruco markers in Gazebo.

- **urdf_geometry_parser** : Include scripts that help in parsing and extracting geometric information from the URDF files.



## Working instructions



To launch the rover in Gazebo along with Aruco markers, detect them, perform spiral search (not correctly working) and teleoperate using keyboard/joystick.

```bash
roslaunch rover_description Rover_spawn_with_aruco_detection.launch
```

