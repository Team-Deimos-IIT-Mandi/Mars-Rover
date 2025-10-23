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

## Building from Source (without Docker)

If you want to build the project from source instead of using Docker:

### 1. Install ROS Noetic
Follow the official ROS Noetic installation guide: [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 2. Create a Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 3. Clone the Repository
```bash
git clone https://github.com/Team-Deimos-IIT-Mandi/Mars-Rover.git
cd ~/catkin_ws
```

### 4. Install Dependencies using rosdep
```bash
# Initialize rosdep if not already done
sudo rosdep init
rosdep update

# Add custom rosdep source for Mars Rover dependencies
echo "yaml file://$(pwd)/src/Mars-Rover/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-mars-rover.list
rosdep update

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Workspace
```bash
cd ~/catkin_ws
catkin build  # or catkin_make
source devel/setup.bash
```

## Using Pre-built Docker Image

### Requirements

Before running the simulation, install:
1. Docker
- Download and install Docker from:  
  [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/)

- Verify installation:
  ```bash
  docker --version
```
2. VcXsrv (for GUI)
Download VcXsrv from:
https://sourceforge.net/projects/vcxsrv/

After installation, launch XLaunch with:
Display number: 0
Start no client

#Pull the Docker Image
```bash
docker pull teamdeimosiitmd/mars_rover:latest
```

#Run the Simulation
Once Docker and VcXsrv are running:
```bash
docker run -it  --network=host  --env="ROS_MASTER_URI=http://localhost:11311"  --env="ROS_HOSTNAME=localhost"  --env="DISPLAY=host.docker.internal:0.0"   --env="LIBGL_ALWAYS_INDIRECT=0" -v /tmp/.X11-unix:/tmp/.X11-unix teamdeimosiitmd/mars_rover:latest
```

#Exec into the container
```bash
docker exec -it <container_name_or_id> /bin/bash
```

## Working instructions

To launch the rover in Gazebo along with Aruco markers, detect them, perform spiral search (not correctly working) and teleoperate using keyboard/joystick.

```bash
roslaunch rover_description main.launch
```

