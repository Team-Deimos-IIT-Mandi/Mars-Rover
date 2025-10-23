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

##Using Pre-built Docker Image

#Requirements

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



# 🚀 Mars Rover - Jetson Nano Deployment Guide

## Prerequisites on Jetson Nano

1. **Install Docker** (if not already installed):
```bash
# Install Docker
sudo apt-get update
sudo apt-get install -y docker.io

# Add your user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Enable Docker service
sudo systemctl enable docker
sudo systemctl start docker
```

2. **Verify Docker is working**:
```bash
docker --version
docker run hello-world
```

---

## 🎯 Quick Start (Pull and Run)

### For Production (Main Branch):
```bash
# Pull the latest Jetson Nano optimized image
docker pull teamdeimosiitmd/mars_rover:jetson-nano

# Run in development mode (interactive shell)
docker run -it --rm \
  --runtime nvidia \
  --privileged \
  --network host \
  -v /dev:/dev \
  --name mars_rover \
  teamdeimosiitmd/mars_rover:jetson-nano

# Inside the container, you can run:
roslaunch rover_description display.launch
roslaunch rover_gazebo rover_world.launch
```

### For Development (Anish-Part Branch):
```bash
docker pull teamdeimosiitmd/mars_rover:dev-latest

docker run -it --rm \
  --runtime nvidia \
  --privileged \
  --network host \
  -v /dev:/dev \
  --name mars_rover_dev \
  teamdeimosiitmd/mars_rover:dev-latest
```

---

## 🔧 Running Modes

### Mode 1: Development Shell (Default)
```bash
docker run -it --rm \
  --runtime nvidia \
  --privileged \
  --network host \
  teamdeimosiitmd/mars_rover:jetson-nano
```

### Mode 2: ROS Core + ROSBridge (for web interfaces)
```bash
docker run -it --rm \
  --runtime nvidia \
  --privileged \
  --network host \
  teamdeimosiitmd/mars_rover:jetson-nano roscore
```

---

## 🛠️ Persistent Container (Recommended for Development)

If you want to keep your changes and data:

```bash
# Create and run a named container
docker run -it \
  --runtime nvidia \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v ~/mars_rover_data:/root/data \
  --name mars_rover_persistent \
  teamdeimosiitmd/mars_rover:jetson-nano

# Stop the container
docker stop mars_rover_persistent

# Restart the container later
docker start -i mars_rover_persistent

# Remove when done
docker rm mars_rover_persistent
```

---

## 📦 Common ROS Commands Inside Container

```bash
# List all ROS topics
rostopic list

# View rover in RViz
roslaunch rover_description display.launch

# Check ROS environment
rospack list | grep rover
rosnode list
```

---

## 🔍 Troubleshooting

### Issue: "docker: Error response from daemon: could not select device driver"
**Solution**: NVIDIA runtime not installed. Run:
```bash
# Install nvidia-docker2
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### Issue: Permission denied for /dev devices
**Solution**: Use `--privileged` flag or specific device mapping:
```bash
docker run -it --rm \
  --device=/dev/ttyUSB0 \
  --device=/dev/video0 \
  teamdeimosiitmd/mars_rover:jetson-nano
```

### Issue: Out of memory during runtime
**Solution**: Close unnecessary applications and use memory limit:
```bash
docker run -it --rm \
  --runtime nvidia \
  --memory="3g" \
  --memory-swap="4g" \
  teamdeimosiitmd/mars_rover:jetson-nano
```

### Issue: Display not working in container
**Solution**: Allow X11 forwarding:
```bash
xhost +local:docker

docker run -it --rm \
  --runtime nvidia \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  teamdeimosiitmd/mars_rover:jetson-nano
```

---

## 🚀 Performance Tips for Jetson Nano

1. **Enable maximum performance mode**:
```bash
sudo nvpmodel -m 0  # 10W mode (max performance)
sudo jetson_clocks   # Max out CPU/GPU clocks
```

2. **Monitor system resources**:
```bash
# On Jetson Nano (outside container)
jtop  # If installed
htop
```

3. **Reduce memory usage**:
- Close GUI applications when not needed
- Use `catkin build` with `--mem-limit 80%` (already configured in Dockerfile)
- Disable swap if causing slowdowns

---

## 🔄 Updating to Latest Image

```bash
# Pull latest version
docker pull teamdeimosiitmd/mars_rover:jetson-nano

# Remove old containers
docker stop mars_rover
docker rm mars_rover

# Run new version
docker run -it --runtime nvidia --privileged --network host \
  teamdeimosiitmd/mars_rover:jetson-nano
```

---

## 📊 Image Information

- **Base Image**: `osrf/ros:noetic-desktop-full`
- **ROS Version**: Noetic
- **Architectures**: AMD64, ARM64 (Jetson Nano optimized)
- **Compiler Optimizations**: `-march=armv8-a -mtune=cortex-a57 -O3`
- **Pre-installed Packages**: Navigation, SLAM, Gazebo, RViz, ROSBridge

---

## 🆘 Need Help?

- Check container logs: `docker logs mars_rover`
- Access running container: `docker exec -it mars_rover bash`
- Inspect image: `docker inspect teamdeimosiitmd/mars_rover:jetson-nano`
