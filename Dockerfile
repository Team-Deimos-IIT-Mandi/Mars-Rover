# =====================================
# Stage 1: Base Image
# =====================================
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# =====================================
# Stage 2 : Install Dependencies 
# =====================================

RUN apt-get update && apt-get install -y \
    # --- Build Tools ---
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    nano \
    \
    # --- Python + ROS Build Tools ---
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    \
    # --- Core ROS Packages ---
    ros-noetic-ros-base \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-message-filters \
    ros-noetic-xacro \
    \
    # --- Vision / Camera ---
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-vision-opencv \
    \
    # --- Gazebo / Simulation (Optional but Useful) ---
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-teleop-twist-joy \
    \
    # --- Navigation / Localization ---
    ros-noetic-gmapping \
    ros-noetic-move-base \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-navigation \
    ros-noetic-robot-localization \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-twist-mux \
    \
    # --- Math & Optimization Libraries ---
    libeigen3-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    \
    && rm -rf /var/lib/apt/lists/*

# =====================================
# Stage 3: Install Ceres Solver (Required for VINS-Mono)
# =====================================
RUN cd /tmp && \
    git clone -b 1.14.0 https://ceres-solver.googlesource.com/ceres-solver && \
    mkdir -p ceres-solver/build && cd ceres-solver/build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/ceres-solver


# Install specific OpenCV version
RUN pip3 install --no-cache-dir opencv-contrib-python==4.2.0.32

# =====================================
# Stage 3: Workspace Setup
# =====================================
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src


# Clone each branch into its own folder
RUN git clone https://github.com/Team-Deimos-IIT-Mandi/Mars-Rover.git $CATKIN_WS/src/Mars-Rover  
    

# Install ROS dependencies from package.xml
RUN rosdep update || true && \
    cd $CATKIN_WS && \
    rosdep install --from-paths src --ignore-src -r -y

# =====================================
# Stage 4: Build the Workspace
# =====================================
RUN bash -c "source /opt/ros/noetic/setup.bash && cd $CATKIN_WS && catkin build"

# =====================================
# Stage 5: Environment Setup
# =====================================
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

# Start bash shell
CMD ["bash"]
