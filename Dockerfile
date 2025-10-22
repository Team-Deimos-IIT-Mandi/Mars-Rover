# ========== Multi-Architecture Build for AMD64 and ARM64 (Jetson Nano) ==========
FROM --platform=$TARGETPLATFORM osrf/ros:noetic-desktop-full

# Build arguments for multi-arch support
ARG TARGETPLATFORM
ARG BUILDPLATFORM
ARG TARGETARCH

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    CATKIN_WS=/root/catkin_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Display build information
RUN echo "Building for platform: $TARGETPLATFORM (arch: $TARGETARCH)" && \
    echo "Building on platform: $BUILDPLATFORM" && \
    uname -m

# ========== Stage 2: Install Dependencies ==========
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential cmake git wget curl gnupg2 lsb-release software-properties-common \
    # Python / ROS tools
    python3-pip python3-dev python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    # Core ROS packages
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-vision-opencv ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-gmapping ros-${ROS_DISTRO}-move-base ros-${ROS_DISTRO}-map-server ros-${ROS_DISTRO}-amcl ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-twist-mux \
    # System tools (minimal)
    nano ca-certificates \
    # OpenCV and math libs
    libopencv-dev python3-opencv libatlas-base-dev libeigen3-dev libgoogle-glog-dev libsuitesparse-dev libboost-all-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# ========== Stage 3: Install Ceres Solver (optimized for ARM64) ==========
ARG CERES_VERSION=1.14.0
RUN git clone --depth 1 --branch ${CERES_VERSION} https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    mkdir build && cd build && \
    if [ "$TARGETARCH" = "arm64" ]; then \
        cmake .. \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_EXAMPLES=OFF \
            -DBUILD_TESTING=OFF \
            -DCMAKE_CXX_FLAGS="-march=armv8-a -mtune=cortex-a57" \
            -DEIGEN_INCLUDE_DIR_HINTS=/usr/include/eigen3; \
    else \
        cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF; \
    fi && \
    make -j$(nproc) install && \
    cd ../.. && rm -rf ceres-solver

# ========== Stage 4: Setup ROS Workspace ==========
RUN rosdep init || true && rosdep update

RUN mkdir -p ${CATKIN_WS}/src

COPY . ${CATKIN_WS}/src/Mars-Rover

# Add custom rosdep sources for additional packages
RUN echo "yaml file://${CATKIN_WS}/src/Mars-Rover/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/50-mars-rover.list && \
    rosdep update

# Build the catkin workspace (with ARM64 optimizations)
RUN cd ${CATKIN_WS} && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && catkin config --extend /opt/ros/${ROS_DISTRO} && \
    if [ \"$TARGETARCH\" = \"arm64\" ]; then \
        catkin config --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS=\"-march=armv8-a -mtune=cortex-a57 -O3\"; \
    else \
        catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    fi && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build -j$(nproc) --mem-limit 80% && \
    rm -rf build logs"

# ========== Stage 5: Environment Setup ==========
RUN mkdir -p /root/.ros/log /root/.gazebo

# Add architecture-specific environment variables
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc && \
    echo "export ROS_IP=127.0.0.1" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=${CATKIN_WS}/src/Mars-Rover/rover_description/models:/usr/share/gazebo-11/models" >> /root/.bashrc && \
    if [ "$TARGETARCH" = "arm64" ]; then \
        echo "export OPENBLAS_CORETYPE=ARMV8" >> /root/.bashrc && \
        echo "export OPENBLAS_NUM_THREADS=4" >> /root/.bashrc; \
    fi

# ========== Stage 6: Entrypoint Script ==========
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${CATKIN_WS}/devel/setup.bash\n\
\n\
# Display system information\n\
echo "================================="\n\
echo "Mars Rover Control System"\n\
echo "================================="\n\
echo "Architecture: $(uname -m)"\n\
echo "ROS Distribution: ${ROS_DISTRO}"\n\
echo "================================="\n\
\n\
MODE=${1:-dev}\n\
if [ "$MODE" = "dev" ]; then\n\
  echo "Launching Development Mode (Bash Shell)"\n\
  exec bash\n\
else\n\
  echo "Launching Mars Rover Control System..."\n\
  roscore &\n\
  ROSCORE_PID=$!\n\
  echo "Started roscore (PID: $ROSCORE_PID)"\n\
  \n\
  # Wait for roscore with timeout\n\
  timeout=30\n\
  elapsed=0\n\
  until rostopic list > /dev/null 2>&1; do\n\
    if [ $elapsed -ge $timeout ]; then\n\
      echo "ERROR: roscore failed to start within ${timeout}s"\n\
      kill $ROSCORE_PID 2>/dev/null || true\n\
      exit 1\n\
    fi\n\
    echo "Waiting for roscore... (${elapsed}s)"\n\
    sleep 1\n\
    elapsed=$((elapsed + 1))\n\
  done\n\
  echo "ROS Master is ready!"\n\
  \n\
  roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 &\n\
  ROSBRIDGE_PID=$!\n\
  echo "Started rosbridge (PID: $ROSBRIDGE_PID)"\n\
  \n\
  trap "echo Shutting down...; kill $ROSBRIDGE_PID $ROSCORE_PID 2>/dev/null; wait; exit 0" SIGTERM SIGINT\n\
  \n\
  echo "================================="\n\
  echo "Services Running:"\n\
  echo "ROS Bridge: ws://localhost:9090"\n\
  echo "ROS Master: http://localhost:11311"\n\
  echo "================================="\n\
  \n\
  wait\n\
fi\n' > /entrypoint.sh && chmod +x /entrypoint.sh

EXPOSE 9090 11311
WORKDIR /root
ENTRYPOINT ["/entrypoint.sh"]
CMD ["dev"]
