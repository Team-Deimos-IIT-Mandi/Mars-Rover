# ========== Multi-Architecture Build for AMD64 and ARM64 (Jetson Nano) ==========
# Use architecture-specific base images that actually exist
ARG TARGETARCH

# Define base images for each architecture
FROM --platform=linux/arm64 arm64v8/ros:noetic AS base-arm64
FROM --platform=linux/amd64 osrf/ros:noetic-desktop-full AS base-amd64

# Select the correct base based on target architecture
FROM base-${TARGETARCH} AS final

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

# ========== Install System Dependencies (Staged for Memory Efficiency) ==========
# Stage 1: Build tools and Python
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl gnupg2 lsb-release software-properties-common \
    python3-pip python3-dev python3-catkin-tools python3-rosdep python3-psutil \
    python3-rosinstall python3-rosinstall-generator python3-wstool \
    nano vim htop net-tools iputils-ping ca-certificates \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Stage 2: Core ROS packages (split for ARM64 memory constraints)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-xacro \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Stage 3: Navigation and control packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gmapping ros-${ROS_DISTRO}-move-base ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-amcl ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-realtime-tools ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-smach ros-${ROS_DISTRO}-smach-ros \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Stage 4: Vision and perception (heavy packages)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev python3-opencv \
    libpcl-dev ros-${ROS_DISTRO}-pcl-ros \
    libatlas-base-dev libeigen3-dev \
    libgoogle-glog-dev libsuitesparse-dev libboost-all-dev \
    libceres-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Stage 5: Visualization (AMD64 only - too heavy for ARM64 emulation)
RUN if [ "${TARGETARCH}" = "amd64" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rviz \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-ros-control \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-interactive-marker-twist-server \
        ros-${ROS_DISTRO}-vision-opencv \
        && apt-get clean && rm -rf /var/lib/apt/lists/*; \
    else \
        echo "Skipping heavy visualization packages for ARM64"; \
    fi

# ========== Setup ROS Workspace ==========
RUN rosdep init || true && rosdep update
RUN mkdir -p ${CATKIN_WS}/src
COPY . ${CATKIN_WS}/src/Mars-Rover

# Remove problematic packages that don't build on ARM64
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        cd ${CATKIN_WS}/src/Mars-Rover && \
        rm -rf viso2 VINS-Mono libviso2 || true; \
    fi

# ========== Build Catkin Workspace (Architecture-Optimized) ==========
RUN cd ${CATKIN_WS} && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && \
    catkin config --extend /opt/ros/${ROS_DISTRO} && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo '=== Installing rosdep dependencies ===' && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    echo '=== Building workspace for ${TARGETARCH} ===' && \
    if [ '${TARGETARCH}' = 'arm64' ]; then \
        catkin build -j2 --no-status --continue-on-failure || catkin build -j1 --no-status; \
    else \
        catkin build -j\$(nproc) --no-status; \
    fi && \
    echo '=== Cleaning up build artifacts ===' && \
    rm -rf build logs .catkin_tools"

# ========== Environment Setup ==========
RUN mkdir -p /root/.ros/log && \
    if [ "$TARGETARCH" != "arm64" ]; then mkdir -p /root/.gazebo; fi

# Setup environment variables with architecture-specific optimizations
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc && \
    echo "export ROS_IP=127.0.0.1" >> /root/.bashrc && \
    if [ "$TARGETARCH" = "amd64" ]; then \
        echo "export GAZEBO_MODEL_PATH=${CATKIN_WS}/src/Mars-Rover/rover_description/models:/usr/share/gazebo-11/models" >> /root/.bashrc; \
    fi && \
    if [ "$TARGETARCH" = "arm64" ]; then \
        echo "# Jetson Nano ARM64 runtime optimizations" >> /root/.bashrc && \
        echo "export OPENBLAS_CORETYPE=ARMV8" >> /root/.bashrc && \
        echo "export OPENBLAS_NUM_THREADS=4" >> /root/.bashrc && \
        echo "export OMP_NUM_THREADS=4" >> /root/.bashrc; \
    fi

# ========== Create Entrypoint Script ==========
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS environment\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${CATKIN_WS}/devel/setup.bash\n\
\n\
# Detect architecture\n\
ARCH=$(uname -m)\n\
if [ "$ARCH" = "aarch64" ]; then\n\
  PLATFORM="Jetson Nano (ARM64)"\n\
else\n\
  PLATFORM="x86_64 (AMD64)"\n\
fi\n\
\n\
# Display system information\n\
echo "================================="\n\
echo "🚀 Mars Rover Control System"\n\
echo "================================="\n\
echo "Platform: $PLATFORM"\n\
echo "Architecture: $ARCH"\n\
echo "ROS Distribution: ${ROS_DISTRO}"\n\
echo "Workspace: ${CATKIN_WS}"\n\
echo "================================="\n\
\n\
MODE=${1:-dev}\n\
\n\
if [ "$MODE" = "dev" ]; then\n\
  echo "🛠️  Launching Development Mode (Bash Shell)"\n\
  echo ""\n\
  echo "Quick commands:"\n\
  echo "  roslaunch rover_description display.launch    # Visualize in RViz"\n\
  if [ "$ARCH" != "aarch64" ]; then\n\
    echo "  roslaunch rover_gazebo rover_world.launch     # Launch Gazebo simulation"\n\
  fi\n\
  echo "  rostopic list                                 # List available topics"\n\
  echo ""\n\
  exec bash\n\
elif [ "$MODE" = "roscore" ]; then\n\
  echo "🌐 Launching ROS Core + ROSBridge"\n\
  roscore &\n\
  ROSCORE_PID=$!\n\
  echo "Started roscore (PID: $ROSCORE_PID)"\n\
  \n\
  # Wait for roscore with timeout\n\
  timeout=30\n\
  elapsed=0\n\
  until rostopic list > /dev/null 2>&1; do\n\
    if [ $elapsed -ge $timeout ]; then\n\
      echo "❌ ERROR: roscore failed to start within ${timeout}s"\n\
      kill $ROSCORE_PID 2>/dev/null || true\n\
      exit 1\n\
    fi\n\
    echo "⏳ Waiting for roscore... (${elapsed}s)"\n\
    sleep 1\n\
    elapsed=$((elapsed + 1))\n\
  done\n\
  echo "✅ ROS Master is ready!"\n\
  \n\
  roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 &\n\
  ROSBRIDGE_PID=$!\n\
  echo "Started rosbridge (PID: $ROSBRIDGE_PID)"\n\
  \n\
  trap "echo Shutting down...; kill $ROSBRIDGE_PID $ROSCORE_PID 2>/dev/null; wait; exit 0" SIGTERM SIGINT\n\
  \n\
  echo "================================="\n\
  echo "✅ Services Running:"\n\
  echo "   ROS Bridge: ws://localhost:9090"\n\
  echo "   ROS Master: http://localhost:11311"\n\
  echo "================================="\n\
  \n\
  wait\n\
else\n\
  echo "❌ Unknown mode: $MODE"\n\
  echo "Available modes: dev, roscore"\n\
  exit 1\n\
fi\n' > /entrypoint.sh && chmod +x /entrypoint.sh

EXPOSE 9090 11311
WORKDIR ${CATKIN_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["dev"]