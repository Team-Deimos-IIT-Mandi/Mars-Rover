# ========== Stage 1: Base Image ==========
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    CATKIN_WS=/root/catkin_ws \
    MY_APP_DIR=/root/my-app \
    NODE_VERSION=20 \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    NODE_OPTIONS=--max-old-space-size=2048

# ========== Stage 2: Install Dependencies ==========
RUN apt-get update && apt-get install -y \
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
    # System tools
    nano vim tmux htop net-tools iputils-ping ca-certificates \
    # OpenCV and math libs
    libopencv-dev python3-opencv libatlas-base-dev libeigen3-dev libgoogle-glog-dev libsuitesparse-dev libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# ========== Stage 3: Install Node.js ==========
RUN curl -fsSL https://deb.nodesource.com/setup_${NODE_VERSION}.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g npm@latest && \
    node --version && npm --version && \
    rm -rf /var/lib/apt/lists/*

# ========== Stage 4: Optional - Install Ceres Solver ==========
ARG CERES_VERSION=1.14.0
RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && cmake .. && make -j$(nproc) install && \
    cd ../.. && rm -rf ceres-solver

# ========== Stage 5: Setup ROS Workspace ==========
RUN rosdep init || true && rosdep update

RUN mkdir -p ${CATKIN_WS}/src
COPY catkin_ws/src ${CATKIN_WS}/src

RUN cd ${CATKIN_WS} && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && catkin config --extend /opt/ros/${ROS_DISTRO} && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build -j$(nproc)"

# ========== Stage 6: Setup Next.js Web App ==========
RUN mkdir -p ${MY_APP_DIR}
COPY my-app/package*.json ${MY_APP_DIR}/

WORKDIR ${MY_APP_DIR}
RUN npm ci --prefer-offline --no-audit --maxsockets 1
COPY my-app ${MY_APP_DIR}
RUN npm run build && npm prune --production

# ========== Stage 7: Environment Setup ==========
RUN mkdir -p /root/.ros/log /root/.gazebo
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc && \
    echo "export ROS_IP=127.0.0.1" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=${CATKIN_WS}/src/Mars-Rover/rover_description/models:/usr/share/gazebo-11/models" >> /root/.bashrc

# ========== Stage 8: Entrypoint Script ==========
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${CATKIN_WS}/devel/setup.bash\n\
MODE=${1:-dev}\n\
if [ "$MODE" = "dev" ]; then\n\
  echo "Launching Development Mode (Bash Shell)"\n\
  exec bash\n\
else\n\
  echo "Launching Mars Rover Control System..."\n\
  roscore &\n\
  ROSCORE_PID=$!\n\
  until rostopic list > /dev/null 2>&1; do sleep 1; done\n\
  roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 &\n\
  ROSBRIDGE_PID=$!\n\
  cd ${MY_APP_DIR} && npm start &\n\
  NEXT_PID=$!\n\
  trap \"kill $NEXT_PID $ROSBRIDGE_PID $ROSCORE_PID 2>/dev/null\" SIGTERM SIGINT\n\
  echo "Web UI: http://localhost:3000 | ROS Bridge: ws://localhost:9090"\n\
  wait\n\
fi\n' > /entrypoint.sh && chmod +x /entrypoint.sh

EXPOSE 3000 9090 11311
WORKDIR /root
ENTRYPOINT ["/entrypoint.sh"]
CMD ["dev"]
