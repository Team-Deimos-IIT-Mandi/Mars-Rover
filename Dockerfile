
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/root/catkin_ws


RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl nano unzip \
    python3-pip python3-catkin-tools python3-rosdep \
    libeigen3-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    git clone -b 1.14.0 https://ceres-solver.googlesource.com/ceres-solver && \
    mkdir -p ceres-solver/build && cd ceres-solver/build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/ceres-solver

RUN pip3 install --no-cache-dir opencv-contrib-python==4.2.0.32

RUN mkdir -p $CATKIN_WS/src

RUN git clone https://github.com/Team-Deimos-IIT-Mandi/Mars-Rover.git $CATKIN_WS/src/Mars-Rover

RUN rosdep update || true
RUN bash -c "cd $CATKIN_WS && rosdep install --from-paths src --ignore-src -r -y"

RUN bash -c "source /opt/ros/noetic/setup.bash && cd $CATKIN_WS && catkin build"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]
