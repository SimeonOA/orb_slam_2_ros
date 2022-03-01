FROM ros:rolling-ros-core-focal

# Update
RUN apt-get update && \
  apt-get install -y --no-install-recommends \
  build-essential \
  software-properties-common \
  python3-pip \
  python3-colcon-common-extensions \
  libboost-serialization-dev \
  libeigen3-dev \
  libpng-dev \
  ros-rolling-tf2-geometry-msgs \
  ros-rolling-image-common \
  ros-rolling-image-transport-plugins \
  ros-rolling-cv-bridge \
  libswscale-dev \
  libavdevice-dev \
  libavformat-dev \
  libavcodec-dev \
  libavutil-dev \
  libx264-dev \
  libogg-dev \
  libtheora-dev \
&& rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir numpy autolab_core

# copy source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src/orb_slam_2_ros
WORKDIR $ROS2_WS/src
COPY [ ".", "orb_slam_2_ros/" ]
RUN mv orb_slam_2_ros/extern/* ./

# build source
WORKDIR $ROS2_WS
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/rolling/setup.bash && colcon build --merge-install

# setup entrypoint
COPY ./docker/rolling/ros_entrypoint.sh /

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]
