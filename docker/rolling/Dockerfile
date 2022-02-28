FROM ros:rolling

# Update
RUN apt update
RUN apt upgrade -y
RUN apt install -y locales 
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
RUN apt-get install apt-utils curl gnupg2 lsb-release software-properties-common python3-colcon-common-extensions git ros-rolling-cv-bridge ros-rolling-diagnostic-updater libboost-all-dev -y

# Set up Rolling keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Set up realsense keys
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE

#Add realsense repo
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u

# Install required realsense and ROS packages
RUN apt-get update && \
    apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg ros-rolling-tf2-geometry-msgs python3-colcon-common-extensions -y

RUN mkdir -p /home/slamdemo_ws/src
WORKDIR /home/slamdemo_ws

RUN colcon build --merge-install

WORKDIR /home/slamdemo_ws/src

RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta
RUN git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git -b ros2
RUN git clone https://github.com/ros-perception/image_common.git -b ros2


WORKDIR /home/slamdemo_ws

# Install ROS dependencies
RUN rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2

RUN . /opt/ros/rolling/setup.sh && colcon build --merge-install
