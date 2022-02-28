FROM ros:galactic
RUN apt update
RUN apt upgrade -y
RUN apt install -y locales 
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
RUN apt install -y curl gnupg2 lsb-release software-properties-common python3-colcon-common-extensions git ros-galactic-cv-bridge ros-galactic-diagnostic-updater libboost-all-dev
# Set up Galactic keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Set up realsense keys
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
#Add realsense repo
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u
RUN apt-get update
RUN apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg ros-galactic-tf2-geometry-msgs python3-colcon-common-extensions
RUN mkdir -p /home/fog_ws/src
WORKDIR /home/fog_ws
RUN colcon build --merge-install
WORKDIR /home/fog_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta
RUN git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git -b ros2
RUN git clone https://github.com/ros-perception/image_common.git -b galactic
WORKDIR /home/fog_ws
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2
RUN . /opt/ros/galactic/setup.sh && colcon build --merge-install
