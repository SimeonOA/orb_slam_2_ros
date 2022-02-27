FROM ros:rolling

# Update
RUN apt update
RUN apt-get install software-properties-common apt-utils -y

# Set working directory
WORKDIR /home/ros/src

# Get the realsense package from git
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta
RUN git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git -b ros2

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


WORKDIR /home/ros


# Install ROS dependencies
RUN rosdep update \
&& rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2


# clone source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# build source

RUN colcon build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install

# setup bashrc
RUN cp /etc/skel/.bashrc ~/

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
