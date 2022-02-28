#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/rolling/setup.bash"
source "$ROS2_WS/install/setup.bash"
exec "$@"
