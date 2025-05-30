#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2ws/install/setup.bash"
# source /usr/local/share/gazebo/setup.bash

exec "$@"
