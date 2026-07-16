#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/ros2waverover/install/setup.bash" --

# Welcome information
echo "ros2waverover bridge docker image"
echo "---------------------"
echo 'ROS distro: ' Humble
echo "---"
exec "$@"
