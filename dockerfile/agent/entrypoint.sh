#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/uros_ws/install/setup.bash"

exec ros2 run micro_ros_agent micro_ros_agent serial --dev $SERIAL_DEV -v6