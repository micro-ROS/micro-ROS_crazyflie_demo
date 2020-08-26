#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/uros_ws/install/setup.bash"

/wait-for localhost:1189 -t 5
socat TCP:localhost:1189 OPEN:/variables.env,creat

source /variables.env
source /serial_dev

ros2 run micro_ros_agent micro_ros_agent serial --dev $SERIAL_DEV --verbose 6 --refs /agent.refs