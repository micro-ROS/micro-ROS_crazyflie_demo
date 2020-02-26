#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/crazyflie/install/setup.bash"

exec ros2 run crazyflie2rviz translator