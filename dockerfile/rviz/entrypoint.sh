#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/crazyflie/install/setup.bash"

exec ros2 run micro-ros_crazyflie_demo_remote attitude_to_vel & ros2 launch micro-ros_crazyflie_demo_remote launch_drone_position.launch.py