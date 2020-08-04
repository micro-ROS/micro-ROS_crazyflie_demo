#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash

node ros2subscriber/app.js & sleep 1

cd gui && npm start

#-c 'source /opt/ros/foxy/setup.bash && node ros2subscriber app.js & bash'"
