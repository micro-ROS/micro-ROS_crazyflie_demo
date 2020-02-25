#FROM microros/base:dashing

# TODO: remove when feature/mempools_refactor is merged.
FROM ros:dashing
RUN mkdir -p uros_ws
WORKDIR uros_ws
RUN git clone --depth 1 -b feature/mempools_refactor https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build \
&&  . /opt/ros/$ROS_DISTRO/setup.sh \
&&  apt update \
&&  apt install ed \
&&  rosdep update \
&&  rosdep install --from-paths src --ignore-src -y \
&&  colcon build \
&&  rm -rf log/ build/ src/* \
&&  rm -rf /var/lib/apt/list/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  . ./install/setup.sh \
&&  apt update \
&&  apt install -y dfu-util \
&&  ros2 run micro_ros_setup create_firmware_ws.sh freertos crazyflie21 \
&&  rm -rf /var/lib/apt/lists/*