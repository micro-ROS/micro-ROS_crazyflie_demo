FROM ros:galactic

ENV DEBIAN_FRONTEND noninteractive

RUN apt update \
&&  apt install -y ros-galactic-desktop \
&&  apt install -y ros-$ROS_DISTRO-rmw-fastrtps-cpp \
&&  apt remove -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
&&  rm -rf /var/lib/apt/lists/*

WORKDIR /crazyflie
COPY ./src ./src

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  rosdep update \
&&  rosdep install --from-paths src --ignore-src -r -y \
&&  colcon build --symlink-install

ENV DEBIAN_FRONTEND dialog

COPY disable_fastdds_shm.xml /tmp/
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]