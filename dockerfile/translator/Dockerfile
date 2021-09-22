FROM ros:galactic

ENV DEBIAN_FRONTEND noninteractive

RUN apt update \
&&  apt install -y ros-galactic-desktop \
&&  rm -rf /var/lib/apt/lists/*

WORKDIR /crazyflie
COPY ./src ./src

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  rosdep update \
&&  rosdep install --from-paths src --ignore-src -r -y \
&&  colcon build --symlink-install

ENV DEBIAN_FRONTEND dialog

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]