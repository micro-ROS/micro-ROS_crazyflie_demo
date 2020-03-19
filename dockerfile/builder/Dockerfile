FROM microros/base:dashing

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  . ./install/setup.sh \
&&  apt update \
&&  apt install -y dfu-util \
&&  ros2 run micro_ros_setup create_firmware_ws.sh freertos crazyflie21 \
&&  rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]