FROM ros:foxy

RUN apt update \
&&  apt-get install -y ros-${ROS_DISTRO}-rqt* \
&&  rm -rf /var/lib/app/lists/*

COPY ./gcs.perspective ./
COPY ./entrypoint.sh ./

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" ]