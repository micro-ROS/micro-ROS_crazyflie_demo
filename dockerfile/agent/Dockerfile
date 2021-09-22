FROM microros/micro-ros-agent:galactic

ENV UROS_FILE /tmp/uros_log.log

RUN apt update && apt install -y netcat socat

RUN echo "UROS_FILE=$UROS_FILE" > /uros_file

COPY ./entrypoint.sh /
COPY ./wait-for /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]