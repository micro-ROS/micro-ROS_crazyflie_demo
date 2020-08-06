FROM microros/micro-ros-agent:foxy

ENV SERIAL_DEV /dev/ttyS10

RUN apt update && apt install -y netcat socat

RUN echo "SERIAL_DEV=$SERIAL_DEV" > /serial_dev

COPY ./entrypoint.sh /
COPY ./wait-for /
COPY ./agent.refs /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]