FROM microros/micro-ros-agent:dashing

RUN apt update && apt install -y netcat socat

COPY ./entrypoint.sh /
COPY ./wait-for /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]