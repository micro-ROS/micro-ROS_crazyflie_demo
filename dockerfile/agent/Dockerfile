FROM microros/micro-ros-agent:dashing

RUN apt install -y netcat socat

COPY ./entrypoint.sh /
COPY ./wait-for /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]