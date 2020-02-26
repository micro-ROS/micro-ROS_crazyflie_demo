FROM ubuntu:bionic

RUN apt update \
&&  apt install -y \
        git \
        usbutils \
        socat \
&&  rm -rf /var/apt/apt/lists/*

RUN apt update \
&&  apt install -y \
        python3 \
        python3-pip \
        python3-pyqt5 \
        python3-pyqt5.qtsvg \
&&  rm -rf /var/apt/apt/lists/*

RUN git clone -b Micro-XRCE-DDS_Bridge https://github.com/eProsima/crazyflie-clients-python.git \
&&  cd crazyflie-clients-python \
&&  pip3 install -e .

RUN mkdir /.env

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]