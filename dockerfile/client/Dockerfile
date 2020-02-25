FROM ubuntu:bionic

RUN apt update \
&&  apt install -y \
        git \
        usbutils \
&&  rm -rf /var/apt/apt/lists/*

RUN apt update \
&&  apt install -y \
        python3 \
        python3-pip \
        python3-pyqt5 \
        python3-pyqt5.qtsvg \
&&  rm -rf /var/apt/apt/lists/*

RUN git clone -b feature/uxrce_bridge https://github.com/eProsima/crazyflie-clients-python.git \
&&  cd crazyflie-clients-python \
&&  pip3 install -e .