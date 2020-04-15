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

RUN git clone -b master https://github.com/bitcraze/crazyflie-lib-python \
&&  cd crazyflie-clients-python \
&&  git checkout d77c1d7179046f51aba19828bc9c0dcb45570fc0
&&  pip3 install -e .

RUN mkdir /.env

COPY ./entrypoint.sh /
COPY ./micro-ROS-brigde.py /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]