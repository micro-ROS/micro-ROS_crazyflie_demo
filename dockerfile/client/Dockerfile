FROM ubuntu:focal

ENV DEBIAN_FRONTEND noninteractive

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

RUN git clone -b master https://github.com/bitcraze/crazyflie-lib-python.git \
&&  cd crazyflie-lib-python \
&&  git checkout 05e1020c489f99a25c113af1cbf18d9a5a7aff5d \
&&  pip3 install -e .

RUN git clone -b master https://github.com/bitcraze/crazyflie-clients-python.git \
&&  cd crazyflie-clients-python \
&&  git checkout c123d1789e2a0783bcb550204def5e97372daa9b \
&&  pip3 install -e .

RUN pip3 install pyserial

RUN mkdir /.env

ENV DEBIAN_FRONTEND dialog

COPY ./entrypoint.sh /
COPY ./uros_cf_bridge.py /
ENTRYPOINT ["/entrypoint.sh"]