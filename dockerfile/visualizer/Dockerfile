FROM ros:galactic

# Avoid warnings and ask users for manual configuration parameters
ENV DEBIAN_FRONTEND noninteractive

ARG USERNAME=cf_visualizer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create cf_visualizer user and give it sudo permissions
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    apt-get install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Update nodejs version
RUN apt-get update \
    && apt install curl \
    && curl -sL https://deb.nodesource.com/setup_12.x -o nodesource_setup.sh \
  
# Configure apt and install packages
RUN apt-get update \
    &&  apt install -y ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    &&  apt remove -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    && apt-get -y install --no-install-recommends git nodejs libnss3-dev \
    libatk-adaptor libgtk2.0-0 libgtk-3-0 libgbm-dev libxss1 libasound2 wget \
    libx11* libxcb-dri*

# Clean up
RUN apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p /visualizer_ws
WORKDIR /visualizer_ws

# Copy visualizer applications
COPY ros2subscriber ./ros2subscriber
COPY gui ./gui

# Install dependencies using npm
RUN . /opt/ros/$ROS_DISTRO/local_setup.sh && \
    npm config set registry http://registry.npmjs.org && \
    cd ros2subscriber && npm install && \
    cd ../gui && npm install

# Change to cf_visualizer username
USER $USERNAME

ENV DEBIAN_FRONTEND dialog
COPY disable_fastdds_shm.xml /tmp/
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
