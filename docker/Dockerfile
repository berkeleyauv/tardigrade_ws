FROM ros:foxy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    git \
    iputils-ping \
    less \
    libblas-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    liblapack-dev \
    libturbojpeg \
    libusb-1.0-0-dev \
    nano \
    nlohmann-json3-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-foxy-camera-info-manager \
    ros-foxy-compressed-depth-image-transport \
    ros-foxy-compressed-image-transport \
    ros-foxy-cv-bridge \
    ros-foxy-diagnostic-updater \
    ros-foxy-geographic-msgs \
    ros-foxy-image-geometry \
    ros-foxy-image-transport \
    ros-foxy-nmea-msgs \
    ros-foxy-robot-localization \
    ros-foxy-rosbridge-suite \
    ros-foxy-stereo-msgs \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-tf2-ros \
    ros-foxy-theora-image-transport \
    ros-foxy-xacro \
    udev \
    usbutils \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pymavlink pyserial

WORKDIR /ws

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
COPY docker/ros_bashrc.sh /ros_bashrc.sh
RUN chmod +x /ros_entrypoint.sh \
    && chmod +x /ros_bashrc.sh \
    && (rosdep init || true) \
    && echo "source /ros_bashrc.sh" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "/ros_entrypoint.sh"]
CMD ["bash"]
