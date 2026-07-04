FROM ros:foxy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    bash-completion \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    iputils-ping \
    less \
    nano \
    udev \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pymavlink pyserial

WORKDIR /ws

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh \
    && (rosdep init || true) \
    && echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
