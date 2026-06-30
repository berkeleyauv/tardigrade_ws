FROM ros:foxy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/* \

WORKDIR /ws

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrcf