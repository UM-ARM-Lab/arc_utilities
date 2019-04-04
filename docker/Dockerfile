ARG ROS_VERSION=kinetic
FROM ros:${ROS_VERSION}-ros-base

RUN mkdir -p ~/catkin_ws/src

RUN apt update -yq\
    && apt -yqq install \
    libeigen3-dev \
    libz-dev \
    && rm -rf /var/lib/apt/lists/*

