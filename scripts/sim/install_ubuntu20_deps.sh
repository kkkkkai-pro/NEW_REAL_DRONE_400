#!/usr/bin/env bash

set -euo pipefail

sudo apt update
sudo apt install -y \
  curl \
  git \
  build-essential \
  cmake \
  python3-rosdep \
  python3-catkin-tools \
  python3-osrf-pycommon

sudo apt install -y ros-noetic-desktop-full

sudo apt install -y \
  libeigen3-dev \
  libpcl-dev \
  libopencv-dev \
  libarmadillo-dev

sudo apt install -y \
  ros-noetic-cmake-modules \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf \
  ros-noetic-topic-tools

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep update
