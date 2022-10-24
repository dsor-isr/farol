#!/usr/bin/env bash

# --------------------------------------
# Install General (apt-get) requirements
# --------------------------------------
echo "Installing C++ requirements"
sudo apt-get update && \
     apt-get install -y --no-install-recommends \
     wget \
     curl \
     git \
     python3-pip \
     python3-catkin-tools \
     libgeographic-dev \
     ros-noetic-geographic-msgs \
     librosconsole-dev \
     ros-noetic-geodesy \
     ros-noetic-rosbridge-suite \
     libudev-dev \
     libusb-1.0-0-dev \
     && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------
# Requirements for farol_gazebo to work properly
# -----------------------------------------------
sudo apt-get update && \
     apt-get install -y --no-install-recommends \
     ros-noetic-web-video-server \
     gstreamer-1.0 \
     gstreamer1.0-plugins-base \
     gstreamer1.0-plugins-good \
     gstreamer1.0-plugins-bad \
     gstreamer1.0-plugins-ugly \
     libgstreamer-plugins-base1.0-dev \
     libgstrtspserver-1.0-dev \
     && rm -rf /var/lib/apt/lists/*

pip3 install toml 

# -------------------------------
# Install the Python requirements
# -------------------------------
echo "Installing Python requirements"
pip3 install numpy matplotlib rospkg catkin_pkg future joystick-controller

# -------------------------------
# Install the C++ requirements
# -------------------------------

# Install Geographiclib 1.50.1 (C++ library - for Lat/Long <-> UTM convertions):
wget -q https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.50.1.tar.gz/download && \
tar xfpz download && \
cd GeographicLib-1.50.1 && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
sudo make && \
sudo make install && \
cd .. && \
cd .. && \
sudo rm -R download GeographicLib-1.50.1

# Install Eigen version 3.4.0 (C++ equivalent of numpy in python):
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
tar xfpz eigen-3.4.0.tar.gz && \
cd eigen-3.4.0 && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
sudo make && \
sudo make install && \
cd .. && \
cd .. && \
sudo rm -R eigen-3.4.0 eigen-3.4.0.tar.gz

# -----------------------------------------------------
# Setup a ROS workspace, clone the code and compile
# -----------------------------------------------------
echo "Setting up caktin workspace"

# Run the following lines to add elements to the .bashrc file
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export CATKIN_ROOT=${HOME}" >> ~/.bashrc
echo "export ROS_WORKSPACE=${CATKIN_ROOT}/catkin_ws" >> ~/.bashrc