#!/bin/bash

# Exporting the ROS to the gazebo Simulation PC
export ROS_MASTER_URI=http://$1:11311
export ROS_HOSTNAME=$HOSTNAME

export GAZEBO_MASTER_URI=http://$1:11345