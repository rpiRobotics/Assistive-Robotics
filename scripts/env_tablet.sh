#!/usr/bin/env bash

export ROS_WS=/home/tablet/catkin_ws_assistive
export ROS_NOETIC=/opt/ros/noetic
source $ROS_NOETIC/setup.bash
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
export ROS_MASTER_URI=http://192.168.1.200:11311/
export ROS_IP=192.168.1.99
echo 1234 | sudo -S chmod 666 /dev/ttyACM*

export DISPLAY=:0 # For rviz remote launch

exec "$@"