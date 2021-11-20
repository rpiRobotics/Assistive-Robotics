#!/usr/bin/env bash

export ROS_WS=/home/oarbot_silver/catkin_ws_assistive
export ROS_MELODIC=/opt/ros/melodic
source $ROS_MELODIC/setup.bash
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
export ROS_MASTER_URI=http://192.168.1.200:11311/
export ROS_IP=192.168.1.101
export DISPLAY:=1

exec "$@"