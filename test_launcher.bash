#!/bin/bash
sleep 1s;
gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; roscore; exec bash\"";
sleep 4s;
gnome-terminal --tab --title="ARM_KINECT" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch oarbot_blue_arm_with_kinect.launch; exec bash\"";
sleep 6s;
gnome-terminal --tab --title="ARM_KINECT" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d /home/burak/catkin_ws_assistive/src/assistive_launch/config/rviz/oarbot_blue-arm-and-kinect-rviz.rviz; exec bash\"";
sleep 3s;
gnome-terminal --tab --title="ARM_KINECT" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch  oarbot_blue_body_single_joint_follower.launch; exec bash\"";

