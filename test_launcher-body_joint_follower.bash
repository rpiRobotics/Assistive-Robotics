#!/bin/bash
sleep 1s;
gnome-terminal --tab --title="JOINT_FOLLOWER_BLUE" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch  oarbot_blue_body_single_joint_follower.launch; exec bash\"";
# gnome-terminal --tab --title="JOINT_FOLLOWER_SILVER" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch  oarbot_silver_body_single_joint_follower.launch; exec bash\"";


