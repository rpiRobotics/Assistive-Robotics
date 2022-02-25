#!/bin/bash
sleep 1s;
# gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; roscore; exec bash\"";
# sleep 4s;
gnome-terminal --tab --title="TABLET" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch ui_tablet.launch; exec bash\"";
gnome-terminal --tab --title="Oarbot_BLUE" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch oarbot_blue.launch; exec bash\"";
gnome-terminal --tab --title="Oarbot_SILVER" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch oarbot_silver.launch; exec bash\"";
gnome-terminal --tab --title="NUC" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch nuc_kinect_with_aruco.launch; exec bash\"";
sleep 6s;

gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_assistive/src/assistive_launch/config/rviz/all_oarbots_localized_with_aruco.rviz; exec bash\"";
sleep 3s;
# gnome-terminal --tab --title="JOINT_FOLLOWER_BLUE" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch  oarbot_blue_body_single_joint_follower.launch; exec bash\"";
# gnome-terminal --tab --title="JOINT_FOLLOWER_SILVER" --command "bash -c \"source ~/.bashrc; roslaunch assistive_launch  oarbot_silver_body_single_joint_follower.launch; exec bash\"";


