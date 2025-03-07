#!/bin/bash
sleep 1s;
# gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roscore; exec bash\"";
# sleep 4s;
gnome-terminal --tab --title="NUC" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch nuc_kinect_with_aruco.launch; exec bash\"";

gnome-terminal --tab --title="TABLET" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch ui_tablet.launch; exec bash\"";

## Launch Oarbot arm KINECT systems 
# Launch KINECTS on the wrists of the arms
gnome-terminal --tab --title="Oarbot_BLUE_Kinect" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch oarbot_blue_kinect.launch; exec bash\"";
sleep 1s;
# Launch KINECTS on the wrists of the arms
gnome-terminal --tab --title="Oarbot_SILVER_Kinect" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch oarbot_silver_kinect.launch; exec bash\"";
sleep 1s;

# Launch OARBOT BLUE
gnome-terminal --tab --title="Oarbot_BLUE" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch oarbot_blue.launch; exec bash\"";
sleep 4s;
# Launch OARBOT SILVER
gnome-terminal --tab --title="Oarbot_SILVER" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch oarbot_silver.launch; exec bash\"";
sleep 4s;


gnome-terminal --tab --title="MASTER" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch master_computer.launch; exec bash\"";
sleep 6s;

gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; rosrun rviz rviz -d ~/catkin_ws_assistive/src/assistive_launch/config/rviz/all_oarbots_localized_with_aruco.rviz; exec bash\"";
sleep 3s;

# gnome-terminal --tab --title="ADMITTANCE" --command "bash -c \"source ~/.bashrc; source ~/catkin_ws_assistive/devel/setup.bash; roslaunch assistive_launch admittance_controller_collaborative.launch; exec bash\"";


