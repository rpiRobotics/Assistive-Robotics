#!/bin/bash


gnome-terminal --tab --title="SIM" --command "bash -c \"source ~/.bashrc; rosrun  turtlesim turtlesim_node; exec bash\"";
sleep 2s;
gnome-terminal --tab --title="Teleport" --command "bash -c \"source ~/.bashrc; rosservice call /turtle1/teleport_absolute 'x: 0.0
y: 2.0
theta: 0.0'; exec bash\"";

gnome-terminal --tab --title="TELEOP" --command "bash -c \"source ~/.bashrc; rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel_base _key_timeout:=0.1; exec bash\"";
# gnome-terminal --tab --title="TELEOP" --command "bash -c \"source ~/.bashrc; rosrun rqt_robot_steering rqt_robot_steering; exec bash\"";


gnome-terminal --tab --title="TF" --command "bash -c \"source ~/.bashrc; python ~/catkin_ws_turtlesim/src/geometry_tutorials/turtle_tf2/nodes/turtle_tf2_broadcaster.py; exec bash\"";


# gnome-terminal --tab --title="TF_BLUE" --command "bash -c \"source ~/.bashrc; rosrun tf static_transform_publisher 0 2. 0        0 0 0  world_floor oarbot_blue_base 100; exec bash\"";
gnome-terminal --tab --title="TF_SILVER" --command "bash -c \"source ~/.bashrc; rosrun tf  static_transform_publisher 1. 2. 0      -1.5708  0 0   world_floor oarbot_silver_base 100; exec bash\"";



# gnome-terminal --tab --title="TF_BLUE" --command "bash -c \"source ~/.bashrc; rosrun tf static_transform_publisher -1.05 2.4 0        -0.78 0 0  world_floor oarbot_blue_base 100; exec bash\"";
# gnome-terminal --tab --title="TF_SILVER" --command "bash -c \"source ~/.bashrc; rosrun tf  static_transform_publisher -0.27 1.8 0      -1.5708  0 0   world_floor oarbot_silver_base 100; exec bash\"";

gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_assistive/src/assistive_launch/config/rviz/collision_avoidance_2d.rviz; exec bash\"";
# gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_assistive/sandbox_scripts/rviz_collision_avoidance_2d.rviz; exec bash\"";