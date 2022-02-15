#!/bin/bash
# gnome-terminal --tab --title="TF_BLUE" --command "bash -c \"source ~/.bashrc; rosrun tf static_transform_publisher 0 2. 0        0 0 0  world_floor oarbot_blue_base 100; exec bash\"";
# gnome-terminal --tab --title="TF_SILVER" --command "bash -c \"source ~/.bashrc; rosrun tf  static_transform_publisher 1. 2. 0      -1.5708  0 0   world_floor oarbot_silver_base 100; exec bash\"";
gnome-terminal --tab --title="TF_BLUE" --command "bash -c \"source ~/.bashrc; rosrun tf static_transform_publisher -1.05 2.4 0        -0.78 0 0  world_floor oarbot_blue_base 100; exec bash\"";
gnome-terminal --tab --title="TF_SILVER" --command "bash -c \"source ~/.bashrc; rosrun tf  static_transform_publisher -0.27 1.8 0      -1.5708  0 0   world_floor oarbot_silver_base 100; exec bash\"";

gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_assistive/src/assistive_launch/config/rviz/collision_avoidance_2d.rviz; exec bash\"";
# gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_assistive/sandbox_scripts/rviz_collision_avoidance_2d.rviz; exec bash\"";