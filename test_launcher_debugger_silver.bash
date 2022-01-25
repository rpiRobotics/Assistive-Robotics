#!/bin/bash
sleep 1s;
gnome-terminal --tab --title="CMD_VEL" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_silver/e_stop_cmd_vel; exec bash\"";
sleep 4s;
gnome-terminal --tab --title="ARM_JointState" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_silver/j2n6s300_left_driver/out/joint_state; exec bash\"";
gnome-terminal --tab --title="CMD_BASE" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_silver/cmd_vel_base; exec bash\"";
gnome-terminal --tab --title="CMD_ARM" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_silver/j2n6s300_left_driver/in/cartesian_velocity; exec bash\"";
gnome-terminal --tab --title="Const r" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_silver/constrained_r
; exec bash\"";
# gnome-terminal --tab --title="" --command "bash -c \"source ~/.bashrc; ; exec bash\"";



