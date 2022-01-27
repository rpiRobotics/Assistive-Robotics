#!/bin/bash
sleep 1s;
gnome-terminal --tab --title="CMD_VEL" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_blue/cmd_vel; exec bash\"";
gnome-terminal --tab --title="ARM_JointState" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_blue/j2n6s300_right_driver/out/joint_state; exec bash\"";
gnome-terminal --tab --title="CMD_BASE" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_blue/cmd_vel_base; exec bash\"";
gnome-terminal --tab --title="CMD_ARM" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_blue/j2n6s300_right_driver/in/cartesian_velocity; exec bash\"";
gnome-terminal --tab --title="Const r" --command "bash -c \"source ~/.bashrc; rostopic echo /oarbot_blue/constrained_r
; exec bash\"";
# gnome-terminal --tab --title="" --command "bash -c \"source ~/.bashrc; ; exec bash\"";



