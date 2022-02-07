#!/bin/bash
sleep 1s;
# gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; roscore; exec bash\"";
# sleep 4s;
gnome-terminal --tab --title="WRENCH LIN" --command  "bash -c \"source ~/.bashrc; rqt_plot /oarbot_blue/j2n6s300_right_driver/out/tool_wrench_filtered/wrench/force/x:y:z /oarbot_silver/j2n6s300_left_driver/out/tool_wrench_filtered/wrench/force/x:y:z; exec bash\"";
gnome-terminal --tab --title="WRENCH ANG" --command  "bash -c \"source ~/.bashrc; rqt_plot /oarbot_blue/j2n6s300_right_driver/out/tool_wrench_filtered/wrench/torque/x:y:z /oarbot_silver/j2n6s300_left_driver/out/tool_wrench_filtered/wrench/torque/x:y:z; exec bash\"";
gnome-terminal --tab --title="CMD_VEL LIN" --command "bash -c \"source ~/.bashrc; rqt_plot /oarbot_blue/cmd_vel/linear/x:y:z /oarbot_silver/cmd_vel/linear/x:y:z; exec bash\"";
gnome-terminal --tab --title="CMD_VEL ANG" --command "bash -c \"source ~/.bashrc; rqt_plot /oarbot_blue/cmd_vel/angular/x:y:z /oarbot_silver/cmd_vel/angular/x:y:z; exec bash\"";
