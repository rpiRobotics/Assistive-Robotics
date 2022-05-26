#!/bin/bash
sleep 1s;
gnome-terminal --tab --title="Correct Torques" --command  "bash -c \"source ~/.bashrc; rosservice call /oarbot_silver/j2n6s300_left_driver/in/set_torque_control_parameters; rosservice call /oarbot_blue/j2n6s300_right_driver/in/set_torque_control_parameters; exec bash\"";
