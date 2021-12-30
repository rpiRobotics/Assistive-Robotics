#!/bin/bash
sleep 1s;
gnome-terminal --tab --title="START_FORCE_CONTROL_BLUE" --command "bash -c \"
source ~/.bashrc; 

rosservice call /oarbot_blue/j2n6s300_right_driver/in/start_force_control;

# DEFAULT PARAMETERS ARE BELOW
# rosservice call /oarbot_blue/j2n6s300_right_driver/in/set_force_control_params '
# inertia_linear: {x: 6.36364, y: 11.1364, z: 19.0909}
# inertia_angular: {x: 0.212404, y: 0.212404, z: 0.212404}
# damping_linear: {x: 18.1818, y: 31.8182, z: 54.5455}
# damping_angular: {x: 0.849618, y: 0.849618, z: 0.849618}
# force_min_linear: {x: 8.0, y: 8.0, z: 8.0}
# force_min_angular: {x: 1.5, y: 1.5, z: 1.5}
# force_max_linear: {x: 12.0, y: 15.0, z: 20.0}
# force_max_angular: {x: 2.5, y: 2.5, z: 2.5}';

rosservice call /oarbot_blue/j2n6s300_right_driver/in/set_force_control_params '
inertia_linear: {x: 1.0, y: 1.0, z: 1.0}
inertia_angular: {x: 0.05, y: 0.05, z: 0.05}
damping_linear: {x: 31.8182, y: 31.8182, z: 31.8182}
damping_angular: {x: 0.449618, y: 0.449618, z: 0.449618}
force_min_linear: {x: 3.0, y: 3.0, z: 3.0}
force_min_angular: {x: 0.8, y: 0.8, z: 0.8}
force_max_linear: {x: 12.0, y: 15.0, z: 20.0}
force_max_angular: {x: 2.5, y: 2.5, z: 2.5}'

exec bash\"";

gnome-terminal --tab --title="START_FORCE_CONTROL_SILVER" --command "bash -c \"
source ~/.bashrc; 

rosservice call /oarbot_silver/j2n6s300_left_driver/in/start_force_control;

# DEFAULT PARAMETERS ARE BELOW
# rosservice call /oarbot_silver/j2n6s300_left_driver/in/set_force_control_params '
# inertia_linear: {x: 6.36364, y: 11.1364, z: 19.0909}
# inertia_angular: {x: 0.212404, y: 0.212404, z: 0.212404}
# damping_linear: {x: 18.1818, y: 31.8182, z: 54.5455}
# damping_angular: {x: 0.849618, y: 0.849618, z: 0.849618}
# force_min_linear: {x: 8.0, y: 8.0, z: 8.0}
# force_min_angular: {x: 1.5, y: 1.5, z: 1.5}
# force_max_linear: {x: 12.0, y: 15.0, z: 20.0}
# force_max_angular: {x: 2.5, y: 2.5, z: 2.5}';

rosservice call /oarbot_silver/j2n6s300_left_driver/in/set_force_control_params '
inertia_linear: {x: 1.0, y: 1.0, z: 1.0}
inertia_angular: {x: 0.05, y: 0.05, z: 0.05}
damping_linear: {x: 31.8182, y: 31.8182, z: 31.8182}
damping_angular: {x: 0.449618, y: 0.449618, z: 0.449618}
force_min_linear: {x: 3.0, y: 3.0, z: 3.0}
force_min_angular: {x: 0.8, y: 0.8, z: 0.8}
force_max_linear: {x: 12.0, y: 15.0, z: 20.0}
force_max_angular: {x: 2.5, y: 2.5, z: 2.5}'

exec bash\"";


