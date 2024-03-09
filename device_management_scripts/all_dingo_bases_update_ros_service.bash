#!/bin/bash
source ~/catkin_ws_assistive/device_management_scripts/computers/dingo_bases.sh

# Function to execute a command on a remote host
execute_remote() {
    local host=$1
    local username=$2
    local password=$3
    local script=$4

    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$host"
    sshpass -p "$password" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "$username" "$host" "$script"
}

# Function to construct the script for each host
construct_script() {
    local robot_name=$1
    local ros_version=$2
    local password=$3

    local script
    read -r -d '' script << EOM
echo $password | sudo -S cp ~/catkin_ws_assistive/src/dingo_customization/$robot_name/scripts/ros-start /usr/sbin/;
echo $password | sudo -S cp ~/catkin_ws_assistive/src/dingo_customization/$robot_name/scripts/setup.bash /etc/ros/;
echo $password | sudo -S cp ~/catkin_ws_assistive/src/dingo_customization/launch/base.launch /etc/ros/$ros_version/ros.d/;
echo $password | sudo -S cp ~/catkin_ws_assistive/src/dingo_customization/launch/accessories.launch /etc/ros/$ros_version/ros.d/;
EOM
    echo "$script"
}

for i in "${!HOSTS[@]}"; do
    echo "------------"
    echo "Updating ROS service on ${USERNAMES[i]}@${HOSTS[i]}"

    script=$(construct_script "${ROBOT_NAMES[i]}" "${ROS_VERSION_NAMES[i]}" "${PASSWORDS[i]}")
    # echo $script
    execute_remote "${HOSTS[i]}" "${USERNAMES[i]}" "${PASSWORDS[i]}" "$script"
done
