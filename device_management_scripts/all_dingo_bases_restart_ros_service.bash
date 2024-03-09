#!/bin/bash
source ~/catkin_ws_assistive/device_management_scripts/computers/dingo_bases.sh

for i in "${!HOSTS[@]}"; do
    echo "------------"
    echo "Restarting ROS service on ${USERNAMES[i]}@${HOSTS[i]}"

    SCRIPT="echo ${PASSWORDS[i]} | sudo -S systemctl restart ros.service;"

    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"
    
    sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$SCRIPT"
done
