#!/bin/bash
(
# Get the list of computer configuration scripts
COMPUTER_CONFIGS=($(ls ~/catkin_ws_assistive/device_management_scripts/computers/*.sh))
COMPUTER_CONFIGS+=("all")

# Prompt the user for selection
echo "Select a computer configuration to run rosclean:"
select config in "${COMPUTER_CONFIGS[@]}"; do
    if [[ " ${COMPUTER_CONFIGS[*]} " =~ " ${config} " ]]; then
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

# Function to run rosclean on each computer configuration
rosclean_computers() {
    local config_file=$1
    source "$config_file"

    for i in "${!HOSTS[@]}"; do
        echo "------------"
        echo "Running rosclean on ${USERNAMES[i]}@${HOSTS[i]}"

        SCRIPT="source /opt/ros/${ROS_VERSION_NAMES[i]}/setup.bash; rosclean purge -y;"

        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"

        # Run rosclean purge on the remote device
        sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$SCRIPT"
    done
}

# Execute rosclean for selected configuration(s)
if [[ $config == "all" ]]; then
    for file in ~/catkin_ws_assistive/device_management_scripts/computers/*.sh; do
        rosclean_computers "$file"
    done
else
    rosclean_computers "$config"
fi
)