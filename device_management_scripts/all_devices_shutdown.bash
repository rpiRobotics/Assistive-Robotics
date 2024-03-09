#!/bin/bash
(
# Get the list of computer configuration scripts
COMPUTER_CONFIGS=($(ls ~/catkin_ws_assistive/device_management_scripts/computers/*.sh))
COMPUTER_CONFIGS+=("all")

# Prompt the user for selection
echo "Select a computer configuration to shutdown:"
select config in "${COMPUTER_CONFIGS[@]}"; do
    if [[ " ${COMPUTER_CONFIGS[*]} " =~ " ${config} " ]]; then
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

# Function to shutdown each computer configuration
shutdown_computers() {
    local config_file=$1
    source "$config_file"

    for i in "${!HOSTS[@]}"; do
        echo "------------"
        echo "shutdowning ${USERNAMES[i]}@${HOSTS[i]}"

        SCRIPT="echo ${PASSWORDS[i]} | sudo -S shutdown -h now;"

        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"

        # shutdown the remote device
        sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$SCRIPT"
    done
}

# Execute shutdown for selected configuration(s)
if [[ $config == "all" ]]; then
    for file in ~/catkin_ws_assistive/device_management_scripts/computers/*.sh; do
        shutdown_computers "$file"
    done
else
    shutdown_computers "$config"
fi
)