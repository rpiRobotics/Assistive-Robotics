#!/bin/bash
(
# Get the list of computer configuration scripts
COMPUTER_CONFIGS=($(ls ~/catkin_ws_assistive/device_management_scripts/computers/*.sh))
COMPUTER_CONFIGS+=("all")

# Prompt the user for selection
echo "Select a computer configuration to set up:"
select config in "${COMPUTER_CONFIGS[@]}"; do
    if [[ " ${COMPUTER_CONFIGS[*]} " =~ " ${config} " ]]; then
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

# Function to run setup for each computer configuration
setup_computers() {
    local config_file=$1
    source "$config_file"

    # Extract the base name without the .sh extension and append "_install.bash"
    local base_name=$(basename "$config_file" .sh)
    local install_script="${base_name}_install.bash"

    for i in "${!HOSTS[@]}"; do
        echo "---------------------------------------------"
        echo "Initial Setup for ${USERNAMES[i]}@${HOSTS[i]}"

        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"

        # Copy the install script to the remote device
        sshpass -p "${PASSWORDS[i]}" scp -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 ~/catkin_ws_assistive/device_management_scripts/initial_setup_scripts/$install_script "${USERNAMES[i]}@${HOSTS[i]}":~/

        # Construct CATKIN_WHITELIST_PACKAGES string
        WHITELIST_PACKAGES=$(printf ";%s" "${CATKIN_WHITELIST_PACKAGES[@]}")
        WHITELIST_PACKAGES=${WHITELIST_PACKAGES:1} # remove the leading semicolon

        # Execute the install script on the remote device
        sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" \
        "export MY_IP='${HOSTS[i]}'; export MY_USERNAME='${USERNAMES[i]}'; export MY_PASSWORD='${PASSWORDS[i]}'; export MY_ROS_VERSION_NAME='${ROS_VERSION_NAMES[i]}'; export WHITELIST_PACKAGES='${WHITELIST_PACKAGES}'; bash ~/$install_script"
    done
}

# Execute setup for selected configuration(s)
if [[ $config == "all" ]]; then
    for file in ~/catkin_ws_assistive/device_management_scripts/computers/*.sh; do
        setup_computers "$file"
    done
else
    setup_computers "$config"
fi
)