#!/bin/bash
(
# Get the list of computer configuration scripts
COMPUTER_CONFIGS=($(ls ~/catkin_ws_assistive/device_management_scripts/computers/*.sh))
COMPUTER_CONFIGS+=("all")

# Define device vendor and product IDs
# based on https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules
declare -A DEVICES=(
    ["097a"]="Generic Superspeed USB Hub"
    ["097b"]="Generic USB Hub"
    ["097c"]="Azure Kinect Depth Camera"
    ["097d"]="Azure Kinect 4K Camera"
    ["097e"]="Azure Kinect Microphone Array"
)

# Prompt the user for selection
echo "Select a computer configuration to Kinect Azure usb_reset:"
select config in "${COMPUTER_CONFIGS[@]}"; do
    if [[ " ${COMPUTER_CONFIGS[*]} " =~ " ${config} " ]]; then
        break
    else
        echo "Invalid selection. Please try again."
    fi
done


# Function to usb_reset each computer configuration
usb_reset_computers() {
    local config_file=$1
    source "$config_file"

    for i in "${!HOSTS[@]}"; do
        echo "------------"
        echo "Kinect Azure usb_resetting ${USERNAMES[i]}@${HOSTS[i]}"
        
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"

        for id in "${!DEVICES[@]}"; do
            DEVICE_CMD="echo ${PASSWORDS[i]} | sudo -S usb_resetter -d 045e:${id} --reset-hub;"

            # usb_reset the remote device
            sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$DEVICE_CMD"
        done
    done
}

# Execute usb_reset for selected configuration(s)
if [[ $config == "all" ]]; then
    for file in ~/catkin_ws_assistive/device_management_scripts/computers/*.sh; do
        usb_reset_computers "$file"
    done
else
    usb_reset_computers "$config"
fi
)