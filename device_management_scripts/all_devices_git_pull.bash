#!/bin/bash
(
# Get the list of computer configuration scripts
COMPUTER_CONFIGS=($(ls ~/catkin_ws_assistive/device_management_scripts/computers/*.sh))
COMPUTER_CONFIGS+=("all")

# Prompt the user for selection
echo "Select a computer configuration for git pull:"
select config in "${COMPUTER_CONFIGS[@]}"; do
    if [[ " ${COMPUTER_CONFIGS[*]} " =~ " ${config} " ]]; then
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

# Function to run git pull for each computer configuration
git_pull_computers() {
    local config_file=$1
    source "$config_file"

    # Extract the base name without the .sh extension and append "_git_pull.bash"
    local base_name=$(basename "$config_file" .sh)
    local git_pull_script="${base_name}_git_pull.bash"

    for i in "${!HOSTS[@]}"; do
        echo "---------------------------------------------"
        echo "Git pulling for ${USERNAMES[i]}@${HOSTS[i]}"

        local SCRIPT="bash ~/catkin_ws_assistive/device_management_scripts/git_pull_scripts/$git_pull_script"

        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"

        # Execute the git pull script on the remote device
        sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$SCRIPT"
    done
}

# Execute git pull for selected configuration(s)
if [[ $config == "all" ]]; then
    for file in ~/catkin_ws_assistive/device_management_scripts/computers/*.sh; do
        git_pull_computers "$file"
    done
else
    git_pull_computers "$config"
fi
)