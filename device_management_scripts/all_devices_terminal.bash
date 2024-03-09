#!/bin/bash
(
# Get the list of computer configuration scripts
COMPUTER_CONFIGS=($(ls ~/catkin_ws_assistive/device_management_scripts/computers/*.sh))
COMPUTER_CONFIGS+=("all")

# Prompt the user for selection
echo "Select a computer configuration to open terminals:"
select config in "${COMPUTER_CONFIGS[@]}"; do
    if [[ " ${COMPUTER_CONFIGS[*]} " =~ " ${config} " ]]; then
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

# # Function to open terminals for each computer configuration
# open_terminals() {
#     local config_file=$1
#     source "$config_file"

#     for i in "${!HOSTS[@]}"; do
#         echo "------------"
#         echo "Opening terminal for ${USERNAMES[i]}@${HOSTS[i]}"

#         ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"
        
#         local COMMAND="gnome-terminal --tab --title='${USERNAMES[i]}' -e 'sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms=ssh-rsa -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]}; exec bash'"
#     done

#     eval "$COMMAND"
# }

# Function to open terminals for each computer configuration
open_terminals() {
    local config_file=$1
    source "$config_file"

    local COMMAND="gnome-terminal"

    for i in "${!HOSTS[@]}"; do
        echo "------------"
        echo "Opening terminal for ${USERNAMES[i]}@${HOSTS[i]}"

        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"

        COMMAND+=" --tab --title='${USERNAMES[i]}' -- bash -c 'sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms=ssh-rsa -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]}; exec bash'"
    done

    # echo "Executing command: $COMMAND"
    eval "$COMMAND"
}

# Execute terminal opening for selected configuration(s)
if [[ $config == "all" ]]; then
    for file in ~/catkin_ws_assistive/device_management_scripts/computers/*.sh; do
        open_terminals "$file"
    done
else
    open_terminals "$config"
fi
)