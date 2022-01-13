#!/bin/bash
HOSTS=("192.168.1.99" "192.168.1.100" "192.168.1.101" "192.168.1.102" "192.168.1.90")
USERNAMES=("tablet" "razer" "oarbot_silver" "oarbot_blue" "nuc" )
PASSWORDS=("1234" "1234" "1234" "1234" "1234")
# SCRIPTS=("cd; ls;"
#     "cd; ls;"
#     "cd; ls;"
#     "cd; ls;"
#     "cd; ls;")
COMMAND="gnome-terminal"

for i in ${!HOSTS[*]} ; do
    echo "------------"
    echo ${i}
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    # echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}
    COMMAND+=' --tab --title='${USERNAMES[i]}' -e "sshpass -p '${PASSWORDS[i]}' ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='"'ssh-rsa'"' -o ConnectTimeout=2 -l '${USERNAMES[i]}' '${HOSTS[i]}'"'
    # gnome-terminal --tab --title=${USERNAMES[i]} -e "sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]}"
done

eval $COMMAND
