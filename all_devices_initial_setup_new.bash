#!/bin/bash
HOSTS=("192.168.1.99" "192.168.1.100" "192.168.1.101" "192.168.1.102")
USERNAMES=("tablet" "rockie" "oarbot_silver" "oarbot_blue" )
PASSWORDS=("1234" "rockie" "1234" "1234" )

SCRIPTS=(
    "cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    source /opt/ros/noetic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='assistive_msgs;assistive_gui;assistive_launch;arduino_talker';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;
    echo 1234 | sudo -S apt install -y spacenavd;
    echo 1234 | sudo -S apt install -y ros-noetic-spacenav-node;
    echo 1234 | sudo -S apt install -y ros-noetic-twist-mux;
    echo 1234 | sudo -S usermod -a -G dialout tablet;
    pip3 install pyserial
    "








    "cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    source /opt/ros/noetic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='assistive_msgs;assistive_gui;assistive_launch';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    grep -qxF 'export ROS_IP=192.168.1.100' ~/.bashrc || echo 'export ROS_IP=192.168.1.100' >> ~/.bashrc;
    grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;
    echo rockie | sudo -S apt install -y spacenavd;
    echo rockie | sudo -S apt install -y ros-noetic-spacenav-node;
    echo rockie | sudo -S apt install -y ros-noetic-twist-mux;
    echo rockie | sudo -S usermod -a -G dialout rockie
    "







    "cd;
    echo 1234 | sudo -S usermod -a -G dialout oarbot_silver;
    echo 1234 | sudo -S apt install -y ros-melodic-twist-mux;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*;
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    pip3 install pyserial;
    pip install general-robotics-toolbox;
    if ! grep -q '^deb .*https://packages.microsoft.com/ubuntu/18.04/prod' /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo 1234 | curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -;
        echo 1234 | sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod;
        echo 1234 | sudo apt-get update;
    fi
    echo 1234 | sudo apt install -y libk4a1.4-dev;
    # sudo apt-get --purge --reinstall install libk4a1.4-dev # If something goes wrong
    echo 1234 | sudo apt install -y libk4abt1.1-dev;
    # sudo apt-get --purge --reinstall install libk4abt1.1-dev # If something goes wrong
    echo 1234 | sudo apt install -y k4a-tools;
    # sudo apt-get --purge --reinstall install k4a-tools # If something goes wrong
    echo 1234 | sudo apt install -y ros-melodic-rgbd-launch;
    cd /etc/udev/rules.d;
    echo 1234 | sudo wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/melodic-devel/kinova_driver/udev/10-kinova-arm.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.* # removes the duplicates
    echo 1234 | sudo rm 99-k4a.rules.* # removes the duplicates
    ##################
    cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    cd src;
    git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;

    cd ..;
    ##############
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;arduino_talker';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;    
    "
    









    "cd;
    echo 1234 | sudo -S usermod -a -G dialout oarbot_blue;
    echo 1234 | sudo -S apt install -y ros-melodic-twist-mux;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*;
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    pip3 install pyserial;
    pip install general-robotics-toolbox;
    if ! grep -q '^deb .*https://packages.microsoft.com/ubuntu/18.04/prod' /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo 1234 | curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -;
        echo 1234 | sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod;
        echo 1234 | sudo apt-get update;
    fi
    echo 1234 | sudo apt install -y libk4a1.4-dev;
    # sudo apt-get --purge --reinstall install libk4a1.4-dev # If something goes wrong
    echo 1234 | sudo apt install -y libk4abt1.1-dev;
    # sudo apt-get --purge --reinstall install libk4abt1.1-dev # If something goes wrong
    echo 1234 | sudo apt install -y k4a-tools;
    # sudo apt-get --purge --reinstall install k4a-tools # If something goes wrong
    echo 1234 | sudo apt install -y ros-melodic-rgbd-launch;
    cd /etc/udev/rules.d;
    echo 1234 | sudo wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/melodic-devel/kinova_driver/udev/10-kinova-arm.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.* # removes the duplicates
    echo 1234 | sudo rm 99-k4a.rules.* # removes the duplicates
    ##################
    cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    cd src;
    git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;

    cd ..;
    ##############
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;arduino_talker';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;    
    ")

echo ${SCRIPTS}
for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}
    # sudo apt-get install sshpass
    sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 
