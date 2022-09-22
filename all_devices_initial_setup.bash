#!/bin/bash
HOSTS=("192.168.1.99" "192.168.1.100" "192.168.1.101" "192.168.1.102" "192.168.1.90")
USERNAMES=("tablet" "razer" "oarbot_silver" "oarbot_blue" "nuc" )
PASSWORDS=("1234" "1234" "1234" "1234" "1234")

SCRIPTS=(
    ############### TABLET ##############
    "cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    cd src;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;
    git clone https://github.com/rpiRobotics/obstacle_detector.git;

    cd ..;
    ####################
    source /opt/ros/noetic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='assistive_msgs;assistive_gui;assistive_launch;tablet_arduino_talker;kinova_bringup;kinova_control;kinova_demo;kinova_description;kinova_docs;kinova_driver;kinova_gazebo;kinova_helper;kinova_msgs;obstacle_detector;';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;
    echo 1234 | sudo -S apt install -y spacenavd;
    echo 1234 | sudo -S apt install -y ros-noetic-spacenav-node;
    echo 1234 | sudo -S apt install -y ros-noetic-twist-mux;
    echo 1234 | sudo -S usermod -a -G dialout tablet;
    pip3 install pyserial;
    "

    ############### RAZER (MAIN COMPUTER) ##############
    "cd;
    echo 1234 | sudo -S usermod -a -G dialout razer;
    echo 1234 | sudo -S apt install -y curl;
    echo 1234 | sudo -S apt install -y software-properties-common;
    echo 1234 | sudo -S apt install -y spacenavd;
    echo 1234 | sudo -S apt install -y ros-melodic-spacenav-node;
    echo 1234 | sudo -S apt install -y ros-melodic-twist-mux;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*;
    echo 1234 | sudo -S apt install -y ros-melodic-robot-localization;
    echo 1234 | sudo -S apt install -y ros-melodic-microstrain-inertial-driver;
    echo 1234 | sudo -S apt install -y ros-melodic-microstrain-inertial-rqt;
    echo 1234 | sudo -S apt install -y ros-melodic-tf2-sensor-msgs;
    echo 1234 | sudo -S apt install -y ros-melodic-bota-driver;
    echo 1234 | sudo -S apt install -y ros-melodic-imu-tools;
    echo 1234 | sudo -S apt install -y ros-melodic-imu-pipeline; # for imu_transformer
    echo 1234 | sudo -S apt install -y python-cvxopt; # TODO
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    pip3 install pyserial;
    pip install general-robotics-toolbox;
    pip install quadprog; # TODO
    echo 1234 | sudo -S apt install -y python-dev; # TODO
    pip install qpsolvers==1.4.1; # TODO
    pip install Shapely;
    if ! grep -q '^deb .*https://packages.microsoft.com/ubuntu/18.04/prod' /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo 1234 | curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -;
        echo 1234 | sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod;
        echo 1234 | sudo apt-get update;
    fi;
    export DEBIAN_FRONTEND=noninteractive;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections;
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
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-microstrain-imu.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-hokuyo-ust-20ln.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-bota-rokubi-ft.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-k4a.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-microstrain-imu.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-hokuyo-ust-20ln.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-bota-rokubi-ft.rules.*; # removes the duplicates
    ##################
    cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    cd src;
    git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;
    git clone https://github.com/rpiRobotics/obstacle_detector.git;
    cd bota_rokubi_ft_sensor;
    git clone -b melodic https://github.com/rpiRobotics/force_torque_tools.git;
    cd ../..;
    ##############
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='tablet_arduino_talker';
    # catkin_make -DCATKIN_WHITELIST_PACKAGES='assistive_msgs;assistive_gui;assistive_launch';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    grep -qxF 'export ROS_IP=192.168.1.100' ~/.bashrc || echo 'export ROS_IP=192.168.1.100' >> ~/.bashrc;
    grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;    
    grep -qxF 'razer    soft   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'razer    soft   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot
    grep -qxF 'razer    hard   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'razer    hard   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot
    "

    ############### OARBOT_SILVER ##############
    "cd;
    echo 1234 | sudo -S usermod -a -G dialout oarbot_silver;
    echo 1234 | sudo -S apt install -y curl;
    echo 1234 | sudo -S apt install -y software-properties-common;
    echo 1234 | sudo -S apt install -y ros-melodic-twist-mux;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*;
    echo 1234 | sudo -S apt install -y ros-melodic-robot-localization;
    echo 1234 | sudo -S apt install -y ros-melodic-microstrain-inertial-driver;
    echo 1234 | sudo -S apt install -y ros-melodic-microstrain-inertial-rqt;
    echo 1234 | sudo -S apt install -y ros-melodic-tf2-sensor-msgs;
    echo 1234 | sudo -S apt install -y ros-melodic-bota-driver;
    echo 1234 | sudo -S apt install -y ros-melodic-imu-tools;
    echo 1234 | sudo -S apt install -y ros-melodic-imu-pipeline;
    echo 1234 | sudo -S apt install -y python-cvxopt; # TODO
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    pip3 install pyserial;
    pip install general-robotics-toolbox;
    pip install quadprog; # TODO
    echo 1234 | sudo -S apt install -y python-dev; # TODO
    pip install qpsolvers==1.4.1; # TODO
    pip install Shapely;
    if ! grep -q '^deb .*https://packages.microsoft.com/ubuntu/18.04/prod' /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo 1234 | curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -;
        echo 1234 | sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod;
        echo 1234 | sudo apt-get update;
    fi;
    export DEBIAN_FRONTEND=noninteractive;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections;
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
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-microstrain-imu.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-hokuyo-ust-20ln.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-bota-rokubi-ft.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-k4a.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-microstrain-imu.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-hokuyo-ust-20ln.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-bota-rokubi-ft.rules.*; # removes the duplicates
    ##################
    cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    cd src;
    git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;
    git clone https://github.com/rpiRobotics/obstacle_detector.git;
    cd bota_rokubi_ft_sensor;
    git clone -b melodic https://github.com/rpiRobotics/force_torque_tools.git;
    cd ../..;
    ##############
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;tablet_arduino_talker';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;
    grep -qxF 'oarbot_silver    soft   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'oarbot_silver    soft   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot
    grep -qxF 'oarbot_silver    hard   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'oarbot_silver    hard   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot
    "

    ############### OARBOT_BLUE ##############
    "cd;
    echo 1234 | sudo -S usermod -a -G dialout oarbot_blue;
    echo 1234 | sudo -S apt install -y curl;
    echo 1234 | sudo -S apt install -y software-properties-common;
    echo 1234 | sudo -S apt install -y ros-melodic-twist-mux;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*;
    echo 1234 | sudo -S apt install -y ros-melodic-robot-localization;
    echo 1234 | sudo -S apt install -y ros-melodic-microstrain-inertial-driver;
    echo 1234 | sudo -S apt install -y ros-melodic-microstrain-inertial-rqt;
    echo 1234 | sudo -S apt install -y ros-melodic-tf2-sensor-msgs;
    echo 1234 | sudo -S apt install -y ros-melodic-bota-driver;
    echo 1234 | sudo -S apt install -y ros-melodic-imu-tools;
    echo 1234 | sudo -S apt install -y ros-melodic-imu-pipeline;
    echo 1234 | sudo -S apt install -y python-cvxopt; # TODO
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    pip3 install pyserial;
    pip install general-robotics-toolbox;
    pip install quadprog; # TODO
    echo 1234 | sudo -S apt install -y python-dev; # TODO
    pip install qpsolvers==1.4.1; # TODO
    pip install Shapely;
    if ! grep -q '^deb .*https://packages.microsoft.com/ubuntu/18.04/prod' /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo 1234 | curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -;
        echo 1234 | sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod;
        echo 1234 | sudo apt-get update;
    fi;
    export DEBIAN_FRONTEND=noninteractive;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections;
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
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-microstrain-imu.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-hokuyo-ust-20ln.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-bota-rokubi-ft.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-k4a.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-microstrain-imu.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-hokuyo-ust-20ln.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-bota-rokubi-ft.rules.*; # removes the duplicates
    ##################
    cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    cd src;
    git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;
    git clone https://github.com/rpiRobotics/obstacle_detector.git;
    cd bota_rokubi_ft_sensor;
    git clone -b melodic https://github.com/rpiRobotics/force_torque_tools.git;
    cd ../..;
    ##############
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;tablet_arduino_talker';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;
    grep -qxF 'oarbot_blue    soft   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'oarbot_blue    soft   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot
    grep -qxF 'oarbot_blue    hard   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'oarbot_blue    hard   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot 
    "
    
    ############### NUC ##############
    "cd;
    echo 1234 | sudo -S usermod -a -G dialout nuc;
    echo 1234 | sudo -S apt install -y curl;
    echo 1234 | sudo -S apt install -y software-properties-common;
    echo 1234 | sudo -S apt install -y ros-melodic-twist-mux;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*;
    echo 1234 | sudo -S apt install -y python-cvxopt; # TODO
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    pip3 install pyserial;
    pip install general-robotics-toolbox;
    pip install quadprog; # TODO
    echo 1234 | sudo -S apt install -y python-dev; # TODO
    pip install qpsolvers==1.4.1; # TODO
    pip install pandas;
    pip install Shapely;
    if ! grep -q '^deb .*https://packages.microsoft.com/ubuntu/18.04/prod' /etc/apt/sources.list /etc/apt/sources.list.d/*; then
        echo 1234 | curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -;
        echo 1234 | sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod;
        echo 1234 | sudo apt-get update;
    fi;
    export DEBIAN_FRONTEND=noninteractive;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accept-eula boolean true' | sudo debconf-set-selections;
    echo 1234 | echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections;
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
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-microstrain-imu.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-hokuyo-ust-20ln.rules;
    echo 1234 | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-bota-rokubi-ft.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-k4a.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-microstrain-imu.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-hokuyo-ust-20ln.rules.*; # removes the duplicates
    echo 1234 | sudo rm 99-bota-rokubi-ft.rules.*; # removes the duplicates
    ##################
    cd; 
    mkdir catkin_ws_assistive; 
    cd catkin_ws_assistive; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
    cd src;
    git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;
    git clone https://github.com/rpiRobotics/obstacle_detector.git;
    cd bota_rokubi_ft_sensor;
    git clone -b melodic https://github.com/rpiRobotics/force_torque_tools.git;
    cd ../..;
    ##############
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;tablet_arduino_talker';
    grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_assistive/devel/setup.bash;    
    grep -qxF 'nuc    soft   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'nuc    soft   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot
    grep -qxF 'nuc    hard   rtprio    99' /etc/security/limits.conf || echo 1234 | echo 'nuc    hard   rtprio    99' | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot 
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
