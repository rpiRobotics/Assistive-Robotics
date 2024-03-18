#!/usr/bin/env bash

# Check if MY_PASSWORD is set
if [ -z "$MY_PASSWORD" ]; then
    echo "Password not set. Exiting..."
    exit 1
fi

# Check if MY_USERNAME is set
if [ -z "$MY_USERNAME" ]; then
    echo "Username not set. Exiting..."
    exit 1
fi

# Check if MY_USERNAME is set
if [ -z "$MY_IP" ]; then
    echo "Username not set. Exiting..."
    exit 1
fi

# Check if MY_ROS_VERSION_NAME is set
if [ -z "$MY_ROS_VERSION_NAME" ]; then
    echo "ROS version name is not set. Exiting..."
    exit 1
fi

cd;

echo $MY_PASSWORD | sudo -S usermod -a -G dialout $MY_USERNAME;
echo $MY_PASSWORD | sudo -S apt-get update;
echo $MY_PASSWORD | sudo -S apt-get install -y curl;
echo $MY_PASSWORD | sudo -S apt-get install -y sshpass;

# CUSTOM RELATED
echo $MY_PASSWORD | sudo -S apt-get install -y software-properties-common;
# echo $MY_PASSWORD | sudo -S apt-get install -y spacenavd;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-spacenav-node;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-twist-mux;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-moveit*;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-robot-localization;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-microstrain-inertial-driver;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-microstrain-inertial-rqt;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-urg-node; # for hokuyo ust 10 lidar sensors
echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-tf2-sensor-msgs;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-bota-driver;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-imu-tools;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-imu-pipeline; # for imu_transformer
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-rqt-ez-publisher;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-navigation; # for navigation stack
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-visualization-tutorials; # rviz python bindings.
echo $MY_PASSWORD | sudo -S apt-get install -y python-cvxopt; # TODO
# echo $MY_PASSWORD | sudo -S apt-get install -y python3-cvxopt; # TODO
# echo $MY_PASSWORD | sudo -S apt-get install -y python3-pip;
echo $MY_PASSWORD | sudo -S apt-get install -y python-pip;
# echo $MY_PASSWORD | sudo -S apt-get install -y python3-dev; # TODO
echo $MY_PASSWORD | sudo -S apt-get install -y python-dev; # TODO

# PYTHON RELATED
pip install pyserial;
pip install quadprog;
pip install qpsolvers==1.4.1; # TODO
pip install pandas;
pip install pygame;
pip install scipy;
pip install numpy==1.21; # needed to resolve the issue "AttributeError: module 'numpy' has no attribute 'typeDict'"
pip install shapely; # needed to calculate the swarm footprint polygon and costmap parameter updater functions
pip install matplotlib==3.7.3;
# pip install ortools==9.7.2996;
pip install general-robotics-toolbox;
echo $MY_PASSWORD | sudo -S pip install usb_resetter; # useful to prevent physically unplugging and plugging back USB cables on the robots

# KINECT, KINOVA ARM, AND SENSORS RELATED
if ! grep -q '^deb .*https://packages.microsoft.com/ubuntu/18.04/prod' /etc/apt/sources.list /etc/apt/sources.list.d/*; then
    echo $MY_PASSWORD | curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -;
    echo $MY_PASSWORD | sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod;
    echo $MY_PASSWORD | sudo apt-get update;
fi;
export DEBIAN_FRONTEND=noninteractive;
echo $MY_PASSWORD | echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | sudo debconf-set-selections;
echo $MY_PASSWORD | echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections;
echo $MY_PASSWORD | echo 'libk4abt1.1 libk4abt1.1/accept-eula boolean true' | sudo debconf-set-selections;
echo $MY_PASSWORD | echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections;
echo $MY_PASSWORD | sudo apt-get install -y libk4a1.4-dev;
# sudo apt-get --purge --reinstall install libk4a1.4-dev # If something goes wrong
echo $MY_PASSWORD | sudo apt-get install -y libk4abt1.1-dev;
# sudo apt-get --purge --reinstall install libk4abt1.1-dev # If something goes wrong
echo $MY_PASSWORD | sudo apt-get install -y k4a-tools;
# sudo apt-get --purge --reinstall install k4a-tools # If something goes wrong
echo $MY_PASSWORD | sudo apt-get install -y ros-$MY_ROS_VERSION_NAME-rgbd-launch;
cd /etc/udev/rules.d;
echo $MY_PASSWORD | sudo wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules;
# echo $MY_PASSWORD | sudo wget https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/$MY_ROS_VERSION_NAME-devel/kinova_driver/udev/10-kinova-arm.rules;
# echo $MY_PASSWORD | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-microstrain-imu.rules;
# echo $MY_PASSWORD | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-hokuyo-ust-20ln.rules;
# echo $MY_PASSWORD | sudo wget https://raw.githubusercontent.com/rpiRobotics/Assistive-Robotics/main/udev/99-bota-rokubi-ft.rules;
# echo $MY_PASSWORD | sudo rm 10-kinova-arm.rules.*; # removes the duplicates
echo $MY_PASSWORD | sudo rm 99-k4a.rules.*; # removes the duplicates
# echo $MY_PASSWORD | sudo rm 99-microstrain-imu.rules.*; # removes the duplicates
# echo $MY_PASSWORD | sudo rm 99-hokuyo-ust-20ln.rules.*; # removes the duplicates
# echo $MY_PASSWORD | sudo rm 99-bota-rokubi-ft.rules.*; # removes the duplicates


# DINGO RELATED
cd;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-dingo-desktop;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-dingo-simulator;
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-$MY_ROS_VERSION_NAME-dingo-navigation;

# ## ONLY ON PHYSICAL ROBOTS, NEED TO INSTALL
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-roslint; # needed to build dingo_base package
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-gazebo-msgs;
# # Adding CLEARPATH authentication keys for the packages.clearpathrobotics.com repository
# wget https://packages.clearpathrobotics.com/public.key -O - | echo $MY_PASSWORD | sudo -S apt-key add -
# SOURCE_LINE="deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main"
# if ! grep -qxF "$SOURCE_LINE" /etc/apt/sources.list.d/clearpath-latest.list; then
#     echo "$SOURCE_LINE" | echo $MY_PASSWORD | sudo -S tee /etc/apt/sources.list.d/clearpath-latest.list
# fi
# echo $MY_PASSWORD | sudo -S apt-get update
# echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-dingo-robot; # AFTER ADDING CLEARPATH KEYS (see https://docs.clearpathrobotics.com/docs/ros1noetic/robots/indoor_robots/dingo/tutorials_dingo/#installing-from-debian-packages)


## Building Steps
cd;
mkdir catkin_ws_assistive;
cd catkin_ws_assistive;
rm -rf {*,.*};

git clone https://github.com/rpiRobotics/Assistive-Robotics.git .;
cd src;
# git clone https://github.com/burakaksoy/AssistiveRobot-SimulationFiles.git; # only on DESKTOP
# git clone https://github.com/burakaksoy/RVizMeshVisualizer.git; # only on DESKTOP
# git clone https://github.com/burakaksoy/uwb_gazebo_plugin; # only on DESKTOP
git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git;
# git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;
# git clone https://github.com/rpiRobotics/obstacle_detector.git;
# git clone --branch throttle-tf-repeated-data-error https://github.com/burakaksoy/geometry2.git; # to fix tf repeating data warning flooding
# git clone https://github.com/burakaksoy/rviz_ortho_view_controller.git;
# git clone https://github.com/facontidavide/rosbag_editor.git;
# git clone https://github.com/burakaksoy/topic_tf_transformers.git; # to read odom from tf

cd bota_rokubi_ft_sensor;
# git clone -b $MY_ROS_VERSION_NAME https://github.com/rpiRobotics/force_torque_tools.git;
cd ../..;

source /opt/ros/$MY_ROS_VERSION_NAME/setup.bash;
catkin_make -DCATKIN_WHITELIST_PACKAGES="$WHITELIST_PACKAGES" -DCMAKE_BUILD_TYPE=Release;

source devel/setup.bash;

# UPDATE ~/.BASHRC FILE
grep -qxF 'source ~/catkin_ws_assistive/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_assistive/devel/setup.bash' >> ~/.bashrc;
grep -qxF "export ROS_IP=$MY_IP" ~/.bashrc || echo "export ROS_IP=$MY_IP" >> ~/.bashrc;
grep -qxF 'export ROS_MASTER_URI=http://192.168.1.100:11311/' ~/.bashrc || echo 'export ROS_MASTER_URI=http://192.168.1.100:11311/' >> ~/.bashrc;
# grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;

source ~/.bashrc;
source ~/catkin_ws_assistive/devel/setup.bash;    

# Adding or updating limits.conf for the user
# grep -qxF "$MY_USERNAME    soft   rtprio    99" /etc/security/limits.conf || echo "$MY_PASSWORD" | echo "$MY_USERNAME    soft   rtprio    99" | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot
# grep -qxF "$MY_USERNAME    hard   rtprio    99" /etc/security/limits.conf || echo "$MY_PASSWORD" | echo "$MY_USERNAME    hard   rtprio    99" | sudo -S tee -a /etc/security/limits.conf; # for FT sensor, effective after reboot