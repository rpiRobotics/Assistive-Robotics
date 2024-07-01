# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=br0

# Load the robot's model type and serial number
source /etc/clearpath-serial.bash

# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
######
export DINGO_WIRELESS_INTERFACE=wlp2s0
export DINGO_OMNI=1

# Pass through to the main ROS workspace of the system.
source /opt/ros/noetic/setup.bash

# If you have a catkin workspace, source it below. e.g.
source /home/administrator/catkin_ws_assistive/devel/setup.bash

# Any additional environment variables that depend on your workspace should be exported here
# source /etc/clearpath-dingo.bash

# Add the config extras file:
# export DINGO_CONFIG_EXTRAS=$(catkin_find rpi06_dingo config/localization.yaml --first-only)

export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.101

# Add dingo custom name
export DINGO_CUSTOM_NAME=d1
export TF_PREFIX="${DINGO_CUSTOM_NAME}_tf_"

# Add the front facing Hokuyo UST10LX LiDAR
export DINGO_LASER=true
export DINGO_LASER_MODEL=ust10
export DINGO_LASER_OFFSET="0.04 0 0.003175" # wrt front_mount in the description
export DINGO_LASER_MOUNT="${TF_PREFIX}front"

# Add the rear facing Hokuyo UST10LX LiDAR
export DINGO_LASER_SECONDARY=true
export DINGO_LASER_SECONDARY_MODEL=ust10
export DINGO_LASER_SECONDARY_OFFSET="-0.04 0 0" # wrt rear_mount in the description
export DINGO_LASER_SECONDARY_MOUNT="${TF_PREFIX}rear"

# Add the Microstrain 3DMGX5 IMU
export DINGO_IMU_MICROSTRAIN=true
export DINGO_IMU_MICROSTRAIN_OFFSET="0 0 0.083175" # wrt base in the description
export DINGO_IMU_MICROSTRAIN_LINK="${TF_PREFIX}microstrain_link"
export DINGO_IMU_MICROSTRAIN_PARENT="${TF_PREFIX}imu_link"

# Add UWB tag IDs on the robot (1: RIGHT, 2:LEFT)
export DINGO_UWB_TAGS=false # true
# below values are specified just for the description, uwb reading nodes have their own yaml files.
export DINGO_UWB_TAG_1_ID="1DB8" # right
export DINGO_UWB_TAG_1_OFFSET="0.177 -0.24 0.543175" # wrt mid mount in the description
export DINGO_UWB_TAG_2_ID="DA36" # left
export DINGO_UWB_TAG_2_OFFSET="0.143 0.24 0.543175" # wrt mid mount in the description