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
source /home/administrator/catkin_ws/devel/setup.bash

# Any additional environment variables that depend on your workspace should be exported here
source /etc/clearpath-dingo.bash

# Add the config extras file:
export DINGO_CONFIG_EXTRAS=$(catkin_find rpi06_dingo config/localization.yaml --first-only)

# Add the front facing Hokuyo UST10LX LiDAR
export DINGO_LASER=true
export DINGO_LASER_MODEL=ust10
export DINGO_LASER_OFFSET="0 0 0"

# Add the rear facing Hokuyo UST10LX LiDAR
export DINGO_LASER_SECONDARY=true
export DINGO_LASER_SECONDARY_MODEL=ust10
export DINGO_LASER_SECONDARY_OFFSET="0 0 0"

# Add the Microstrain 3DMGX5 IMU
export DINGO_IMU_MICROSTRAIN=true
export DINGO_IMU_MICROSTRAIN_OFFSET="0 0 0.08"


