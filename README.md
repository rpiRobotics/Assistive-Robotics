# Assistive Robotics
This is a repository for Assitive Robotics Project that utilizes RPI manifactured Omnidirectional Assistive Robots (OARbots)
# The Devices In The Lab:

| Description             | Username      | Hostname (Computer Name) | IP            | Password  | OS           | ROS     |
| ---                     | ---           | ---                      | ---           | ---       | ---          | ---     |
| Pendant Tablet          | tablet        | tablet20                 | 192.168.1.99  | 1234      | Ubuntu 20.04 | Noetic  |
| Main Computer           | rockie        | rockie-20                | 192.168.1.100 | rockie    | Ubuntu 20.04 | Noetic |
| Oarbot (Silver)         | oarbot_silver | oarbot-silver-NUC        | 192.168.1.101 | 1234      | Ubuntu 20.04 | Noetic |
| Oarbot (Blue)           | oarbot_blue   | oarbot-blue-NUC          | 192.168.1.102 | 1234      | Ubuntu 20.04 | Noetic |

# Setting up the system

Simply run
```
./all_devices_initial_setup.bash
```

# Setting Up Kinect Azure for Oarbots
Setup the Sensor SDK and Body SDK for Kinect Azure following https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download and https://docs.microsoft.com/en-us/azure/kinect-dk/body-sdk-download . 

At the time of this document the support is only for Ubuntu 18.04 with an AMD64 architecture CPU, additionally a dedicated GPU is a must (with minimum model specified as NVIDIA Geforce GTX 1050 see [here](https://docs.microsoft.com/en-us/azure/kinect-dk/system-requirements) for other requirements) 

The steps are summarized as:

## Ssh into the robots
The passwords are given at the top for the oarbots, using the information given ssh into the robots for example,
```
ssh oarbot_blue@192.168.1.101
```
or 
```
ssh oarbot_silver@192.168.1.102
```

Then on each robot, follow the steps below:
## Setting up Linux Software Repository for Microsoft Products
```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
```
## Setting up the SDKs
```
sudo apt install libk4a1.4-dev
sudo apt install libk4abt1.1-dev
sudo apt install k4a-tools
```
## Adding permissions for Azure Kinect as a usb device
```
cd /etc/udev/rules.d
sudo wget https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules
```

Plug in the USB of Kinect Azure and connect it to the power. Then you can verify the installation with `k4aviewer` or `k4abt_simple_3d_viewer` commands. 

## Setting up the ROS packages for Kinect Azure
(See the instructions for building [here](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/building.md) as reference)
```
sudo apt-get install ros-melodic-rgbd-launch
cd ~/catkin_ws_assistive/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
cd ..
catkin_make -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;assistive_launch;arduino_talker'
#(or catkin_make --force_cmake -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;assistive_launch;arduino_talker')
source ~/.bashrc
source ~/catkin_ws_assistive/devel/setup.bash
```
## Adjusting Default launch parameters for Azure Kinect
You can edit the default FPS argument value to 30 in `/src/Azure_Kinect_ROS_Driver/launch/kinect_rgbd.launch`.

Finally you can verify Kinect Ros installation working by
```
roslaunch azure_kinect_ros_driver kinect_rgbd.launch
```
and in a new terminal you can check for the published images with
```
rqt_image_view
```

For further information about the topics and the usage see https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/usage.md 

# Setting Up Kinova Arm for Oarbots

TODO
