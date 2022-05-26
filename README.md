# Assistive Robotics
This is a repository for Assitive Robotics Project that utilizes RPI manifactured Omnidirectional Assistive Robots (OARbots)
# The Devices In The Lab:

| Description             | Username      | Hostname (Computer Name) | IP            | Password  | OS           | ROS     |
| ---                     | ---           | ---                      | ---           | ---       | ---          | ---     |
| Pendant Tablet          | tablet        | tablet20                 | 192.168.1.99  | 1234      | Ubuntu 20.04 | Noetic  |
| Main Computer           | razer         | razer-18                 | 192.168.1.100 | 1234      | Ubuntu 18.04 | Melodic |
| Oarbot (Silver)         | oarbot_silver | oarbot-silver-P15        | 192.168.1.101 | 1234      | Ubuntu 18.04 | Melodic |
| Oarbot (Blue)           | oarbot_blue   | oarbot-blue-P15          | 192.168.1.102 | 1234      | Ubuntu 18.04 | Melodic |
| NUC for Kinect          | nuc           | nuc-18                   | 192.168.1.90  | 1234      | Ubuntu 18.04 | Melodic |

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
git clone https://github.com/burakaksoy/Azure_Kinect_ROS_Driver.git
cd ..
catkin_make -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;assistive_launch;tablet_arduino_talker'
#(or catkin_make --force_cmake -DCATKIN_BLACKLIST_PACKAGES='assistive_gui;assistive_launch;tablet_arduino_talker')
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
## Hardware Setup

1. Plugin the powercable and the joystick
2. Connect the USB cable to your laptop
3. Powerup the arm

The green light on the joystick will be flashing for around a minute. After that, there will be two results after you do that
### Joystick Steady Green light
You are good to go! Please direct to [software setup](#software-setup).

### Joystick Green Light flashing or Steady Red Light
You need to update the firmware of the arm. If you try to use the ROS package when this situation happens, it'll show connection error.

1. Please follow the instruction in this [webpage](https://github.com/Kinovarobotics/kinova-ros/issues/248).
2. You can find the latest firmware [here](https://www.kinovarobotics.com/en/resources/gen2-technical-resources).
3. You can find the installation files of Development Center, a GUI available with the SDK [here](https://www.kinovarobotics.com/en/resources/gen2-faq) in the first question of "Usage". If the download link is missing, you can find it [here](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view) (File name is "`PS 0000 0009_1.5.1.zip`") and the document [here](https://www.kinovarobotics.com/sites/default/files/UG-008_KINOVA_Software_development_kit-User_guide_EN_R02%20%281%29.pdf) or [here](https://drive.google.com/file/d/1y5TByFsF97s4_zh14E-q0YHSlMr5P1qe/view).
4. We only tested the Development Center in win10 while updating the firmware. However, Development Center in both win10 and ubuntu20.04 (using the ubuntu16.04 version in the file) can connected to the arm and control the arm.
5. While rebooting the arm in the updating firmware step, you might want to do some "physical therapy" for the arm (basically move the arms around) and wait a bit before restarting your arm.

## Software Setup

### Control the arm via Development Center
You can find the installation files of Development Center, a GUI available with the SDK [here](https://www.kinovarobotics.com/en/resources/gen2-faq) in the first question of "Usage". If the download link is missing, you can find it [here](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view) and the document [here](https://www.kinovarobotics.com/sites/default/files/UG-008_KINOVA_Software_development_kit-User_guide_EN_R02%20%281%29.pdf).

### Control the arm via ROS package
Please direct [here](https://github.com/eric565648/kinova-ros/tree/noetic-devel) for how to launch the arm, perform joint angle and cartetian space control.

### Put the arm to service mode
- Plug in USB your robot 
- Open the `C:\Program Files (x86)\JACO-SDK\RobotConfiguration` or the equivalent path on you system.
- Put the last firmware you upload in your robot in the firmware folder.
- Run `ActivateServiceConfiguration.exe` (it is a console app, see the outputs by running it from a terminal/command line)
If that works well at  General setting > update at the arm type field you will see JAco v2 6dof Service.

### To use admittance control 
1. Open Development Center GUI, general setting > update 
2. Verify that main firmware is updated. 
3. Arm type is in service mode (see above).
4. Actuator firmware is updated.
5. Verify that all the torque sensors are functional. (see details below)
6. Do the torque calibration. (see details below)
7. Verify that the gravity vector and the payload correctly defined. (see details below)

#### Steps 1-4 are already done in the above.

#### 5. Verifying all torque sensors are functional
Open the Development Center and go to Monitoring >angular >torque  column
- Push against each joint to apply an external torque to each joint and observe if the value on screen reacts
  accordingly to your action. Push in both direction and the torque should be positive in one direction and
            negative in the opposite one.
- If it does not react as expected, You have a problem with one or more of your torque sensor.
- If it reacts as expected, try step 6.

#### 6. Doing the torque calibration
Calibrate arm torque by reset sensors to zero value.
Get the user Guide. Go at the page 58 and place the robot at the picture position (candle like position)
- Once the arm is in the right position, you have to open the Development Center and go to Advanced settings and press 4 times to the Apply to all button in torque zero menu.
-  Try to put the arm in torque mode with torque console, but close the Development Center before opening Torque console.
-  If the arm does not switch to torque control try step 7.  

#### 7. Verify that the gravity vector and the payload correctly defined.
- Open the Torque Console Interface and set the Gravity vector. 
- Then set the payload if you have one.
- When it is done, try to switch to torque mode with the Torque Console Interface
 
If after all of those step it continue to not work, it may be a hardware issue. That need further investigation 

# Installing Ubuntu 18.04 to Lenovo Thinkpad P15 Gen2 Laptops
 See the hardware specs of this computer model in https://psref.lenovo.com/syspool/Sys/PDF/ThinkPad/ThinkPad_P15_Gen_2/ThinkPad_P15_Gen_2_Spec.pdf
 
 We have 
 CPU: Core i7-11850H
 GPU: NVIDIA T1200 (4GB GDDR4, 60W)
 RAM: 16 GB DDR4-3200MHz
 SCREEN: 1080P IPS
 STORAGE: 512GB SSD M.2 2280 SSD PCIe NVMe
 WIFI: Intel Wi-Fi® 6E AX210, 802.11ax 2x2 Wi-Fi + Bluetooth 5.2
 
 ## Prepare the BIOS settings
 As described here: https://download.lenovo.com/pccbbs/mobiles_pdf/tp_p1_gen2_ubuntu_18.04_lts_installation_v1.0.pdf
 (Note that this file is for P1, not for P15, so not all of steps here works for installing ubuntu 18.04 to P15)
 - Disable "OS Optimized Defaults" in "Restart"
 - Make sure "Hybrid Graphics" is selected instead of "Discrete Graphics" under Config>Display settings. Later we will have to make this "Discrete Graphics" again.
 - F10 "Save & Exit"
 ## Install Ubuntu 18.04 with a USB
 - Insert Ubuntu 18.04 installation USB and boot up the laptop.
 - Press "Enter" while you are booting.
 - Press "F12" to select the USB device.
 - In the GRUB menu select "Install Ubuntu"
 - Make sure you are connected to internet via Ethernet (WIFI does not work for now)
 - Select Normal installation,
 - For the Other options select "Download Updates while installing Ubuntu" and "Install third-party software for graphics and Wifi hardware..." options
 - Do the storage managements as you wish (I did without encryption)
 - For the username and computer name use the devices info table at the top of this readme file.
 - After the installation if the computer does not boot after the GRUB menu, (see https://askubuntu.com/questions/162075/my-computer-boots-to-a-black-screen-what-options-do-i-have-to-fix-it.)
  - Basically you will boot with nomodeset option  in the grub menu instead of quiet splash and then you will install graphics card after you boot in
  - While booting keep pressing "shift" buttons to see GRUB menu,
  - Press "e" while the OS that you would like to boot is selected.
  - Go to line that starts with "linux" and erase "quiet splash" and write "nomodeset" instead.
  - Press "Ctrl+x", you will be able to boot
  - We will have to install NVIDIA graphics card and WIFI drivers
## Install NVIDIA drivers
- open a terminal and do `sudo apt update` and `sudo apt upgrade`
- press windows key and search for "Software & Updates"
- From ubuntu Software tab select Download From Main Server
- From Updates tab Select Never for Notify me of a new Ubuntu version
- From Additional Drivers tab select Using Nvidia driver metapackage from nvidia-driver495 (proprietary)
- Press Apply Changes button and once it finished reboot
- This will install the NVIDIA driver but you can also see options of the section 5 of [this](https://download.lenovo.com/pccbbs/mobiles_pdf/tp_p1_gen2_ubuntu_18.04_lts_installation_v1.0.pdf) for other installation options (We did not try those)
- When it boots up, go to BIOS settings and select "Discrete Graphics" instead of "Hybrid Graphics" under Config>Display settings.
- Save and Exit
- Once you reboot, now without doing the "nomodeset" step in GRUB menu you will be able to login
## Install Wifi Drivers
At the time of this installation, the most up-to-date Ubuntu 18.04 kernel version is 5.4.0 (you can check yours with command `uname -a`). However, the wifi hardware used on this computer (Intel Wi-Fi® 6E AX210) requires at least kernel version 5.10 (You can verify this at [here](https://wireless.wiki.kernel.org/en/users/drivers/iwlwifi)).  
As stated in one of the answers in [this link](https://ubuntu.forumming.com/question/14149/ubuntu-20-04-lts-driver-intel-wi-fi-6e-ax210-160mhz), "The Linux 5.10 kernel (or later) will ship as part of Ubuntu 21.04 in April. This version will also get backported to Ubuntu 20.04 LTS at a later date. It's possible to manually install a mainline kernel in Ubuntu however if it breaks you get the pieces." We will install kernel 5.11 to make the wifi adapter work, but as suggested in the same answer be warned to review the implications of installing a kernel version manually [here](https://askubuntu.com/questions/119080/how-to-update-kernel-to-the-latest-mainline-version-without-any-distro-upgrade/885165#885165). 
### Installing Kernel 5.11
- First install Mainline as a graphical kernel installing tool. (See details [here](https://ubuntuhandbook.org/index.php/2020/08/mainline-install-latest-kernel-ubuntu-linux-mint/))
- run `sudo add-apt-repository ppa:cappelikan/ppa`
- `sudo apt update`
- `sudo apt install mainline`
- Open Mainline Kernel Installer and install 5.11.0
- After installation, reboot.
- `sudo update-grub` and `sudo reboot`
- As described [here](https://github.com/spxak1/weywot/blob/main/ax210.md#boot-with-kernel-5101), the output of `sudo dmesg | grep iwl` will show us some errors with the information about which firmware we need to install. 
- For example we needed `iwlwifi-ty-a0-gf-a0-39` to `iwlwifi-ty-a0-gf-a0-59`.
### Installing Firmware
- At the output of dmesg command it is suggested to check https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/
- Go this website and download the newest firmware (eg. as of today it was linux-firmware-20211027.tar.gz (sig))
- It takes some time to download, be patient
- Uncompress the file with `tar -zxvf linux-firmware-20211027.tar.gz`
- `cd linux-firmware-20211027/`
- Copy the firmwares to `/lib/firmware/` with command `sudo cp -ax * /lib/firmware`
- Now reboot and the wifi should work!
## Additional settings 
- Connect the wifi to `OARBOT_5G`
- Open settings on Privacy tab, disable automatic screen lock, location services enabled.
- Sharing tab, enable sharing and screen sharing, select require a password and make the password `1234`
- Power tab, disable dim screen when inactive, blank screen 5 minutes, Automatic suspend OFF, When power button is pressed Power Off
- Details tab, Users tab, Unlock and enable automatic log in
- Install GNOME Tweaks and launch
- On Power tab disable suspend when laptop lid is closed
- Install Dconf Editor and launch
- on /org/gnome/desktop/remote-access, disable require-encryption
- `sudo apt install ssh`
## Install ROS Melodic
- See [here](http://wiki.ros.org/Installation/Ubuntu)
  
# Usage

run 
```
./test_launcher_all_oarbots.bash
```

Then to correct the torque readings on Kinova arms, run:
```
./correct_kinova_torques.bash
```

To plot the Force/Torque plots either run:
```
./test_launcher_rqt_plotter.bash
```

Or if you have PlotJuggler installed, run:
```
./test_launcher_plotjuggler.bash
```