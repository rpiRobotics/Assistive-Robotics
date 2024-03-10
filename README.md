# Assistive Robotics

This is a repository for multi-robot assitive applications that utilizes Clearpath Dingo Omni-directional robots.

# Computers in the System:


| Description                     | Username      | Hostname (Computer Name) | MAC Address       | IP            | Password  | OS           | ROS     |
| ------------------------------- | ------------- | ------------------------ | ----------------- | ------------- | --------- | ------------ | ------- |
| GUI Tablet                      | tablet        | tablet20                 | AC:67:5D:57:30:50 | 192.168.1.99  | 1234      | Ubuntu 20.04 | Noetic  |
| Main Computer                   | ???           | ?????                    | 3C:8C:F8:EB:0E:C8 | 192.168.1.100 | ????      | Ubuntu 20.04 | Noetic  |
| Dingo 1 Base (Right Arm Config) | administrator | cpr-do100-10000050       | 0C:7A:15:CB:A6:0B | 192.168.1.101 | clearpath | Ubuntu 20.04 | Noetic  |
| Dingo 1 Helper                  | oarbot_blue   | oarbot-blue-P15          | F4:46:37:FF:E5:A1 | 192.168.1.102 | 1234      | Ubuntu 18.04 | Melodic |
| Dingo 2 Base (Left Arm Config)  | administrator | cpr-do100-10000051       | F0:B6:1E:B7:29:1F | 192.168.1.103 | clearpath | Ubuntu 20.04 | Noetic  |
| Dingo 2 Helper                  | oarbot_silver | oarbot-silver-P15        | 4C:77:CB:EC:81:87 | 192.168.1.104 | 1234      | Ubuntu 18.04 | Melodic |
| Overhead Camera Kinect Processor| nuc           | nuc-18                   | 1C:69:7A:0D:A3:E0 | 192.168.1.90  | 1234      | Ubuntu 18.04 | Melodic |

# Setting up the system

<details> 
    <summary>Click to expand</summary>

<!-- Simply run
```
./all_devices_initial_setup.bash
``` -->

</details>

<!-- --------------------------------------------------------------------------- -->

# Setting up the Helper Computers

<details> 
    <summary>Installing Ubuntu 18.04 to Lenovo Thinkpad P15 Gen2 Laptops</summary>

## Installing Ubuntu 18.04 to Lenovo Thinkpad P15 Gen2 Laptops

See the hardware specs of this computer model in https://psref.lenovo.com/syspool/Sys/PDF/ThinkPad/ThinkPad_P15_Gen_2/ThinkPad_P15_Gen_2_Spec.pdf

We have
CPU: Core i7-11850H
GPU: NVIDIA T1200 (4GB GDDR4, 60W)
RAM: 16 GB DDR4-3200MHz
SCREEN: 1080P IPS
STORAGE: 512GB SSD M.2 2280 SSD PCIe NVMe
WIFI: Intel Wi-Fi® 6E AX210, 802.11ax 2x2 Wi-Fi + Bluetooth 5.2

### Prepare the BIOS settings

As described here: https://download.lenovo.com/pccbbs/mobiles_pdf/tp_p1_gen2_ubuntu_18.04_lts_installation_v1.0.pdf
(Note that this file is for P1, not for P15, so not all of steps here works for installing ubuntu 18.04 to P15)

- Disable "OS Optimized Defaults" in "Restart"
- Make sure "Hybrid Graphics" is selected instead of "Discrete Graphics" under Config>Display settings. Later we will have to make this "Discrete Graphics" again.
- F10 "Save & Exit"

### Install Ubuntu 18.04 with a USB

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

### Install NVIDIA drivers

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

### Install Wifi Drivers

At the time of this installation, the most up-to-date Ubuntu 18.04 kernel version is 5.4.0 (you can check yours with command `uname -a`). However, the wifi hardware used on this computer (Intel Wi-Fi® 6E AX210) requires at least kernel version 5.10 (You can verify this at [here](https://wireless.wiki.kernel.org/en/users/drivers/iwlwifi)).
As stated in one of the answers in [this link](https://ubuntu.forumming.com/question/14149/ubuntu-20-04-lts-driver-intel-wi-fi-6e-ax210-160mhz), "The Linux 5.10 kernel (or later) will ship as part of Ubuntu 21.04 in April. This version will also get backported to Ubuntu 20.04 LTS at a later date. It's possible to manually install a mainline kernel in Ubuntu however if it breaks you get the pieces." We will install kernel 5.11 to make the wifi adapter work, but as suggested in the same answer be warned to review the implications of installing a kernel version manually [here](https://askubuntu.com/questions/119080/how-to-update-kernel-to-the-latest-mainline-version-without-any-distro-upgrade/885165#885165).

#### Installing Kernel 5.11

- First install Mainline as a graphical kernel installing tool. (See details [here](https://ubuntuhandbook.org/index.php/2020/08/mainline-install-latest-kernel-ubuntu-linux-mint/))
- run `sudo add-apt-repository ppa:cappelikan/ppa`
- `sudo apt update`
- `sudo apt install mainline`
- Open Mainline Kernel Installer and install 5.11.0
- After installation, reboot.
- `sudo update-grub` and `sudo reboot`
- As described [here](https://github.com/spxak1/weywot/blob/main/ax210.md#boot-with-kernel-5101), the output of `sudo dmesg | grep iwl` will show us some errors with the information about which firmware we need to install.
- For example we needed `iwlwifi-ty-a0-gf-a0-39` to `iwlwifi-ty-a0-gf-a0-59`.

#### Installing Firmware

- At the output of dmesg command it is suggested to check https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/
- Go this website and download the newest firmware (eg. as of today it was linux-firmware-20211027.tar.gz (sig))
- It takes some time to download, be patient
- Uncompress the file with `tar -zxvf linux-firmware-20211027.tar.gz`
- `cd linux-firmware-20211027/`
- Copy the firmwares to `/lib/firmware/` with command `sudo cp -ax * /lib/firmware`
- Now reboot and the wifi should work!

### Additional settings

- Connect the wifi to lab network.
- Open settings on Privacy tab, disable automatic screen lock, location services enabled.
- Sharing tab, enable sharing and screen sharing, select require a password and make the password `1234`
- Power tab, disable dim screen when inactive, blank screen 5 minutes, Automatic suspend OFF, When power button is pressed Power Off
- Details tab, Users tab, Unlock and enable automatic log in
- Install GNOME Tweaks and launch
- On Power tab disable suspend when laptop lid is closed
- Install Dconf Editor and launch
- on /org/gnome/desktop/remote-access, disable require-encryption
- `sudo apt install ssh`

### Install ROS Melodic

- See [here](http://wiki.ros.org/Installation/Ubuntu)

</details>

<details> 
    <summary>Setting Up Kinect Azure</summary>

## Setting Up Kinect Azure

Setup the Sensor SDK and Body SDK for Kinect Azure following https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download and https://docs.microsoft.com/en-us/azure/kinect-dk/body-sdk-download .

At the time of this document the support is only for Ubuntu 18.04 with an AMD64 architecture CPU, additionally a dedicated GPU is a must (with minimum model specified as NVIDIA Geforce GTX 1050 see [here](https://docs.microsoft.com/en-us/azure/kinect-dk/system-requirements) for other requirements)

The steps are summarized as:

### Ssh into the Kinect Processor Computer of the robots

The passwords are given at the top for the  Kinect Processors. Use the information given and ssh into the robots. For example,

```
ssh oarbot_blue@192.168.1.101
```

or

```
ssh oarbot_silver@192.168.1.102
```

Then on each robot, follow the steps below:

### Setting up Linux Software Repository for Microsoft Products

```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
```

### Setting up the SDKs

```
sudo apt install libk4a1.4-dev
sudo apt install libk4abt1.1-dev
sudo apt install k4a-tools
```

### Adding permissions for Azure Kinect as a usb device

```
cd /etc/udev/rules.d
sudo wget https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules
```

Plug in the USB of Kinect Azure and connect it to the power. Then you can verify the installation with `k4aviewer` or `k4abt_simple_3d_viewer` commands.

### Setting up the ROS packages for Kinect Azure

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

### Adjusting Default launch parameters for Azure Kinect

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

</details>

<details> 
    <summary>Setting Up Kinova Arm for robots</summary>

## Setting Up Kinova Arm for robots

### Hardware Setup

1. Plugin the powercable and the joystick
2. Connect the USB cable to your laptop
3. Powerup the arm

The green light on the joystick will be flashing for around a minute. After that, there will be two results after you do that

#### Joystick Steady Green light

You are good to go! Please direct to [software setup](#software-setup).

#### Joystick Green Light flashing or Steady Red Light

You need to update the firmware of the arm. If you try to use the ROS package when this situation happens, it'll show connection error.

1. Please follow the instruction in this [webpage](https://github.com/Kinovarobotics/kinova-ros/issues/248).
2. You can find the latest firmware [here](https://www.kinovarobotics.com/en/resources/gen2-technical-resources).
3. You can find the installation files of Development Center, a GUI available with the SDK [here](https://www.kinovarobotics.com/en/resources/gen2-faq) in the first question of "Usage". If the download link is missing, you can find it [here](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view) (File name is "`PS 0000 0009_1.5.1.zip`") and the document [here](https://www.kinovarobotics.com/sites/default/files/UG-008_KINOVA_Software_development_kit-User_guide_EN_R02%20%281%29.pdf) or [here](https://drive.google.com/file/d/1y5TByFsF97s4_zh14E-q0YHSlMr5P1qe/view).
4. We only tested the Development Center in win10 while updating the firmware. However, Development Center in both win10 and ubuntu20.04 (using the ubuntu16.04 version in the file) can connected to the arm and control the arm.
5. While rebooting the arm in the updating firmware step, you might want to do some "physical therapy" for the arm (basically move the arms around) and wait a bit before restarting your arm.

### Software Setup

#### Control the arm via Development Center

You can find the installation files of Development Center, a GUI available with the SDK [here](https://www.kinovarobotics.com/en/resources/gen2-faq) in the first question of "Usage". If the download link is missing, you can find it [here](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view) and the document [here](https://www.kinovarobotics.com/sites/default/files/UG-008_KINOVA_Software_development_kit-User_guide_EN_R02%20%281%29.pdf).

#### Control the arm via ROS package

Please direct [here](https://github.com/eric565648/kinova-ros/tree/noetic-devel) for how to launch the arm, perform joint angle and cartetian space control.

#### Put the arm to service mode

- Plug in USB your robot
- Open the `C:\Program Files (x86)\JACO-SDK\RobotConfiguration` or the equivalent path on you system.
- Put the last firmware you upload in your robot in the firmware folder.
- Run `ActivateServiceConfiguration.exe` (it is a console app, see the outputs by running it from a terminal/command line)
  If that works well at  General setting > update at the arm type field you will see JAco v2 6dof Service.

#### To use admittance control

1. Open Development Center GUI, general setting > update
2. Verify that main firmware is updated.
3. Arm type is in service mode (see above).
4. Actuator firmware is updated.
5. Verify that all the torque sensors are functional. (see details below)
6. Do the torque calibration. (see details below)
7. Verify that the gravity vector and the payload correctly defined. (see details below)

##### Steps 1-4 are already done in the above.

##### 5. Verifying all torque sensors are functional

Open the Development Center and go to Monitoring >angular >torque  column

- Push against each joint to apply an external torque to each joint and observe if the value on screen reacts
  accordingly to your action. Push in both direction and the torque should be positive in one direction and
  negative in the opposite one.
- If it does not react as expected, You have a problem with one or more of your torque sensor.
- If it reacts as expected, try step 6.

##### 6. Doing the torque calibration

Calibrate arm torque by reset sensors to zero value.
Get the user Guide. Go at the page 58 and place the robot at the picture position (candle like position)

- Once the arm is in the right position, you have to open the Development Center and go to Advanced settings and press 4 times to the Apply to all button in torque zero menu.
- Try to put the arm in torque mode with torque console, but close the Development Center before opening Torque console.
- If the arm does not switch to torque control try step 7.

##### 7. Verify that the gravity vector and the payload correctly defined.

- Open the Torque Console Interface and set the Gravity vector.
- Then set the payload if you have one.
- When it is done, try to switch to torque mode with the Torque Console Interface

If after all of those step it continue to not work, it may be a hardware issue. That need further investigation

</details>

</details>

<!-- --------------------------------------------------------------------------- -->

# Setting Up Dingo Base Computers

<details> 
    <summary>Getting Started</summary>

## Getting Started

Four Dingo-O robots arrived with a printed document named "Custom Robot Quickstart Guide". We only needed to apply section 3 and section 5 of this document after the batteries are fully charged (both the robot batteries and the PlayStation controller batteries). The texts are in these sections are copied below:

### Section 3: Getting Started

Your system has been configured to allow you to get started immediately after receipt. Follow these instructions to get
moving.

1. Remove the Dingo's side panels and top fairings (yellow), insert the batteries provided (or confirm they are
   inserted), then replace the top fairings and side panels.
2. Turn on the Dingo via the HMI button pad on the rear. Note that the computer may beep when starting up.
3. Press "PS" button on gamepad to turn it on.

### Section 5: Wireless

To set up the wireless communications on your Dingo, you must either first establish a wired Ethernet connection or use the HDMI output and USB keyboard from the robots's computer. If you use the Ethernet option, connect your computer to an Ethernet port on the Dingo's computer by removing the Dingo fairing, and set a
static IP on your computer to `192.168.131.19` (for example). If there are no free ethernet ports you may temporarily disconnect
one of the payloads such as a lidar sensor. SSH into the robot computer with:

```
ssh administrator@192.168.131.1
```

Enter the login password when prompted. Once you have successfully logged in, you can connect the robot's computer to a desired wireless network.

If you use the HDMI and USB keyboard option, when you bootup the robot, the login terminal screen of Ubuntu comes up. Enter in the username and password given at the top of this document and log in.

Once you log in, you can connect your robot to a desired wireless network using Netplan.

Simply create a file called `60-wireless.yaml` inside of the `/etc/netplan folder` on your robot's computer. Copy and paste
the contents below into the file, and make sure to modify the wireless interface, SSID, and password fields.

```txt
network:
    wifis:
    # Replace WIRELESS_INTERFACE with the name of the wireless network device, e.g. wlane or wlp3s0
    # Fill in the SSID_GOES_HERE and PASSWORD_GOES_HERE fields as appropriate. The password may be included
    as plain-text
    # or as a password hash. To generate the hashed password, run
    #
    echo -n 'WIFI_PASSWORD' | iconv -t UTF-16LE | openssl md4 -binary | xxd -p
    # If you have multiple wireless cards you may include a block for each device.
    # For more options, see https://netplan.io/reference/
    WIRELESS_INTERFACE:
        optional: true
        access-points:
            SSID_GOES HERE:
            password: PASSWORD_GOES_HERE
        dhcp4: true
        dhcp4-overrides:
            send-hostname: true
```

Once you have saved the file, you will then need to apply your new Netplan configuration and bring up your wireless
connection by running:

```bash
sudo netplan apply
```

More advanced networking examples, including configurations for accessing a wifi network requiring WPA Enterprise
credentials, can be found here:
https://netplan.io/examples/

You can verify that your robot is connected to a wireless network by running:

```bash
ip a
```

This will show all active connections and their IP addresses, including your robot's connection to the desired wireless
network, and the IP address assigned to the robot's computer.

</details>

<details> 
    <summary>Needed Software Customizations on a factory fresh Dingo-O robot</summary>

## Needed Software Customizations on a factory fresh Dingo-O robot

**WARNING: You may want to create a backup of the edited files before begin the process described here.**
To make the dingo robots work as desired in this repository, there are some customizations needed.

Follow the instructions in `ReadMe.md` file of in `src/dingo_customization/` of this repository.

</details>


<details> 
    <summary>Physical UWB setup</summary>

## Physical UWB setup

### Related websites for the Qorvo (DecaWave) UWB module documents

DW1000 [https://www.qorvo.com/products/p/DW1000#documents](https://www.qorvo.com/products/p/DW1000#documents)

DW1001C [https://www.qorvo.com/products/p/DWM1001C#documents](https://www.qorvo.com/products/p/DWM1001C#documents)

DWM1001-DEV [https://www.qorvo.com/products/p/DWM1001-DEV#documents](https://www.qorvo.com/products/p/DWM1001-DEV#documents)

MDEK1001 [https://www.qorvo.com/products/p/MDEK1001#documents](https://www.qorvo.com/products/p/MDEK1001#documents)

### Download the Android DRTLS phone app

[https://www.qorvo.com/products/p/MDEK1001#documents](https://www.qorvo.com/products/p/MDEK1001#documents)

Download DRTLS App : Android Application APK

### Calibration Script

Used to determine the every module's (tags and anchors) offsets based on [this white paper with name: Antenna Delay Calibration of DW1000-Based Products and Systems (Application Note APS014)](https://www.qorvo.com/products/d/da008449).

Set 4 of them an on a nice square with best possible known manual position measurements.

(3 of them gives only one solution, 4 of them gives a Least Squares solution with RMSE error to have an idea of how accurate the calculated offsets are.)

Take note of the manually measured distances, they are needed in the calibration script.

From the android app, put all the modules in the same network and set them as anchors. From the powered ones, only one of them must be set as initiator.

Use `antenna_offset_finding.m` MATLAB script in `uwb_matlab_scripts/` directory of this repo to find the offsets of each UWB module. Then set the offsets in `antenna_calibration.yaml` in `src/assistive_launch/config/` folder. Comments of the script should be sufficient to guide you for further details.

Note: This script would work on Windows 10 but not in Windows 11 as of writing this document. See details [here](https://www.mathworks.com/matlabcentral/answers/1912280-bluetooth-scanning-error-in-windows-11-solutions#answer_1173820)

This script uses the BLE interface of the firmware to communicate with the tags. For further information see section 7 of [DWM1001 Firmware API Guide](https://www.qorvo.com/products/d/da007975)

After the calibration is done, set modules back as tags those you won't to use as anchors from the Android app.

### Script to Calculate and Write the Anchor positions into the Modules

Mount the UWB anchors in the physical workspace area. Measure the distances between them with a laser distance meter.
Use `truck_bay_uwb_locations.m`  MATLAB script in `uwb_matlab_scripts/` directory of this repo. Comments of the script should be sufficient to guide you for further details.

Note: This script would work on Windows 10 but not in Windows 11 as of writing this document. See details [here](https://www.mathworks.com/matlabcentral/answers/1912280-bluetooth-scanning-error-in-windows-11-solutions#answer_1173820)

This script uses the BLE interface of the firmware to communicate with the tags. For further information see section 7 of [DWM1001 Firmware API Guide](https://www.qorvo.com/products/d/da007975)

### Configuring USB Ports of the UWB Tags Mounted on the Robots

At each robot equipped with the UWB tags, this step is required to read data from the UWB tags. 

Determine the USB serial ports of the UWB tags by unplugging and re-pluggin the USB cables of the tags and using this command:

```
ls /dev/serial/by-path/
```

The determined port should be something like this:

```
/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0
```

For each UWB tag, edit the `<robot_name>_uwb_<# of the UWB tag>.yaml` config files in `assistive_launch/config/` directory and assign the `serial_port` parameter to the determined port. 

For example, we name the first Dingo robot as `d1` and the USB port of its `2`nd UWB tag is specified in `assistive_launch/config/d1_uwb_2.yaml` file with the following content:

```
serial_port: '/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0'
topic_name: 'uwb/tag_2_serial_ranging'
```



</details>

<!-- --------------------------------------------------------------------------- -->

# Setting Up Tablet (Graphical User Interface Device) Computer

The tablet device as the GUI of the system is based on 'ARM Smart Teacpendant Project' hardware.

<details> 
    <summary>Tablet OS Setup</summary>

## Tablet OS Setup

### Steps to install Ubuntu (20.04 or 18.04) on Surface Go 2 Tablet

Requires a USB-C to USB-A adapter and flash drive

1. Update Windows 10/11 using Settings -> Updates
2. Download and create Ubuntu 20.04 amd64 USB install drive
3. Disable Windows bitlocker and reboot. If it says “waiting to active”, finish activation, then disable: https://www.isunshare.com/windows-10/4-ways-to-remove-or-disable-bitlocker-encryption-on-windows-10.html
4. Shrink the Windows 10 partition using Windows disk manager: https://www.tenforums.com/tutorials/96288-shrink-volume-partition-windows-10-a.html Suggested to shrink by 64000 MB
5. Connect bootable USB drive and reboot using advanced startup options: https://www.digitalcitizen.life/boot-your-windows-10-pc-usb-flash-drive The bootable usb drive may have the title “Linpus Lite”
6. Install Ubuntu as normal
7. Remove the USB drive
8. At this point Ubuntu is installed, but will not boot automatically. Do the advanced startup options again, and select “ubuntu”. This will boot into Ubuntu.
   Follow these instructions in Ubuntu to disable Windows boot: https://www.reddit.com/r/SurfaceLinux/comments/egds33/possible_fix_for_booting_directly_to_grub_on/ Windows can still be booted using Grub menu
   Ubuntu should now boot. The post is copied here for convenience:

   ```
       Possible fix for booting directly to grub on Surface Go
       If you're having trouble getting your Surface Go to boot to grub instead of the Windows Boot Manager, I might have something to try if you're brave: I moved the Microsoft folder in /boot/efi/EFI out of the way (In Ubuntu: sudo mv /boot/efi/EFI/Microsoft /boot/efi/EFI/Microsoft.bak) and now grub is loaded by default. I'd really only recommend this if you:

       Have a Windows USB recovery made and you know it's bootable

       Have your files backed up off the SSD (both Linux and Windows (if you care))

       Feel comfortable screwing around fixing a potentially broken EFI partition

       Aren't the sort of person who blames other people when you break your own computer following instructions you found on the Internet!

       All that said, it works for me on my recently purchased 8GB/128GB Surface Go w/ Ubuntu 19.10. I had already dumped the WIndows partition though, so I never tested whether grub had any issues loading Windows. You may also need to mess around with efibootmgr to fix the boot order, but I'm not sure.
   ```
9. You may also need to disable secure boot. This is achived from the BIOS settings. To enter the BIOS settings, while powering up the tablet, Press and hold the volume-up button on your Surface and at the same time, press and release the power button. When you see the Surface logo, release the volume-up button.
   The UEFI menu will display within a few seconds.
10. The default kernel version installed with ubuntu 20.04 as of writing this document is 5.15 however this version causes hanging problem when shutting down. Installing kernel version 5.4.243 via Mainline kernel installer resolves the issue for this specific tablet.
11. After the install of the OS, make sure the wifi power saving is disabled by editing:

    ```bash
    sudo nano /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
    ```

    By default there is:

    ```txt
    [connection]
    wifi.powersave = 3
    ```

    **Change the value to 2**. Then to take effect, run:

    ```bash
    sudo systemctl restart NetworkManager
    ```

    Disabling wifi power saving reduces the wifi latency by a considerable amount! For more information see [here](https://gist.github.com/jcberthon/ea8cfe278998968ba7c5a95344bc8b55).

</details>

<!-- --------------------------------------------------------------------------- -->

# System Bootup Process Instructions

<details> 
    <summary>Click to expand</summary>


***Warning:** The order in the following instructions are important for successful bootup process. Following the order especially in the Dingo battery powered up components (Dingo base computer, Dingo helper computer, and Kinova arm) is essential, because otherwise Dingo robots may shut down after ~30 seconds, probably as a default protective measurement by Clearpath (Further investigation is needed on this matter).*

1. Make sure that on each Dingo robot, the Kinova arm is switched off, the Dingo helper computer charger is unplugged, and the robot batteries are connected and charged. 
2. Turn on the master/main computer and run `roscore`.
3. Turn on the GUI tablet.
4. Make sure the overhead Kinect camera is plugged in to the power and connected to the Kinect Azure computer. 
5. Turn on the Overhead Camera (Kinect Azure) computer (NUC).
6. Turn on each Dingo robot by using the button on the robot rear. 
7. Wait for the robot computer to complete its boot up process. 

    *This will launch the default robot control nodes, IMU node, UWB localization nodes, and front and rear Lidar sensor nodes*
8. Switch on the Kinova arm on the robots.
9. Plug in the Dingo helper computer charger.
10. Turn on the Dingo helper computer.  
11. Wait for the Dingo helper computer complete its boot up process. 
12. On master computer, launch `` file. 

     *This launch file will remotely launch
    the GUI node on the GUI tablet,
    the overhead camera and ArUco tag detection robot localization related nodes on the Overhead Camera computer (NUC),
    the Kinova arm, FT Sensor, and Kinect cameras on the Kinova arm wrist on each Dingo helper computer.
    Also, in the master/main computer it will launch
    the human body localization fusion.*

    Note that further devices, nodes, and features can be added to each computer without updating the information written in the last explanation above. That paragraph is intended to cover the main idea of system architecture which explains what system component runs on which computer in the system and therefore to create a sense on the System Bootup Process Instructions.

</details>

<!-- --------------------------------------------------------------------------- -->

# Usage

<details> 
    <summary>Click to expand</summary>

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
./rqt_plotter.bash
```

Or if you have PlotJuggler installed, run:

```
./plotjuggler.bash
```

</details>
