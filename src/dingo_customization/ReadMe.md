# Dingo_customization package 
Files in this package includes the all changes that we did on a physical Dingo-O robot for the Assistive Robot project. It is created as a ROS package only for the sake of finding its files with `$(find dingo_customization)` command in other launch files.

## Changed/New Files

The Backups for the edited files are saved in the `/backup` folder.

The edited/new file list as follows:

### Scripts (bash files)

* `/usr/sbin/ros-start`: We specify the `ROS_MASTER` as a remote computer,  define the robot's `ROS_IP`, and comment out the `ROS_HOSTNAME` in this file.
* `/etc/ros/setup.bash`: Includes new environment variable exports for custom name of the robot for namespacing and proper tf frame prefixing. Also has UWB tag id definitions as well as the LIDAR and IMU sensor configurations.

### Launch files

* `/etc/ros/noetic/ros.d/base.launch`: This file is a symbolic link to `/opt/ros/noetic/share/dingo_base/launch/base.launch`.
* `/etc/ros/noetic/ros.d/accessories.launch`: This file is a symbolic link to `/opt/ros/noetic/share/dingo_bringup/launch/accessories.launch`.
* `/opt/ros/noetic/share/dingo_description/launch/description.launch`: We specify the tf prefix of the robot and the uwb tag ids in the robot description file in this launch file. Also the new custom description xacro file of the robot that allows us to do that is specified in this file.
* `/opt/ros/noetic/share/dingo_control/launch/control.launch`: Control parameter file of the omni directional wheels, tf prefixes of the wheels and the odom and robot base frames specified for the knowledge of the joint_publisher and velocity_controllers, robot_localization package configurations with the integration of uwb sensors and custom twist_mux configurations are specified in this file. 

### URDF (.xacro) files

* `...../dingo_customization/dingo-o-prefixed.urdf.xacro`: This is our new file which includes our custom description file of the robot.

<!-- ### Config (.yaml) files

* `...../dingo_customization/config/control_omni.yaml`
* `...../dingo_customization/config/robot_localization.yaml` -->

## Commands to copy in a robot

For example, after ssh'ing into robot `d1`, the following commands copies the necessary files to the correct directories:

```bash
sudo cp ~/catkin_ws_assistive/src/dingo_customization/d1/scripts/ros-start /usr/sbin/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/d1/scripts/setup.bash /etc/ros/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/base.launch /etc/ros/noetic/ros.d/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/accessories.launch /etc/ros/noetic/ros.d/

sudo cp ~/catkin_ws_assistive/src/dingo_customization/d2/scripts/ros-start /usr/sbin/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/d2/scripts/setup.bash /etc/ros/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/base.launch /etc/ros/noetic/ros.d/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/accessories.launch /etc/ros/noetic/ros.d/

sudo cp ~/catkin_ws_assistive/src/dingo_customization/d3/scripts/ros-start /usr/sbin/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/d3/scripts/setup.bash /etc/ros/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/base.launch /etc/ros/noetic/ros.d/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/accessories.launch /etc/ros/noetic/ros.d/

sudo cp ~/catkin_ws_assistive/src/dingo_customization/d4/scripts/ros-start /usr/sbin/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/d4/scripts/setup.bash /etc/ros/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/base.launch /etc/ros/noetic/ros.d/
sudo cp ~/catkin_ws_assistive/src/dingo_customization/launch/accessories.launch /etc/ros/noetic/ros.d/
```

For other robots (`d2,d3,d4`), ssh into them and execute after replacing the `d1` parts in the commands.


## Detailed explanations of some changes done in this package

<details>

### Dingo Setup for Remote Master

<details>


Once each robot is connected to the wireless network with static IP addresses (that can be done through the router settings. We set the IP addresses as specified in the table at the top this document), we set each to use the same ROS master. To do
this do the following on each robot. First, run 

```bash
sudo nano /usr/sbin/ros-start
```

In this `ros-start` file, change the line `export ROS_MASTER_URI=http://127.0.0.1:11311` to  

```bash
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.101 (USE THE CORRECT IP ADRESS HERE)
```

and comment out the line `export ROS_HOSTNAME=$(hostname)`
to make sure that the robot uses the host machine as its ROS Master. 

The reason of doing this comes from the fact that the Clearpath has setup the starting of the ROS nodes of the robot as a service that is initated during the boot-up. [For further information about this see this link.](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/). Clearpath created a `dingo_bringup` package to achieve this service behaviour. The installation file is given [here](https://github.com/dingo-cpr/dingo_robot/blob/noetic-devel/dingo_bringup/scripts/install), and the step of doing this installation on a fresh install from scracth is explained in [here](https://docs.clearpathrobotics.com/docs/robots/indoor_robots/dingo/tutorials_dingo#installing-dingo-software).

**For the final version of the changes made in this section, see `/src/dingo_customization/d1/scripts/ros-start` file. We basically replace the default file with this custom file.**

<pre><del> To make sure that ros.service by Clearpath starts after the network is really online,
edit `ros.service` file with command
`sudo nano /lib/systemd/system/ros.service`
and add the following lines  

```txt
After=network-online.target
Wants=network-online.target
```

in place of the line

```txt
After=network.target
```

[For further information about this above see this link.](https://www.freedesktop.org/wiki/Software/systemd/NetworkTarget/) </del></pre>

After these changes, also add the following lines in the master computer's `~/.bashrc` file:  

```bash
export ROS_IP=192.168.1.100
export ROS_MASTER_URI=http://192.168.1.100:11311/
```

**Note that these changes on the robots will make the robots to look for the master computer running the `roscore` command while they are booting. If the `roscore` is not running on the master computer during the booting of the robots, the robots will be able to boot correctly and connect to the WiFi. However, the `ros.service` of `systemctl` will fail and therefore the robot will not be able to move (the comms and Wi-Fi indicator LEDs will be off on the robot's HMI interface). If you run the `roscore` command on the master computer after the robots are booted up, you need to manually start the `ros.service` on the robots by ssh'ing into them. This is achieved by running this command on the robot terminals:**  

```bash
sudo systemctl start ros.service
```

</details>

### Namespacing the Dingo Robots

<details>

After the factory default install of Dingo-O, there are two files in `/etc/ros/noetic/ros.d` named `base.launch` and `accessories.launch`. Those file are actually symbolic links to the launch files of `dingo_base` and `dingo_bringup` packages that are installed in `/opt/ros/noetic/share/`. These launch files are automatically launched during the boot of the robot.

The goal is to modify the automatic roslaunch files to do the following:

* Add namespacing to prevent naming conflicts
* Add e-stop functionality
* Add scaling for forward and inverse kinematics

Following https://www.clearpathrobotics.com/assets/guides/kinetic/ridgeback/startup.html

For namespaces such as `/d1,/d2,/d3,/d4`, add `<group ns="NAMESPACE"> ... </group>` into the launch files. For example, for dingo robot `d1`, edit the launch files as follows:

```html
<launch>
    <group ns="d1">
        ...
        ... ORIGINAL CONTENT OF THE 'base.launch file' or 'accessories.launch' file.
        ...
    </group>
<launch>
```

It's important to note that, in the original `dingo_base` package, the TF names of the wheel links are not parameterized and therefore causes a conflict when using different `tf prefix`es for each Dingo robot. To resolve this issue, we edited `dingo_hardware.cpp` and `dingo_hardware.h` files so that they parameterize the TF names of the wheel links. Therefore when we launch the `dingo_base` package in `base.launch` file, we follow this kind of prefixing to each wheel link:

```html
<node pkg="dingo_base" type="dingo_node" name="dingo_node" output="screen">
    <param name="dingo_omni" type="bool" value="$(optenv DINGO_OMNI 0)" />
    <rosparam command="load" file="$(find dingo_base)/config/$(arg motor_params).yaml" />
    <param name="wireless_interface" value="$(optenv DINGO_WIRELESS_INTERFACE wlp2s0)" />

    <param name="front_left_wheel"  value="$(arg tf_prefix)front_left_wheel" />
    <param name="front_right_wheel" value="$(arg tf_prefix)front_right_wheel" />
    <param name="rear_left_wheel"   value="$(arg tf_prefix)rear_left_wheel" />
    <param name="rear_right_wheel"  value="$(arg tf_prefix)rear_right_wheel" />
</node>
```

This edited version of the `dingo_base` package is given along with the other packages of this repository. (TODO: Fork `dingo_base` and properly mananage these edits.)

</details>

### Needed Edits for `/etc/ros/setup.bash`

<details>

* Add `export DINGO_OMNI=1` if it does not setup as `1`.
* Comment out or edit the line `source /home/administrator/catkin_ws/devel/setup.bash` such that it points to the name of our workspace `catkin_ws_assistive` as `source /home/administrator/catkin_ws_assistive/devel/setup.bash`.
* Comment out the line `source /etc/clearpath-dingo.bash`or make sure it does not conflict with the `export ...` lines such as `export DINGO_OMNI=1` specified in this `/etc/ros/setup.bash` file.
* Comment out the line `export DINGO_CONFIG_EXTRAS=$(catkin_find rpi06_dingo config/localization.yaml --first-only)` if it exists.
* Make sure the lines related to LIDAR, IMU and the UWB sensors are included in this file. 
* For the final version of `/etc/ros/setup.bash` file, see `/src/dingo_customization/d1/scripts/setup.bash`. We basically replace the default file with this custom file.

</details>

</details>