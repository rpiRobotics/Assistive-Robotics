
Source: [https://msadowski.github.io/linux-static-port/](https://msadowski.github.io/linux-static-port/)

Assign a static USB port on Linux
=================================

Quite often I found myself needing static device names in Linux, especially when using _ROS_. As an example imagine having two sensors of a same type that yo connect to your PC. Now, if you don’t map your ports in any way you might end up with devices being mapped to _/dev/ttyACM0_ and _/dev/ttyACM1_, depending which one was connected first.

Let’s say your sensors are at different positions on the robot. If you want to make sure that you are using the proper device in your code you can either:

1.  Unplug and plug back the sensors again until you get the names you want
2.  Assign a static name to a port and be done with it

If you want to learn about the option 2 then read on.

_Otherwise keep playing with your cables!_

Follow the .rules
=================

Below I will show you the steps to create a symbolic link for the USB device so that you don’t have to replug anything ever again to fix the order of the devices.

*   List USB attributes for the device:

`udevadm info --name=/dev/ttyACMx --attribute-walk`

Where _x_ is a number matching your device port. In the attribute list you should look for an attribute that is unique for a device (check _idVendor_ and _idProduct_).

*   Create (or open) a file `_/etc/udev/rules.d/99-usb-serial.rules_`
*   If you found an unique attribute of the device then write a following line in the rules file:

`KERNEL=="ttyACM*", ATTRS{idVendor}=="6472", SYMLINK+="sensor_0"`

If there is nothing unique in the two of your sensors, you can do static assignment based on the USB kernel (so basically whatever device you connect to a given port it will create a symbolic link with a same name). It works well even with USB hubs!

`KERNEL=="ttyACM*", KERNELS=="1-4:1.0", SYMLINK+="sensor_0"`

*   Reload your udevadm rules:

`udevadm control --reload-rules`

Now, if everything went correctly you can connect your device and type in your terminal:

`ls /dev/sens*`

to see if you successfully created a symlink. Since it’s a symbolic link you will be able to access both _/dev/sensor\_0_ and _/dev/ttyACM0_ but they will point to the same device.

That’s it, now (hopefully) you don’t have to reconnect your sensor every time you are running your ROS nodes.

