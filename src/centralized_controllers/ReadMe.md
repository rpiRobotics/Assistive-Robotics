# centralized_controllers (Package)

## auto_disable_ekf_localization (Node)

A simple python node, which utilizes the `toggleFilterProcessing`* service of the sensor fusion ([robot_localization](https://github.com/cra-ros-pkg/robot_localization) package ekf) node.  The node will listen for the body joint pose topics coming from the tf body joint broadcaster. Each time a body pose arrives it updates the last time.**
In a separate thread check for how much time has passed since the last message arrival time. If the time passed is more than a timeout (specified as a parameter by the user) call the toggle Filter Processing Node to pause the filter processing while still publishing the last pose. Keep the state of the filter in this node's scope with a private boolean parameter. If the pose messages start coming back, and if we had disabled the filter processing, request to enable it back with the toggle service again. For both toggling operations make sure the request is successful. otherwise do not change the private boolean parameter that holds the filter state and try again in the next iteration of the timer. 

![AutoDisabler](https://github.com/rpiRobotics/Assistive-Robotics/assets/34518156/77c68af7-a6eb-408e-82d2-8595c4983df2)


*`ToggleFilterProcessingSrv` service allows another node to toggle on/off filter processing while still publishing and Uses a robot_localization package's custom ToggleFilterProcessing service. ([see here for more](https://docs.ros.org/en/noetic/api/robot_localization/html/api/classRobotLocalization_1_1RosFilter.html#ad87227f3976d25577258b1c2cbf8c17a))

**To make the node more general, we will make this node to automatically find all the
`nav_msgs/Odometry`
`geometry_msgs/PoseWithCovarianceStamped`
`geometry_msgs/TwistWithCovarianceStamped`
`sensor_msgs/Imu`
topics that the localization node subscribes to. These are the ROS message types that the `robot_localization` package supports as input ([source](https://docs.ros.org/en/noetic/api/robot_localization/html/preparing_sensor_data.html#adherence-to-ros-standards)). We will create a subscriber for each of them as well and within each subsriber we will simply just update the last arrival time. That's all we need.

Usage of the ToggleFilterProcessingSrv
From a terminal this  DISABLES the processing:
```
rosservice call /human_localization_ekf_se/toggle "'on': false" 
and returns
status: True
```
This ENABLES the processing:
```
rosservice call /human_localization_ekf_se/toggle "'on': false" 
and again returns
status: True
```
if the request is successful.

The definition file of the service has title: `ToggleFilterProcessing.srv`
with the following content:
```
bool on
---
bool status
```
