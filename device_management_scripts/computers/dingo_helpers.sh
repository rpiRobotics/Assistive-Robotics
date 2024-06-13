HOSTS=(
    "192.168.1.102" 
    "192.168.1.104"
    )
USERNAMES=(
    "oarbot_blue"
    "oarbot_silver"
    ) 
PASSWORDS=(
    "1234" 
    "1234"
    )
ROS_VERSION_NAMES=(
    "melodic"
    "melodic"
    )
# Specify the packages to be built in these computers
CATKIN_WHITELIST_PACKAGES=(
    "assistive_msgs"
    "assistive_gui"
    "assistive_launch"
    # "tablet_arduino_talker"
    "rokubi_kinova_adapter"
    "force_torque_tools"
    "force_torque_sensor_calib"
    "gravity_compensation"
    "tf_to_fake_imu"
    # "obstacle_detector"
    "assistive_controllers"
    "oarbot_control"
    # "vel_controller"
    # "dingo_base"
    # "dingo_customization"
    "azure_kinect_ros_driver"
    "kinova_gazebo"
    "kinova_control"
    "kinova_msgs"
    # "kinova_arm_moveit_demo"
    # "j2n6s300_moveit_config"
    "j2n6s300_ikfast"
    "kinova_driver"
    "kinova_description"
    "kinova_bringup"
    "kinova_helper"
    "kinova_demo"
    # "uwb_gazebo_plugin"
    # "uwb_pose_publisher"
    # "uwb_reader"
    # "tf2"
    "tf_broadcasters"
    "topic_tf_transformers"
    # "lab_gazebo"
    # "RVizMeshVisualizer"
    # "rviz_ortho_view_controller"
    # "rosbag_editor"
)


# export WHITELIST_PACKAGES="assistive_msgs;assistive_gui;assistive_launch;rokubi_kinova_adapter;force_torque_tools;force_torque_sensor_calib;gravity_compensation;tf_to_fake_imu;assistive_controllers;oarbot_control;azure_kinect_ros_driver;kinova_gazebo;kinova_control;kinova_msgs;j2n6s300_ikfast;kinova_driver;kinova_description;kinova_bringup;kinova_helper;kinova_demo;tf_broadcasters;topic_tf_transformers"