#!/usr/bin/env python  

import rospy

import numpy as np
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg 
# from geometry_msgs.msg import PoseStamped, TransformStamped    

import kinova_msgs.msg

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class SimpleCollabMove():
    def __init__(self):
        
        # init node
        rospy.init_node('collaborative_move_simple', anonymous=True)

        # Topic name to publish
        self.robot_cartesian_cmd_vel_topic_name = rospy.get_param("~robot_cartesian_cmd_vel_topic_name", "j2n6s300_driver/in/cartesian_velocity")
        # Publishgin msg type (either Geometry_msgs::Twist or kinova_msgs::PoseVelocity)
        self.robot_cartesian_cmd_vel_msg_type = rospy.get_param("~robot_cartesian_cmd_vel_msg_type", "geometry_msgs.msg.Twist")

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0) # 100Hz needed for kinova arm
        self.rate = rospy.Rate(self.pub_rate)
        self.expected_duration = 1.00/self.pub_rate # seconds per cycle

        # Publisher
        if self.robot_cartesian_cmd_vel_msg_type == "geometry_msgs.msg.Twist":
            self.pub_pose_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, geometry_msgs.msg.Twist, queue_size=1)
        elif self.robot_cartesian_cmd_vel_msg_type == "kinova_msgs.msg.PoseVelocity":
            self.pub_pose_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)

        # create collaborative move toggle service
        self.enable_collab_move = rospy.get_param("~enable_collab_move", False)
        self.toggle_collab_move_service_name = rospy.get_param("~toggle_collab_move_service_name")
        # Service to toggle the collaborative moving (enable/disable)
        self.srv_toggle_collab_move = rospy.Service(self.toggle_collab_move_service_name, SetBool, self.srv_toggle_collab_move_cb)

        # tf definitions
        # Specified end effector tf frame name (id)
        self.tf_end_effector_frame_name = rospy.get_param("~tf_end_effector_frame_name", "j2n6s300_end_effector")
        # Specified arm base tf frame name 
        self.tf_robot_base_frame_name = rospy.get_param("~tf_robot_base_frame_name", "oarbot_blue_base")
        # Specified arm base tf frame name 
        self.tf_arm_base_frame_name = rospy.get_param("~tf_arm_base_frame_name", "j2n6s300_link_base")
        # Specified target end effector tf frame name
        self.tf_target_end_effector_frame_name = rospy.get_param("~tf_target_end_effector_frame_name", "target_end_effector")        

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # TF broadcaster (for visualization purposes)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # start control loop
        rospy.Timer(rospy.Duration(self.expected_duration), self.collab_move_loop)

    def collab_move_loop(self, event=None):
        while not rospy.is_shutdown():
            start_time = rospy.get_rostime()
        
    
    def srv_toggle_collab_move_cb(self,req):
        assert isinstance(req, SetBoolRequest)

        if req.data:
            self.enable_collab_move = True
            rospy.loginfo("Enable simple collaborative move")   

        else:
            self.enable_collab_move = False
            rospy.loginfo("Disable admittance control")

        return SetBoolResponse(True, "The simple collaborate movement is now set to: {}".format(self.enable_collab_move))

if __name__ == '__main__':
    collab_move = SimpleCollabMove()
    rospy.spin()