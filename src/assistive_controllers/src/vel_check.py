#!/usr/bin/env python3  

"""
Author: Chen-Lung Eric Lu
Node: vel_check
Description:
    This node check if the function of vel_splitter node is correct.
Parameters:
    - 
Subscribes to:
    - tf2
Publishes to:
    - /observed_vel (geometry_msgs::Twist)
Broadcasts to:
    - 
"""
import rospy
import tf2_ros
import numpy as np
import general_robotics_toolbox as rox
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

from kinova_msgs.msg import PoseVelocity,JointVelocity

from oarbot_moveit.oarbot_moveit import Oarbot
from copy import deepcopy as dp
import general_robotics_toolbox as rox
from math import sin,cos,pi

import time

class VelCheck():
    def __init__(self):
        rospy.init_node('vel_splitter', anonymous=True)
        self.is_following_started = False

        # Topic name to publish
        self.obs_vel_topic_name = rospy.get_param("~base_cartesian_cmd_vel_topic_name", "obs_vel")

        # Publisher
        self.pub_obs_vel = rospy.Publisher(self.obs_vel_topic_name, Twist,queue_size=1)

        # Specified end effector tf frame name
        self.tf_ee_frame = rospy.get_param("~tf_end_effector_frame_name", "j2n6s300_end_effector") 

        # Specified support (arm base) tf frame name
        # sup and arm base share the same frame
        self.tf_sup_frame = rospy.get_param("~tf_arm_base_frame_name", "j2n6s300_link_base")

        # Specified world tf frame name 
        self.tf_world_frame = rospy.get_param("~tf_world_frame_name", "world")

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Last ee to world transform
        self.last_ee_world = None
        # Last sup to world transform
        self.last_sup_world = None
        # last stamp
        self.last_stamp = None

        # Start vel check
        self.pub_rate = 100
        rospy.Timer(rospy.Duration(1.00/self.pub_rate), self.tf_cb)

    def tf_cb(self, msg):

        this_stamp =  rospy.Time()
        try:
            # returns type geometry_msgs.msg.TransformStamped
            T_ee2world = self.tfBuffer.lookup_transform(self.tf_end_effector_frame_name, self.tf_body_joint_frame_name, this_stamp) # in ee frame 
            T_sup2world = self.tfBuffer.lookup_transform(self.tf_arm_base_frame_name, self.tf_end_effector_frame_name,  this_stamp) # in base frame

            eeworld_q,eeworld_p = self.tf2trans(T_ee2world)
            this_T_ee2world = rox.Transform(rox.q2R(eeworld_q),eeworld_p)
            supworld_q,supworld_p = self.tf2trans(T_sup2world)
            this_T_sup2world = rox.Transform(rox.q2R(supworld_q),supworld_p)

            if self.last_stamp is None:
                self.last_ee_world = this_T_ee2world
                self.last_sup_world = this_T_sup2world
                self.last_stamp = this_stamp
                return
            
            d_ee2world = this_T_ee2world*self.last_ee_world
            # d_ee2world_sup = self.last_sup_world.inv()*d_ee2world
            # d_ee2world_ee = self.last_ee_world.inv()*d_ee2world

            k,dtheta = rox.R2rot(d_ee2world.R)
            omega_world = k*dtheta/(this_stamp.to_sec()-self.last_stamp.to_sec()) # omega = k*dot_theta
            omega_ee = self.last_ee_world.inv()*omega_world

            v_world = d_ee2world.p/(this_stamp.to_sec()-self.last_stamp.to_sec()) # v = dot_p
            v_sup = self.last_sup_world.inv()*v_world

            obs_vel_msg = Twist()
            obs_vel_msg.angular.x = omega_ee[0]
            obs_vel_msg.angular.y = omega_ee[1]
            obs_vel_msg.angular.z = omega_ee[2]
            obs_vel_msg.linear.x = v_sup[0]
            obs_vel_msg.linear.y = v_sup[1]
            obs_vel_msg.linear.z = v_sup[2]

            self.pub_obs_vel.publish(obs_vel_msg)

            self.last_ee_world = this_T_ee2world
            self.last_sup_world = this_T_sup2world
            self.last_stamp = this_stamp
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            rospy.logwarn('Waiting to find the transformation from %s to %s, OR transformation from %s to %s' 
                            % (self.tf_ee_frame, self.tf_world_frame, 
                            self.tf_sup_frame, self.tf_world_frame))
    
    def tf2trans(self,trans):
        
        q = [trans.transform.rotation.w,trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z]
        p = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
        return q,p

if __name__ == '__main__':
    velcheck = VelCheck()
    rospy.spin()