#!/usr/bin/env python

"""
Author: Chen-Lung Eric Lu, Burak Aksoy
Node: oarbot_redundancy_resolver
Description:
    This node takes a velocity twist command for the end effector of oarbot (arm+base = 10DoF) and resolves
    the redundancy for moving the arm and the base with some sort of controller/planner 
    to weight which parts to move.
    
    In other words, the node takes in desired end-effector (angular and linear) 
    velocity and split it to robot arm/support height/mobile base velocity
    based on designed rule which can be extended. 
Parameters:
    - TODO
Subscribes to:
    - /$(robot)/cmd_vel (geometry_msgs::Twist)
    - /joint_states (TODO) (eg. /oarbot_silver/j2n6s300_left_driver/out/joint_state)
Publishes to:
    - /$(robot)/cmd_vel_arm for (/j2n6s300_driver/in/cartesian_velocity (kinova_msgs::CartesianVelocity))
    - /$(robot)/cmd_vel_base (geometry_msgs::Twist)
Broadcasts to:
    - 
"""

import rospy

import geometry_msgs.msg # for Twist
import kinova_msgs.msg  # for PoseVelocity ,JointVelocit
import sensor_msgs.msg # for JointState

import std_msgs.msg # for Float64 to show constrained r (? TODO) value 
import visualization_msgs.msg # for Marker to show control constraint ball marker on rviz for debug purposes

import numpy as np
import math


class OarbotRedundancyResolver():
    def __init__(self):
        rospy.init_node('oarbot_redundancy_resolver', anonymous=True)

        # Published topic names 
        self.cmd_vel_arm_topic_name = rospy.get_param("~cmd_vel_arm_topic_name", "j2n6s300_driver/in/cartesian_velocity")
        self.cmd_vel_base_topic_name = rospy.get_param("~cmd_vel_base_topic_name", "/cmd_vel_base")
        
        # Subscribed topic names
        self.cmd_vel_topic_name = rospy.get_param("~cmd_vel_topic_name", "/cmd_vel")
        # TODO
        self.joint_states_arm_topic_name = rospy.get_param("~joint_states_arm_topic_name", "/arm_joint_states")
        self.joint_states_base_topic_name = rospy.get_param("~joint_states_base_topic_name", "/base_joint_states")

        # Control law ball parameters wrt arm root 
        self.control_ball_center_xyz = rospy.get_param("~control_ball_center", [0.5, 0.0, 0.3])
        self.control_ball_center_xyz = np.array(self.control_ball_center_xyz) # (3,)
        self.control_ball_radius = rospy.get_param("~control_ball_radius", 0.33)

        self.control_upper_xyz = rospy.get_param("~control_upper_xyz", [0.6, 0.3, 0.5])
        self.control_upper_xyz = np.array(self.control_upper_xyz) # (3,)

        self.control_lower_xyz = rospy.get_param("~control_lower_xyz", [0.25, -0.3, -0.1])
        self.control_lower_xyz = np.array(self.control_lower_xyz) # (3,)

        # Some needed TF frame names
        self.tf_mobile_base_frame_id = rospy.get_param("~tf_mobile_base_frame_id", "oarbot_blue_base")
        self.tf_arm_base_frame_id = rospy.get_param("~tf_arm_base_frame_id", "root_right_arm")

        self.arm_joints_tf_prefix = rospy.get_param("~arm_joints_tf_prefix", 'j2n6s300_')

        # Variables
        # TODO
        self.joint_state_arm = None
        self.joint_state_base = None # TODO
        # self.joint_base_start = None
        # self.joint_sup_start = None
        # self.joint_arm_start = None
        self.joint_finger_start_index = None
        # Create oarbot toolbox from robotics toolbox
        # self.bot = Oarbot() # TODO


        # Publishers
        self.pub_cmd_vel_base = rospy.Publisher(self.cmd_vel_base_topic_name, geometry_msgs.msg.Twist, queue_size=1)
        self.pub_cmd_vel_arm = rospy.Publisher(self.cmd_vel_arm_topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)
        # self.pub_cmd_joint_vel_arm = rospy.Publisher(self.cmd_vel_arm_topic_name, kinova_msgs.msg.JointVelocity, queue_size=1)

        # Debug publishers
        self.pub_constrained_markers = rospy.Publisher("constraint_marker", visualization_msgs.msg.Marker, queue_size=1)
        self.pub_constrained_r = rospy.Publisher("constrained_r", std_msgs.msg.Float64, queue_size=1)


        # Subscribers
        self.sub_cmd_vel = rospy.Subscriber(self.cmd_vel_topic_name, 
                                            geometry_msgs.msg.Twist, 
                                            self.split_vel_callback, 
                                            queue_size=1)

        self.sub_joint_states_arm = rospy.Subscriber(self.joint_states_arm_topic_name, 
                                                 sensor_msgs.msg.JointState, 
                                                 self.joint_states_arm_callback, 
                                                 queue_size=1)

        self.sub_joint_states_base = rospy.Subscriber(self.joint_states_base_topic_name, 
                                                 sensor_msgs.msg.JointState, 
                                                 self.joint_states_base_callback, 
                                                 queue_size=1)

    def split_velocity_callback(self, msg):
        # Convert desired vel command as in robotics course 6x1 vector with [w,v]^T
        des_cmd = np.array([msg.angular.x,msg.angular.y,msg.angular.z,msg.linear.x,msg.linear.y,msg.linear.z])
        des_cmd = np.reshape(des_cmd,(6,1))

        # 



        # TODO


    def joint_states_arm_callback(self, msg):
        # TODO
        # Figure out at which index in the joint state msg the finger joints at the arm starts
        if self.joint_arm_state is None:
            self.joint_finger_start = msg.name.index(self.arm_joints_tf_prefix + "joint_finger_1")
            # If multiple indexes are going to be figured out, comment out above, comment in below, then add more elif conditions
            # for i in range(len(msg.name)):
            #     if msg.name[i] == self.arm_joints_tf_prefix + "joint_finger_1":
            #         self.joint_finger_start = i

        # Store the joint state of the arm
        self.joint_state_arm = msg
        # Publish the constraint ball for debug purposes, it uses current arm base pose
        self.publish_constraint_ball()


    def joint_states_base_callback(self, msg):
        # TODO
        pass


    def publish_cmd_vel_base(self, vx,vy,vz, wz):
        # example usage: 
        # self.publish_cmd_vel_base(vx,vy,vz, wz)

        # Generate and publish the Twist message
        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz
        
        # Scaling
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = wz

        self.pub_cmd_vel_base.publish(cmd_vel)

    def publish_cmd_vel_arm(self, vx,vy,vz,wx,wy,wz):
        # example usage: 
        # self.publish_cmd_vel_arm(vx,vy,vz, wx,wy,wz)

        # Generate and publish the Twist message
        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz
        
        # Scaling
        cmd_vel.angular.x = wx
        cmd_vel.angular.y = wy
        cmd_vel.angular.z = wz

        self.pub_cmd_vel_arm.publish(cmd_vel)

    def publish_constraint_ball(self):
        m = visualization_msgs.msg.Marker()
        m.header.frame_id = self.tf_arm_base_frame_id
        m.header.stamp = rospy.Time.now()
        
        m.type = visualization_msgs.msg.Marker.SPHERE
        
        m.pose.position.x = self.control_ball_center_xyz[0]
        m.pose.position.y = self.control_ball_center_xyz[1]
        m.pose.position.z = self.control_ball_center_xyz[2]
        m.pose.orientation.w = 1
        
        m.scale.x = self.control_ball_radius*2
        m.scale.y = self.control_ball_radius*2
        m.scale.z = self.control_ball_radius*2

        m.color.a = 0.2
        m.color.r = 1
        m.color.g = 1

        self.pub_constraint_markers.publish(m)


if __name__ == "__main__":
    oarbotRedundancyResolver = OarbotRedundancyResolver()
    rospy.spin()
