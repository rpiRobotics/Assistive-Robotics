#!/usr/bin/env python

"""
Author: Chen-Lung Eric Lu
Node: vel_splitter
Description:
    This is an extension of body_single_joint_follower (from Burak Aksoy)
    from moving only the arm to moving the arm and the base (redundant joints)
    with some sort of controller/planner to weight which parts to move.
    
    The node takes in desired end-effector (angular and linear) velocity 
    and split it to robot arm/support height/mobile base velocity
    based on designed rule which can be extended. 
Parameters:
    - 
Subscribes to:
    - /$(robot)/desired_vel (geometry_msgs::Twist)
    - /joint_states
Publishes to:
    - /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::PoseVelocity)
    - /$(robot)/sup_vel (geometry_msgs::Twist)
    - /$(robot)/cmd_vel (geometry_msgs::Twist)
Broadcasts to:
    - 
"""

import rospy

import numpy as np
import general_robotics_toolbox as rox
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

from kinova_msgs.msg import PoseVelocity,JointVelocity

from oarbot_moveit import Oarbot
from copy import deepcopy as dp
import general_robotics_toolbox as rox
from qpsolvers import solve_qp
from math import log, sin,cos,pi

import time

class VelSplit():
    def __init__(self):
        rospy.init_node('vel_splitter', anonymous=True)
        self.is_following_started = False

        # Topic name to subscribe
        self.desired_cartesian_cmd_vel_topic_name = rospy.get_param("~desired_cartesian_cmd_vel_topic_name", "desired_vel")
        self.joint_states_topic_name = rospy.get_param("~joint_states_topic_name", "/joint_states")

        # Topic name to publish
        self.base_cmd_vel_topic_name = rospy.get_param("~base_cartesian_cmd_vel_topic_name", "cmd_vel")
        self.sup_cmd_vel_topic_name = rospy.get_param("~sup_cartesian_cmd_vel_topic_name", "sup_vel")
        self.robot_cartesian_cmd_vel_topic_name = rospy.get_param("~robot_cartesian_cmd_vel_topic_name", "arm_cartesian_vel")
        self.robot_joint_cmd_vel_topic_name = rospy.get_param("~robot_cartesian_cmd_vel_topic_name", "joint_vel")

        # Variables
        self.joint_state = None # joint state
        self.joint_base_start = None
        self.joint_sup_start = None
        self.joint_arm_start = None
        self.joint_finger_start = None
        self.bot = Oarbot()

        # Publisher
        self.pub_base_vel_cmd = rospy.Publisher(self.base_cmd_vel_topic_name, Twist,queue_size=1)
        self.pub_sup_vel_cmd = rospy.Publisher(self.sup_cmd_vel_topic_name, Float64,queue_size=1)
        self.pub_robot_cart_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, Twist, queue_size=1)
        # self.pub_robot_cart_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, PoseVelocity, queue_size=1)
        self.pub_robot_joint_vel_cmd = rospy.Publisher(self.robot_joint_cmd_vel_topic_name, JointVelocity, queue_size=1)

        self.pub_constraint_markers = rospy.Publisher("constraint_marker",Marker,queue_size=1)
        self.pub_constrained_r = rospy.Publisher("constrained_r",Float64,queue_size=1)

        # control law param
        self.control_center = np.array([0.5,0,0.3])
        self.control_r = 0.33

        # control law param
        self.control_x_upper = 0.6
        self.control_x_lower = 0.25
        self.control_y_upper = 0.3
        self.control_y_lower = -0.3
        self.control_z_upper = 0.50
        self.control_z_lower = -0.1
        self.control_upper = np.array([self.control_x_upper,self.control_y_upper,self.control_z_upper])
        self.control_lower = np.array([self.control_x_lower,self.control_y_lower,self.control_z_lower])

        # Subscriber
        self.joint_sub = rospy.Subscriber(self.joint_states_topic_name, JointState, self.joint_state_cb, queue_size=1)

        # Start control (subscriber)
        self.sub_desired_vel = rospy.Subscriber(self.desired_cartesian_cmd_vel_topic_name, Twist, self.splitVel_cb, queue_size=1)

    def joint_state_cb(self, msg):

        if self.joint_arm_start is None:
            for i in range(len(msg.name)):
                if msg.name[i] == 'j2n6s300_joint_1':
                    self.joint_arm_start = i
                elif msg.name[i] == 'connect_base_and_world_x':
                    self.joint_base_start = i
                elif msg.name[i] == 'base_to_support':
                    self.joint_sup_start = i
                elif msg.name[i] == 'j2n6s300_joint_finger_1':
                    self.joint_finger_start = i
        
        self.joint_state = msg

        # m = Marker()
        # m.header.frame_id = 'j2n6s300_link_base'
        # m.header.stamp = rospy.Time.now()
        # m.type = Marker.CUBE
        # m.pose.position.x = (self.control_x_upper+self.control_x_lower)/2
        # m.pose.position.y = (self.control_y_upper+self.control_y_lower)/2
        # m.pose.position.z = (self.control_z_upper+self.control_z_lower)/2
        # m.pose.orientation.w = 1
        # m.scale.x = self.control_x_upper-self.control_x_lower
        # m.scale.y = self.control_y_upper-self.control_y_lower
        # m.scale.z = self.control_z_upper-self.control_z_lower
        # m.color.a = 0.2
        # m.color.r = 1
        # m.color.g = 1

        m = Marker()
        m.header.frame_id = 'j2n6s300_link_base'
        m.header.stamp = rospy.Time.now()
        m.type = Marker.SPHERE
        m.pose.position.x = self.control_center[0]
        m.pose.position.y = self.control_center[1]
        m.pose.position.z = self.control_center[2]
        m.pose.orientation.w = 1
        m.scale.x = self.control_r*2
        m.scale.y = self.control_r*2
        m.scale.z = self.control_r*2
        m.color.a = 0.2
        m.color.r = 1
        m.color.g = 1

        self.pub_constraint_markers.publish(m)
                    

    def splitVel_cb(self, cmd_msg):

        if self.joint_state is None:
            return

        des_cmd = np.array([cmd_msg.angular.x,cmd_msg.angular.y,cmd_msg.angular.z,cmd_msg.linear.x,cmd_msg.linear.y,cmd_msg.linear.z])
        des_cmd = np.reshape(des_cmd,(6,1))

        joint_array = np.asfarray(self.joint_state.position)
        joint_array[self.joint_arm_start:self.joint_arm_start+6] -= self.bot.q_zeros_arm
        arm_cmd, sup_cmd, base_cmd = self.splitLaw(des_cmd,joint_array)

        arm_msg = Twist()
        arm_msg.angular.x = arm_cmd[0]
        arm_msg.angular.y = arm_cmd[1]
        arm_msg.angular.z = arm_cmd[2]
        arm_msg.linear.x = arm_cmd[3]
        arm_msg.linear.y = arm_cmd[4]
        arm_msg.linear.z = arm_cmd[5]

        sup_msg = Float64()
        sup_msg.data = sup_cmd

        base_msg = Twist()
        base_msg.angular.z = base_cmd[2]
        base_msg.linear.x = base_cmd[3]
        base_msg.linear.y = base_cmd[4]

        self.pub_robot_cart_vel_cmd.publish(arm_msg)
        self.pub_sup_vel_cmd.publish(sup_msg)
        self.pub_base_vel_cmd.publish(base_msg)

    def splitLaw(self, des_cmd, joint_array):
        """
        Input: 
            des_cmd: [angular_x,angular_y,angular_z,linear_x,linear_y,linear_z] (vd)
        Output:
            arm_cmd: Robot arm cartesian cmd (in the arm base frame)
            [linear_x,linear_y,linear_z,angular_x,angular_y,angular_z] (np.array)
            sup_cmd: Support height velocity (in the base frame. Note: it's 1DOF)
            linear_z (float)
            base_cmd: Mobile base velocity (in the base frame)
            [linear_x,linear_y,angular_z] (np.array)
        """
        st = time.perf_counter_ns()

        # ||JA qadot + JB qbdot - vd||^2 + qadot^T W_a qadot + qbdot^T W_b qbdot

        q = np.append(joint_array[self.joint_base_start:self.joint_base_start+3],joint_array[self.joint_sup_start])
        q = np.append(q,joint_array[self.joint_arm_start:self.joint_arm_start+6])

        J = self.bot.jacobian(q)
        J_arm = self.bot.arm_jacobian(q[4:])

        # u,s,v = np.linalg.svd(J_arm)
        # print(s)

        R_sup2base = np.transpose(rox.rot([0,0,1],q[2]))
        Jee_sup = np.dot(R_sup2base,J[:3,:])
        Jee_sup = np.vstack((Jee_sup,np.dot(R_sup2base,J[3:,:])))

        T_arm2ee = self.bot.fwdkin_arm(q[4:])

        nu_omega = np.dot(T_arm2ee.R,des_cmd[:3])
        nu = np.append(nu_omega,des_cmd[3:])
        # print(nu)

        # Kq=.001*np.eye(len(q))    #small value to make sure positive definite
        constrained_r = np.linalg.norm(T_arm2ee.p-self.control_center)
        arm_w, base_w = self.weighting(T_arm2ee.p,nu,constrained_r)

        # testing
        # arm_w = 10
        # base_w = 0.1

        # the more the weight, the less it's used
        Wa = np.ones(6)*arm_w # weighting for arm axis velocity
        Wb = np.ones(len(q)-6)*base_w # weighting for base axis velocity
        Wba = np.diag(np.append(Wb,Wa))

        H = np.dot(np.transpose(Jee_sup),Jee_sup)+Wba
        H = (H+np.transpose(H))/2

        f = -np.dot(np.transpose(Jee_sup),nu).reshape((len(q),))
        # print("f",f)
        # print("H",H)

        qdot = solve_qp(H,f)
        # print("qdot",qdot)
        # print("nu res",np.dot(Jee_sup,qdot))
        # print("=================")

        arm_cmd = np.dot(J_arm,qdot[4:])
        arm_cmd[:3] = np.dot(np.transpose(T_arm2ee.R),arm_cmd[:3])

        sup_cmd = qdot[3]

        Rbo = np.array([[cos(q[2]),sin(q[2])],[-sin(q[2]),cos(q[2])]])
        qdotbase_xy = np.dot(Rbo,qdot[:2])
        base_cmd = np.array([0,0,qdot[2],qdotbase_xy[0],qdotbase_xy[1],0])
        # base_cmd = np.array([0,0,qdot[2],qdot[0],qdot[1],0])

        et = time.perf_counter_ns()
        # print("duration:",(et-st)*1e-9)

        constrained_r_msg = Float64()
        constrained_r_msg.data = constrained_r
        self.pub_constrained_r.publish(constrained_r_msg)

        return arm_cmd,sup_cmd,base_cmd
    
    def weighting(self, p, nu, r):

        if r > self.control_r:
            r = self.control_r-0.01
        
        next_r = np.linalg.norm(p+0.01*nu[3:]-self.control_center)
        if next_r < r:
            wr = 3
        else:
            a = 3-log(self.control_r,2)
            wr = log(-1*r+self.control_r,2)+a

        if wr >= 0:
            wa = 0.01
            wb = wa*(10**wr)
        else:
            wb = 0.01
            wa = wb/(10**wr)

        return wa,wb

    def splitLaw_dis(self, des_cmd, joint_array):
        """
        Input: 
            des_cmd: [angular_x,angular_y,angular_z,linear_x,linear_y,linear_z]
        Output:
            arm_cmd: Robot arm cartesian cmd (in the arm base frame)
            [angular_x,angular_y,angular_z,linear_x,linear_y,linear_z] (np.array)
            sup_cmd: Support height velocity (in the base frame. Note: it's 1DOF)
            linear_z (float)
            base_cmd: Mobile base velocity (in the base frame)
            [angular_x,angular_y,angular_z,linear_x,linear_y,linear_z] (np.array)
        """

        ee_arm = self.bot.fwdkin_arm(joint_array[self.joint_arm_start:self.joint_arm_start+6])

        if np.prod(ee_arm.p >= self.control_lower) and np.prod(ee_arm.p <= self.control_upper):
            arm_move = True
            base_move = False
        else:
            arm_move = False
            base_move = True

        q = dp(joint_array[self.joint_base_start:self.joint_base_start+3])
        q = np.append(q,joint_array[self.joint_sup_start])
        q = np.append(q,joint_array[self.joint_arm_start:self.joint_arm_start+6])

        Rb_ab = rox.rot(self.bot.bot.H[:,2],q[2]) # Rb_ab = Rotation of third joint (the first two are prismatic joints)
        # frame_rot = np.eye(6)
        # frame_rot[0:3,0:3] = Rb_ab.T
        # frame_rot[3:6,3:6] = Rb_ab.T

        # base move 
        ee_arm_cross = rox.hat(ee_arm.p)
        phi_ee_sup_inv = np.eye(6)
        phi_ee_sup_inv[3:6,0:3] = ee_arm_cross
        # nu_sup_sup = np.matmul(phi_ee_sup_inv,np.matmul(frame_rot,des_cmd))
        nu_sup_sup = np.matmul(phi_ee_sup_inv,des_cmd)
        sup_cmd = dp(nu_sup_sup[5])

        nu_sup_sup[5] = 0
        p_car_sup_cross = rox.hat(self.bot.bot.P[:,3]+q[3])
        phi_sup_car_inv = np.eye(6)
        phi_sup_car_inv[3:6,0:3] = p_car_sup_cross
        base_cmd = np.matmul(phi_sup_car_inv,nu_sup_sup)
        
        # arm move
        # arm_cmd = np.matmul(frame_rot,des_cmd)
        arm_cmd = des_cmd
        
        # x-direction separation
        if ((ee_arm.p[0] > self.control_x_upper) and des_cmd[3]>0) or ((ee_arm.p[0] < self.control_x_lower) and des_cmd[3]<0):
            arm_cmd[3]=0
        else:
            base_cmd[3]=0
        
        # y-direction separation
        if ((ee_arm.p[1] > self.control_y_upper) and des_cmd[4]>0) or ((ee_arm.p[1] < self.control_y_lower) and des_cmd[4]<0):
            arm_cmd[4]=0
        else:
            base_cmd[4]=0
        
        # z-direction separation
        # print(ee_arm.p)
        if ((ee_arm.p[2] > self.control_z_upper) and des_cmd[5]>0) or ((ee_arm.p[2] < self.control_z_lower) and des_cmd[5]<0):
            arm_cmd[5]=0
        else:
            base_cmd[5]=0
            sup_cmd=0

        # let the angular velocity of the base equals 0
        base_cmd[0:3] = np.zeros((3,1))

        return arm_cmd,sup_cmd,base_cmd


if __name__ == '__main__':
    velsplit = VelSplit()
    rospy.spin()