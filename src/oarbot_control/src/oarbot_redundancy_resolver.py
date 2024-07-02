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
    - cmd_vel_arm_topic_name
    - cmd_vel_base_topic_name
    - debug_constraint_marker_topic_name
    - debug_constrained_r_topic_name
    - cmd_vel_topic_name
    - joint_states_arm_topic_name
    - control_ball_center
    - control_ball_radius
    - control_upper_xyz
    - control_lower_xyz
    - tf_world_frame_id
    - tf_mobile_base_frame_id
    - tf_arm_base_frame_id
    - arm_joints_tf_prefix
    - mobile_base2arm_base_xy
    - is_left_arm_config
    - plate_bottom_min
    - plate_bottom_max
    - plate_bottom2arm_base
    - pub_rate_arm
    - pub_rate_base    
    - min_rate_cmd_vel
Subscribes to:
    - /$(robot)/cmd_vel (geometry_msgs::Twist)
    - /joint_states (eg. /oarbot_silver/j2n6s300_left_driver/out/joint_state)
    - tf2
Publishes to:
    - /$(robot)/cmd_vel_arm (geometry_msgs::Twist)
    - /$(robot)/cmd_vel_base (geometry_msgs::Twist)
Broadcasts to:
    - 
"""

import rospy

import tf2_ros

import geometry_msgs.msg # for Twist
import kinova_msgs.msg  # for PoseVelocity ,JointVelocit
import sensor_msgs.msg # for JointState

import std_msgs.msg # for Float64 to show constrained r value 
import visualization_msgs.msg # for Marker to show control constraint ball marker on rviz for debug purposes

from oarbot_kinematics import Oarbot
import general_robotics_toolbox as rox
from qpsolvers import solve_qp

import numpy as np
import math
import time
class OarbotRedundancyResolver():
    def __init__(self):

        rospy.init_node('oarbot_redundancy_resolver', anonymous=True)

        # Published topic names 
        self.cmd_vel_arm_topic_name = rospy.get_param("~cmd_vel_arm_topic_name", "j2n6s300_driver/in/cartesian_velocity")
        self.cmd_vel_base_topic_name = rospy.get_param("~cmd_vel_base_topic_name", "/cmd_vel_base")

        # Debug Published topic names
        self.debug_constraint_marker_topic_name = rospy.get_param("~debug_constraint_marker_topic_name", "constraint_marker")
        self.debug_constrained_r_topic_name = rospy.get_param("~debug_constrained_r_topic_name", "constrained_r")

        # Publishers
        self.pub_cmd_vel_base = rospy.Publisher(self.cmd_vel_base_topic_name, geometry_msgs.msg.Twist, queue_size=1)
        self.pub_cmd_vel_arm = rospy.Publisher(self.cmd_vel_arm_topic_name, geometry_msgs.msg.Twist, queue_size=1)

        # Debug publishers
        self.pub_constraint_markers = rospy.Publisher(self.debug_constraint_marker_topic_name, visualization_msgs.msg.Marker, queue_size=1)
        self.pub_constrained_r = rospy.Publisher(self.debug_constrained_r_topic_name, std_msgs.msg.Float64, queue_size=1)
        
        # Subscribed topic names
        self.cmd_vel_topic_name = rospy.get_param("~cmd_vel_topic_name", "/cmd_vel")

        self.joint_states_arm_topic_name = rospy.get_param("~joint_states_arm_topic_name", "/arm_joint_states")

        # Control law ball parameters wrt arm root 
        self.control_ball_center_xyz = rospy.get_param("~control_ball_center", [0.5, 0.0, 0.3])
        self.control_ball_center_xyz = np.array(self.control_ball_center_xyz) # (3,)
        self.control_ball_radius = rospy.get_param("~control_ball_radius", 0.33)

        # Some needed TF frame names
        self.tf_world_frame_id = rospy.get_param("~tf_world_frame_id", "map")
        self.tf_mobile_base_frame_id = rospy.get_param("~tf_mobile_base_frame_id", "oarbot_blue_base")
        self.tf_arm_base_frame_id = rospy.get_param("~tf_arm_base_frame_id", "root_right_arm")

        self.arm_joints_tf_prefix = rospy.get_param("~arm_joints_tf_prefix", 'j2n6s300_')

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Get zero-config transformation between arm base and the mobile base
        self.mobile_base2arm_base_xy = rospy.get_param("~mobile_base2arm_base_xy", [0.305, -0.03])
        self.is_left_arm_config = rospy.get_param("~is_left_arm_config", False)
        # Get z height limits from base
        self.plate_bottom_min = rospy.get_param("~plate_bottom_min", 0.295)
        self.plate_bottom_max = rospy.get_param("~plate_bottom_max", 0.76)
        self.plate_bottom2arm_base = rospy.get_param("~plate_bottom2arm_base", 0.061)
        # Calculate max and min heights of the arm base can be at
        self.base_z_up_limit = self.plate_bottom_max + self.plate_bottom2arm_base
        self.base_z_low_limit = self.plate_bottom_min + self.plate_bottom2arm_base
        # Create oarbot toolbox from robotics toolbox
        self.bot = Oarbot(self.mobile_base2arm_base_xy, self.is_left_arm_config,
                          self.base_z_up_limit,self.base_z_low_limit)
        # self.bot = Oarbot(self.mobile_base2arm_base_xy, False,
        #                   self.base_z_up_limit,self.base_z_low_limit)

        # Variables
        self.joint_state_arm = None
        self.joint_arm_start_index = None
        
        self.arm_cmd = None 
        self.base_cmd = None
        
        # Subscribers
        self.sub_cmd_vel = rospy.Subscriber(self.cmd_vel_topic_name, 
                                            geometry_msgs.msg.Twist, 
                                            self.split_velocity_callback, 
                                            queue_size=1)

        self.sub_joint_states_arm = rospy.Subscriber(self.joint_states_arm_topic_name, 
                                                 sensor_msgs.msg.JointState, 
                                                 self.joint_states_arm_callback, 
                                                 queue_size=1)



        # Publish rate arm
        self.pub_rate_arm = rospy.get_param("~pub_rate_arm", 100.0) # 100Hz needed for kinova arm
        # Publish rate base
        self.pub_rate_base = rospy.get_param("~pub_rate_base", 25.0) # for Oarbot base

        self.min_rate_cmd_vel = rospy.get_param("min_rate_cmd_vel",25.0) # Minimum expected rate of input cmd
        

        self.is_zero_cmd_vel_arm_sent_ever = False
        self.is_zero_cmd_vel_base_sent_ever = False

        self.velocity_command_sent_arm = True
        self.velocity_command_sent_base = True
        
        self.time_last_cmd_vel = 0.0
        self.time_last_arm_cmd = 0.0
        self.time_last_base_cmd = 0.0

        self.cmd_wait_timeout = 1.00/self.min_rate_cmd_vel

        # Start publishing
        rospy.Timer(rospy.Duration(1.00/self.pub_rate_arm), self.command_arm)
        rospy.Timer(rospy.Duration(1.00/self.pub_rate_base), self.command_base)


    def split_velocity_callback(self, msg):
        
        
        # Convert desired vel command as in robotics course 6x1 vector with [w,v]^T
        des_cmd = np.array([msg.angular.x,msg.angular.y,msg.angular.z,msg.linear.x,msg.linear.y,msg.linear.z])
        des_cmd = np.reshape(des_cmd,(6,1))

        if self.joint_state_arm is not None:
            # Start constructing 10 DoF oarbot current joint array considering Base as 4 DoF and Arm as 6 DoF
            # Construct arm joint angles from subscribed joint states
            joint_array_arm = np.asfarray(self.joint_state_arm.position) # Get arm joint angles from joint states
            joint_array_arm = joint_array_arm[self.joint_arm_start_index:self.joint_arm_start_index+6] # Discard the joints for fingers
            joint_array_arm -= self.bot.q_zeros_arm # subtract the user manual zero angles of the arm from the current joint angles

            try:
                # Construct the base joint angles from tf info
                # First get the transforms between the world and the mobile base (for x y theta) and mobile base to arm base (for z)
                # returns type geometry_msgs.msg.TransformStamped
                self.T_world2mobile_base = self.tfBuffer.lookup_transform(
                    self.tf_world_frame_id, 
                    self.tf_mobile_base_frame_id, 
                    rospy.Time()) # in world frame 
                self.T_mobile_base2arm_base = self.tfBuffer.lookup_transform(
                    self.tf_mobile_base_frame_id, 
                    self.tf_arm_base_frame_id,  
                    rospy.Time()) # in mobile base frame 

                # X and Y are trivial to obtain
                x = self.T_world2mobile_base.transform.translation.x
                y = self.T_world2mobile_base.transform.translation.y

                # Theta is not so trival, need to find the euler angles for heading angle of mobile base
                # Construct the quaternion from the tf first
                qw = self.T_world2mobile_base.transform.rotation.w # Scalar part of quaternion
                qx = self.T_world2mobile_base.transform.rotation.x
                qy = self.T_world2mobile_base.transform.rotation.y
                qz = self.T_world2mobile_base.transform.rotation.z
                # Euler XYZ
                # orientation_error = tf_conversions.transformations.euler_from_quaternion(q_orientation_error)
                roll_x, pitch_y, yaw_z = self.euler_from_quaternion(qx, qy, qz, qw) # in radians
                # Z is trivial as well, but requires to substract the min base height to find the modeled joint angle
                z =  self.T_mobile_base2arm_base.transform.translation.z - self.base_z_low_limit
                joint_array_base = np.asfarray([x, y, yaw_z, z])

                # Finally construct 10x1 joint array vector
                joint_array = np.append(joint_array_base,joint_array_arm)

                self.arm_cmd, self.base_cmd = self.splitLaw(des_cmd,joint_array)

                # Finally since the velocity command is ready to send to base and the arm, set the flags to sent
                self.time_last_cmd_vel = rospy.Time.now().to_sec()
                self.velocity_command_sent_arm = False
                self.velocity_command_sent_base = False
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Put a warning which says that the transformation could not found
                warn_msg = 'Waiting to find the transformation from %s to %s, OR transformation from %s to %s' \
                        % ( self.tf_world_frame_id, self.tf_mobile_base_frame_id, 
                            self.tf_mobile_base_frame_id, self.tf_arm_base_frame_id)
                # rospy.logwarn(warn_msg)
                rospy.logwarn_throttle(20.0, warn_msg + "(Throttled to 20.0s)")

    def splitLaw(self, des_cmd, q):
        """
        Input: 
            des_cmd: [angular_x,angular_y,angular_z,linear_x,linear_y,linear_z] (vd)
        Output:
            arm_cmd: Robot arm cartesian cmd (in the arm base frame)
            [linear_x,linear_y,linear_z,angular_x,angular_y,angular_z] (np.array)
            base_cmd: Mobile base velocity (in the base frame)
            [linear_x,linear_y,angular_z,linear_z] (np.array)
        """
        # st = time.perf_counter_ns()

        # ||JA qadot + JB qbdot - vd||^2 + qadot^T W_a qadot + qbdot^T W_b qbdot
        # rospy.logwarn("q: " + str(q))

        J_world2ee_in_world = self.bot.jacobian(q) # 6x10
        J_armbase2ee_in_armbase = self.bot.arm_jacobian(q[4:]) # 6x6

        R_mobilebase2world = rox.rot([0,0,1],q[2]).T # TODO: could have get it from TF
        # rospy.logerr("R_mobilebase2world: " + str(rox.R2q(R_mobilebase2world)))

        if self.is_left_arm_config:
            R_mobilebase2armbase = rox.rot([0,0,1],math.pi) # TODO: could have get it from TF
        else:
            R_mobilebase2armbase = rox.rot([0,0,1],0.0) # TODO: could have get it from TF
        # rospy.logerr("R_mobilebase2armbase: " + str(rox.R2q(R_mobilebase2armbase)))

        R_armbase2world = np.matmul(R_mobilebase2armbase.T, R_mobilebase2world) # TODO: could have get it from TF
        # rospy.logerr("R_armbase2world: " + str(rox.R2q(R_armbase2world)))

        # np.kron(np.eye(2),a) # Creates a block diagonal version of given matrix a repeated 2 times
        J_world2ee_in_armbase = np.matmul(np.kron(np.eye(2),R_armbase2world), J_world2ee_in_world) # 6x10

        T_armbase2ee_in_armbase = self.bot.fwdkin_arm(q[4:]) # TODO: could have get it from TF
        # rospy.logerr("P_armbase2ee_in_armbase: " + str(T_armbase2ee_in_armbase.p))
        # rospy.logerr("R_armbase2ee_in_armbase: " + str(rox.R2q(T_armbase2ee_in_armbase.R)))

        nu_omega = np.dot(T_armbase2ee_in_armbase.R, des_cmd[:3]) # Desired omega represented in arm_base (previously it was represented in ee)
        nu_v = np.dot(R_mobilebase2armbase.T, des_cmd[3:]) # Desired linear vel represented in arm_base (previously it was represented in mobile base)
        nu = np.append(nu_omega,nu_v) # 6x1 hence nu is in arm_base frame
        # print(nu)

        constrained_r = np.linalg.norm(T_armbase2ee_in_armbase.p-self.control_ball_center_xyz)
        
        # For debug, publish the constrained r value
        constrained_r_msg = std_msgs.msg.Float64()
        constrained_r_msg.data = constrained_r
        self.pub_constrained_r.publish(constrained_r_msg)

        arm_w, base_w = self.weighting(T_armbase2ee_in_armbase.p,nu,constrained_r)

        # testing
        # arm_w = 10
        # base_w = 0.1

        # the more the weight, the less it's used
        Wa = np.ones(6)*arm_w # weighting for arm axis velocity
        Wb = np.ones(len(q)-6)*base_w # weighting for base axis velocity
        Wba = np.diag(np.append(Wb,Wa)) # 10x10

        H = np.matmul(J_world2ee_in_armbase.T,J_world2ee_in_armbase) + Wba
        H = (H+H.T)/2.0

        f = -np.dot(J_world2ee_in_armbase.T,nu).reshape((len(q),))
        # print("f",f)
        # print("H",H)

        qdot = solve_qp(H,f)
        # print("qdot",qdot)
        # print("nu res",np.dot(Jee_sup,qdot))
        # print("=================")

        arm_cmd = np.dot(J_armbase2ee_in_armbase,qdot[4:])
        arm_cmd[:3] = np.dot(np.transpose(T_armbase2ee_in_armbase.R),arm_cmd[:3]) # represent omega back in the ee (it was found at armbase)

        # Rbo = np.array([[math.cos(q[2]),math.sin(q[2])],[-math.sin(q[2]),math.cos(q[2])]])
        
        qdotbase_xy = np.dot(R_mobilebase2world[:2,:2],qdot[:2])
        base_cmd = np.array([0,0,qdot[2],qdotbase_xy[0],qdotbase_xy[1],qdot[3]])
        # wx wy wz vx vy vz

        # et = time.perf_counter_ns()
        # print("duration:",(et-st)*1e-9)

        return arm_cmd,base_cmd
    
    def weighting(self, p, nu, r):

        if r > self.control_ball_radius:
            r = self.control_ball_radius-0.01
        
        next_r = np.linalg.norm(p+0.01*nu[3:]-self.control_ball_center_xyz)
        if next_r < r:
            wr = 3
        else:
            a = 3-math.log(self.control_ball_radius,2)
            wr = math.log(-1*r+self.control_ball_radius,2)+a

        if wr >= 0:
            wa = 0.01
            wb = wa*(10**wr)
        else:
            wb = 0.01
            wa = wb/(10**wr)

        return wa,wb

    def command_base(self, event=None):
        # If the velocity command for base is sent and the time's been past more than the timeout amount
        if self.velocity_command_sent_base and (rospy.Time.now().to_sec() - self.time_last_cmd_vel > self.cmd_wait_timeout):
            # send zero velocity command
            # self.publish_cmd_vel_base(0.,0.,0.,0.)

            if not self.is_zero_cmd_vel_base_sent_ever:
                rospy.logwarn_once("Zero velocities to the BASE are sent for the first time")
                self.is_zero_cmd_vel_base_sent_ever = True

        else:
            # send the latest velocity command
            vx = self.base_cmd[3]
            vy = self.base_cmd[4]
            vz = self.base_cmd[5]

            wz = self.base_cmd[2]
            self.publish_cmd_vel_base(vx,vy,vz, wz)

            self.velocity_command_sent_base = True
                
            self.is_zero_cmd_vel_base_sent_ever = False

    def command_arm(self, event=None):
        # If the velocity command for arm is sent and the time's been past more than the timeout amount
        if self.velocity_command_sent_arm and (rospy.Time.now().to_sec() - self.time_last_cmd_vel > self.cmd_wait_timeout):
            # send zero velocity command
            # self.publish_cmd_vel_arm(0.,0.,0.,0.,0.,0.)

            if not self.is_zero_cmd_vel_arm_sent_ever:
                rospy.logwarn_once("Zero velocities to the ARM are sent for the first time")
                self.is_zero_cmd_vel_arm_sent_ever = True
        else:
            # send the latest velocity command
            vx = self.arm_cmd[3]
            vy = self.arm_cmd[4]
            vz = self.arm_cmd[5]

            wx = self.arm_cmd[0]
            wy = self.arm_cmd[1]
            wz = self.arm_cmd[2]
            self.publish_cmd_vel_arm(vx,vy,vz, wx,wy,wz)

            self.velocity_command_sent_arm = True
                
            self.is_zero_cmd_vel_arm_sent_ever = False

    def joint_states_arm_callback(self, msg):
        # Figure out at which index in the joint state msg the finger joints at the arm starts
        if self.joint_state_arm is None:
            self.joint_arm_start_index = msg.name.index(self.arm_joints_tf_prefix + "joint_1")
            # If multiple indexes are going to be figured out, comment out above, comment in below, then add more elif conditions
            # for i in range(len(msg.name)):
            #     if msg.name[i] == self.arm_joints_tf_prefix + "joint_finger_1":
            #         self.joint_arm_start_index = i

        # Store the joint state of the arm
        self.joint_state_arm = msg
        # Publish the constraint ball for debug purposes, it uses current arm base pose
        self.publish_constraint_ball()

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

        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz
        
        # Scaling
        cmd_vel.angular.x = wx
        cmd_vel.angular.y = wy
        cmd_vel.angular.z = wz

        # Generate and publish the Twist message
        # cmd_vel = kinova_msgs.msg.PoseVelocity()
        # cmd_vel.twist_linear_x = vx
        # cmd_vel.twist_linear_y = vy
        # cmd_vel.twist_linear_z = vz
        # cmd_vel.twist_angular_x = wx
        # cmd_vel.twist_angular_y = wy
        # cmd_vel.twist_angular_z = wz

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

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

if __name__ == "__main__":
    oarbotRedundancyResolver = OarbotRedundancyResolver()
    rospy.spin()
