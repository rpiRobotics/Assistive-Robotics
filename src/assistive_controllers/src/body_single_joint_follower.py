#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: body_single_joint_follower
Description:
    Waits for a keyboard/joystick/voice input command to 
    get the transform btw. the end effector and the specified body joint.
    Then in an infinite loop tries to keep the same transformation using a proportional control law.
    Control is achieved with the velocity command to the robot arm kinova.
Parameters:
    - 
Subscribes to:
    - tf2
Publishes to:
    - /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::PoseVelocity)
Broadcasts to:
    - 
"""

import rospy

import numpy as np
import tf2_ros
import kinova_msgs.msg

class BodySingleJointFollower():
    def __init__(self):
        rospy.init_node('body_single_joint_follower', anonymous=True)
        self.is_following_started = False

        # Topic name to publish
        self.robot_cartesian_cmd_vel_topic_name = rospy.get_param("~robot_cartesian_cmd_vel_topic_name", "j2n6s300_driver/in/cartesian_velocity")

        # Publish rate
        self.pub_rate = rospy.get_param("pub_rate", 100.0) # 100Hz needed for kinova arm
        self.rate = rospy.Rate(self.pub_rate)

        # Publisher
        self.pub_pose_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)

        # Control Law Gains
        self.v_p_gain = rospy.get_param("v_p_gain", 2*1.5)
        self.v_d_gain = rospy.get_param("v_d_gain", 0.0)
        self.w_p_gain = rospy.get_param("w_p_gain", 1.362366*1.2)
        self.w_d_gain = rospy.get_param("w_d_gain", 0.0)

        # Specified body joint tf frame name to follow
        self.tf_body_joint_frame_name = rospy.get_param("~tf_followed_body_joint_frame_name", "JOINT_WRIST_LEFT").lower() 

        # Specified end effector tf frame name (id)
        self.tf_end_effector_frame_name = rospy.get_param("~tf_end_effector_frame_name", "j2n6s300_end_effector")

        # Specified arm base tf frame name 
        self.tf_arm_base_frame_name = rospy.get_param("~tf_arm_base_frame_name", "j2n6s300_link_base")

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.initial_time = rospy.Time.now().to_sec()

        # Start control
        rospy.Timer(rospy.Duration(1.00/self.pub_rate), self.followJoint)


    def followJoint(self, event=None):
        # Find the transform between the specified joint and the end effector
        try:
            self.T_ee2joint = self.tfBuffer.lookup_transform(self.tf_body_joint_frame_name, self.tf_end_effector_frame_name, rospy.Time()) # in ee frame

            self.T_ee2joint_in_base = self.tfBuffer.transform(self.T_ee2joint, self.tf_arm_base_frame_name) # in base frame (needed since kinova takes position cmd wrt base but orientation cmd wrt end effector)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            # TODO
            # Do not command the robot since the transformation could not found
            self.publishPoseVelCmd(0, 0, 0, 0, 0, 0)

        if self.is_following_started:
            # Calculate the error btw the desired and the current pose
            position_error, orientation_error = self.poseErrorCalculator(self.T_ee2joint_desired, self.T_ee2joint_in_base_desired, self.T_ee2joint, self.T_ee2joint_in_base)

            # With control law specify the command
            Vx, Vy, Vz, Wx, Wy, Wz = self.controlLaw(position_error, orientation_error)

            # Publish the command to move the end effector to the body joint
            self.publishPoseVelCmd(Vx, Vy, Vz, Wx, Wy, Wz)

        else:
            # Wait for user input to start the following 
            # For now wait 15 seconds and then start following
            if (rospy.Time.now().to_sec() - self.initial_time) >= 15.0:
                self.is_following_started = True

            # Save the current transform as the desired pose btw end effector and the desired joint
            self.T_ee2joint_desired = self.T_ee2joint # in ee frame
            self.T_ee2joint_in_base_desired = self.T_ee2joint_in_base # in base frame 
 

    def publishPoseVelCmd(self, Vx, Vy, Vz, Wx, Wy, Wz):
        pose_vel_msg = kinova_msgs.msg.PoseVelocity()
        pose_vel_msg.twist_linear_x = Vx  
        pose_vel_msg.twist_linear_y = Vy
        pose_vel_msg.twist_linear_z = Vz
        pose_vel_msg.twist_angular_x = Wx
        pose_vel_msg.twist_angular_y = Wy
        pose_vel_msg.twist_angular_z = Wz
        self.pub_pose_vel_cmd.publish(pose_vel_msg)

    def poseErrorCalculator(self, T_des_in_ee, T_des_in_base, T_cur_in_ee, T_cur_in_base):
        """
        des: desired
        cur: current
        """
        # Position error
        P_err_x = T_cur_in_base.transform.translation.x - T_des_in_base.transform.translation.x
        P_err_y = T_cur_in_base.transform.translation.y - T_des_in_base.transform.translation.y
        P_err_z = T_cur_in_base.transform.translation.z - T_des_in_base.transform.translation.z
        position_error = np.array([P_err_x,P_err_y,P_err_z]) # (3,)
        
        # Orientation error (with quaternion vector)
        # based on http://www.cs.cmu.edu/~cga/dynopt/readings/Yuan88-quatfeedback.pdf eqn 27,28
        qw_cur = T_cur_in_ee.transform.rotation.w
        qx_cur = T_cur_in_ee.transform.rotation.x
        qy_cur = T_cur_in_ee.transform.rotation.y
        qz_cur = T_cur_in_ee.transform.rotation.z
        qv_cur = np.array([qx_cur,qy_cur,qz_cur]) # (3,) 

        qw_des = T_des_in_ee.transform.rotation.w
        qx_des = T_des_in_ee.transform.rotation.x
        qy_des = T_des_in_ee.transform.rotation.y
        qz_des = T_des_in_ee.transform.rotation.z
        qv_des = np.array([qx_des,qy_des,qz_des]) # (3,)

        orientation_error = qw_cur*qv_des - qw_des*qv_cur - np.cross(qv_cur,qv_des) # (3,)
        # orientation_error = qw_des*qv_cur - qw_cur*qv_des - np.cross(qv_des,qv_cur)
        return position_error, orientation_error

    def controlLaw(self,position_error, orientation_error):
        P_err = [self.allowence(n, 0.005) for n in position_error] # 0.5cm
        R_err = [self.allowence(n, 0.015) for n in orientation_error] # 5 degrees = 0.09 radians
        
        Vx = P_err[0] * self.v_p_gain
        Vy = P_err[1] * self.v_p_gain
        Vz = P_err[2] * self.v_p_gain
        
        Wx = R_err[0] * self.w_p_gain
        Wy = R_err[1] * self.w_p_gain
        Wz = R_err[2] * self.w_p_gain

        return Vx, Vy, Vz, Wx, Wy, Wz

    def allowence(self, x, allowed):
        if abs(x) <= allowed:
            return 0
        else:
            return x


if __name__ == '__main__':
    bodySingleJointFollower = BodySingleJointFollower()
    rospy.spin()