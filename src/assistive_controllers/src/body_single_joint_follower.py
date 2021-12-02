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
import tf2_geometry_msgs
import geometry_msgs.msg 
# from geometry_msgs.msg import PoseStamped, TransformStamped    

import kinova_msgs.msg

# # Because of transformations
import tf_conversions
#  tf_conversions.transformations.euler_from_quaternion(Q_eg)
import tf.transformations 



class BodySingleJointFollower():
    def __init__(self):
        rospy.init_node('body_single_joint_follower', anonymous=True)
        self.is_following_started = False

        # Topic name to publish
        self.robot_cartesian_cmd_vel_topic_name = rospy.get_param("~robot_cartesian_cmd_vel_topic_name", "j2n6s300_driver/in/cartesian_velocity")

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0) # 100Hz needed for kinova arm
        self.rate = rospy.Rate(self.pub_rate)

        # Publisher
        self.pub_pose_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)

        # Control Law Gains
        self.v_p_gain = rospy.get_param("~v_p_gain", 2*1.5)
        self.v_d_gain = rospy.get_param("~v_d_gain", 0.0)
        self.w_p_gain = rospy.get_param("~w_p_gain", 1.362366*1.2)
        self.w_d_gain = rospy.get_param("~w_d_gain", 0.0)

        # Specified body joint tf frame name to follow
        self.tf_body_joint_frame_name = rospy.get_param("~tf_followed_body_joint_frame_name", "JOINT_WRIST_LEFT").lower() 

        # Specified end effector tf frame name (id)
        self.tf_end_effector_frame_name = rospy.get_param("~tf_end_effector_frame_name", "j2n6s300_end_effector")

        # Specified arm base tf frame name 
        self.tf_arm_base_frame_name = rospy.get_param("~tf_arm_base_frame_name", "j2n6s300_link_base")

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # TF2 broadcaster (for showing purposes)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster

        self.initial_time = rospy.Time.now().to_sec()

        # Start control
        rospy.Timer(rospy.Duration(1.00/self.pub_rate), self.followJoint)


    def followJoint(self, event=None):
        # Find the transform between the specified joint and the end effector
        try:
            # returns type geometry_msgs.msg.TransformStamped
            self.T_ee2joint = self.tfBuffer.lookup_transform(self.tf_end_effector_frame_name, self.tf_body_joint_frame_name, rospy.Time()) # in ee frame 
            self.T_base2ee = self.tfBuffer.lookup_transform(self.tf_arm_base_frame_name, self.tf_end_effector_frame_name,  rospy.Time()) # in base frame 
            self.T_base2joint = self.tfBuffer.lookup_transform(self.tf_arm_base_frame_name, self.tf_body_joint_frame_name,  rospy.Time()) # in base frame 

            if self.is_following_started:
                # Calculate the error btw the desired and the current pose
                position_error, orientation_error = self.poseErrorCalculator()

                # rospy.logwarn("position_error: " + "{:.3f}".format(position_error[0]) + ", {:.3f}".format(position_error[1]) + ", {:.3f}".format(position_error[2])  )
                # rospy.logwarn("orientation_error: " + "{:.2f}".format(orientation_error[0]) + ", {:.2f}".format(orientation_error[1]) + ", {:.2f}".format(orientation_error[2])  )

                # With control law specify the command
                Vx, Vy, Vz, Wx, Wy, Wz = self.controlLaw(position_error, orientation_error)
                # rospy.logwarn("control law result : Vx, Vy, Vz, Wx, Wy, Wz = "+ str([Vx, Vy, Vz, Wx, Wy, Wz]))

                # Publish the command to move the end effector to the body joint
                self.publishPoseVelCmd(Vx, Vy, Vz, Wx, Wy, Wz)

            else:
                # Wait for user input to start the following 
                # For now wait 15 seconds and then start following
                if (rospy.Time.now().to_sec() - self.initial_time) >= 5.0:
                    self.is_following_started = True
                    rospy.logwarn_once("FOLLOWING SHOULD START NOW")

                # Save the current Pose as the desired pose btw end effector and the joint to be followed
                self.T_ee2joint_desired = self.T_ee2joint # in ee frame
                self.T_base2ee_desired = self.T_base2ee # in base frame
                self.T_base2joint_desired = self.T_base2joint # in base frame
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            rospy.logwarn('Waiting to find the transformation from %s to %s, OR transformation from %s to %s' 
                            % (self.tf_end_effector_frame_name, self.tf_body_joint_frame_name, 
                            self.tf_arm_base_frame_name, self.tf_end_effector_frame_name))
            # Do not command the robot since the transformation could not found
            self.publishPoseVelCmd(0, 0, 0, 0, 0, 0)


    def publishPoseVelCmd(self, Vx, Vy, Vz, Wx, Wy, Wz):
        pose_vel_msg = kinova_msgs.msg.PoseVelocity()
        pose_vel_msg.twist_linear_x = Vx  
        pose_vel_msg.twist_linear_y = Vy
        pose_vel_msg.twist_linear_z = Vz
        pose_vel_msg.twist_angular_x = Wx
        pose_vel_msg.twist_angular_y = Wy
        pose_vel_msg.twist_angular_z = Wz
        self.pub_pose_vel_cmd.publish(pose_vel_msg)

    def poseErrorCalculator(self):
        """
        des: desired
        cur: current
        """
        # Position error in base
        P_ee2joint_in_base_x = (self.T_base2joint.transform.translation.x - self.T_base2ee.transform.translation.x)
        P_ee2joint_in_base_y = (self.T_base2joint.transform.translation.y - self.T_base2ee.transform.translation.y)
        P_ee2joint_in_base_z = (self.T_base2joint.transform.translation.z - self.T_base2ee.transform.translation.z)
        P_ee2joint_in_base = np.array([P_ee2joint_in_base_x,P_ee2joint_in_base_y,P_ee2joint_in_base_z])

        P_ee2joint_desired_in_base_x = (self.T_base2joint_desired.transform.translation.x - self.T_base2ee_desired.transform.translation.x) 
        P_ee2joint_desired_in_base_y = (self.T_base2joint_desired.transform.translation.y - self.T_base2ee_desired.transform.translation.y) 
        P_ee2joint_desired_in_base_z = (self.T_base2joint_desired.transform.translation.z - self.T_base2ee_desired.transform.translation.z) 
        P_ee2joint_desired_in_base = np.array([P_ee2joint_desired_in_base_x,P_ee2joint_desired_in_base_y,P_ee2joint_desired_in_base_z])

        position_error = (P_ee2joint_in_base - P_ee2joint_desired_in_base).tolist() # (3,)
        
        # Orientation error (with quaternion vector)
        # based on http://www.cs.cmu.edu/~cga/dynopt/readings/Yuan88-quatfeedback.pdf eqn 27,28

        # Quaternion base2joint (current)
        qw_cur = self.T_base2joint.transform.rotation.w # Scalar part of quaternion
        qx_cur = self.T_base2joint.transform.rotation.x
        qy_cur = self.T_base2joint.transform.rotation.y
        qz_cur = self.T_base2joint.transform.rotation.z
        q_base2joint = [qx_cur,qy_cur,qz_cur, qw_cur]
        R_base2joint = tf.transformations.quaternion_matrix(q_base2joint)

        # Quaternion ee2joint_desired
        qw_des = self.T_ee2joint_desired.transform.rotation.w # Scalar part of quaternion
        qx_des = self.T_ee2joint_desired.transform.rotation.x
        qy_des = self.T_ee2joint_desired.transform.rotation.y
        qz_des = self.T_ee2joint_desired.transform.rotation.z
        q_ee2joint_desired = [qx_des,qy_des,qz_des, qw_des]
        R_ee2joint_desired = tf.transformations.quaternion_matrix(q_ee2joint_desired)
        q_ee2joint_desired_inv = tf_conversions.transformations.quaternion_inverse(q_ee2joint_desired)

        # Quaternion base2ee_goal
        # q_base2ee_goal = tf.transformations.quaternion_multiply(q_base2joint, q_ee2joint_desired_inv)
        R_base2ee_goal = np.dot(R_base2joint, R_ee2joint_desired.T)
        # q_base2ee_goal_inv = tf_conversions.transformations.quaternion_inverse(R_base2ee_goal)
        

        # Quaternion base2ee (current)
        qw_cur = self.T_base2ee.transform.rotation.w # Scalar part of quaternion
        qx_cur = self.T_base2ee.transform.rotation.x
        qy_cur = self.T_base2ee.transform.rotation.y
        qz_cur = self.T_base2ee.transform.rotation.z
        q_base2ee = [qx_cur,qy_cur,qz_cur, qw_cur]
        R_base2ee = tf.transformations.quaternion_matrix(q_base2ee)
        # q_base2ee_inv = tf_conversions.transformations.quaternion_inverse(q_base2ee)

        # Quaternion orientation_error
        # q_orientation_error = tf.transformations.quaternion_multiply(q_base2ee_inv,q_base2ee_goal)
        # or
        # q_orientation_error = tf.transformations.quaternion_multiply(q_base2ee,q_base2ee_goal_inv)
        # orientation_error = q_orientation_error[0:3].tolist()

        # Rotation orientation error
        R_orientation_error = np.dot(R_base2ee.T, R_base2ee_goal)

        # Euler XYZ
        # orientation_error = tf_conversions.transformations.euler_from_quaternion(q_orientation_error)
        
        #orientation_error = tf_conversions.transformations.euler_from_matrix(R_orientation_error)

        q_orientation_error = tf_conversions.transformations.quaternion_from_matrix(R_orientation_error)
        orientation_error = q_orientation_error[0:3].tolist()

        # Publish a tf frame for showing the goal for the robot
        # P_ee2joint_desired_x = self.T_ee2joint_desired.transform.translation.x
        # P_ee2joint_desired_y = self.T_ee2joint_desired.transform.translation.y
        # P_ee2joint_desired_z = self.T_ee2joint_desired.transform.translation.z
        # P_ee2joint_desired = np.array([P_ee2joint_desired_x,P_ee2joint_desired_y,P_ee2joint_desired_z])
        # P_joint2goal = np.dot(-R_ee2joint_desired[:3,:3].T,P_ee2joint_desired)

        P_joint2goal = np.dot(R_base2joint[:3,:3].T,-P_ee2joint_desired_in_base)
        self.broadcast_tf_goal(q_ee2joint_desired_inv,P_joint2goal)

        return position_error, orientation_error

    def controlLaw(self,position_error, orientation_error):
        P_err = [self.allowence(n, 0.005) for n in position_error] # 0.5cm
        R_err = [self.allowence(n, 0.015) for n in orientation_error] # 5 degrees = 0.09 radians
        
        Vx = P_err[0] * self.v_p_gain
        Vy = P_err[1] * self.v_p_gain
        Vz = P_err[2] * self.v_p_gain
        
        Wx = R_err[0] * self.w_p_gain #*0.5
        Wy = R_err[1] * self.w_p_gain #*0.5
        Wz = R_err[2] * self.w_p_gain #*1.5 

        return Vx, Vy, Vz, Wx, Wy, Wz

    def allowence(self, x, allowed):
        if abs(x) <= allowed:
            return 0
        else:
            return x

    def broadcast_tf_goal(self, q_joint2goal, p_joint2goal):
        """
        For showing purposes,
        Broadcasts a frame to tf from the specified body joint.
        This broadcasted frame is the one that is the end efffector tries to align itself
        """
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = self.tf_body_joint_frame_name
        t.child_frame_id = self.tf_body_joint_frame_name + "_goal"

        t.transform.translation.x = p_joint2goal[0]
        t.transform.translation.y = p_joint2goal[1]
        t.transform.translation.z = p_joint2goal[2]
        
        t.transform.rotation.x = q_joint2goal[0]
        t.transform.rotation.y = q_joint2goal[1]
        t.transform.rotation.z = q_joint2goal[2]
        t.transform.rotation.w = q_joint2goal[3]

        self.tf_broadcaster.sendTransform(t)



if __name__ == '__main__':
    bodySingleJointFollower = BodySingleJointFollower()
    rospy.spin()