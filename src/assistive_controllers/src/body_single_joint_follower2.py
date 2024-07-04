#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: body_single_joint_follower
Description:
    Waits for a keyboard/joystick/voice input command to (TODO, currently just waits couple of seconds)
    get the transform btw. the end effector and the specified body joint.
    Then in an infinite loop tries to keep the same transformation using a proportional control law.
    Control is achieved with the velocity command to the robot arm or the whole oarbot.
Parameters:
    - TODO
Subscribes to:
    - tf2
    - geometry_msgs::WrenchStamped (external): to be used to calculate the applied force
    - geometry_msgs::WrenchStamped (control): to be used to carry a payload (optional)
Publishes to:
    - /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::PoseVelocity) 
        or /oarbot_xxx/cmd_vel (geometry_msgs::Twist)

Broadcasts to:
    - 
"""

import rospy
import time


import numpy as np
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg 
# from geometry_msgs.msg import PoseStamped, TransformStamped    

import kinova_msgs.msg

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# # Because of transformations
import tf_conversions
#  tf_conversions.transformations.euler_from_quaternion(Q_eg)
import tf.transformations 

class BodySingleJointFollower():
    def __init__(self):
        rospy.init_node('body_single_joint_follower', anonymous=True)

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


        # Topic name to subscribe
        self.wrench_external_topic_name = rospy.get_param("~wrench_external_topic_name", "j2n6s300_driver/out/tool_wrench_filtered")
        self.wrench_control_topic_name = rospy.get_param("~wrench_control_topic_name", "j2n6s300_driver/out/tool_wrench_control")
        # Subscriber
        rospy.Subscriber(self.wrench_external_topic_name, geometry_msgs.msg.WrenchStamped, self.wrench_external_callback , queue_size=1)
        rospy.Subscriber(self.wrench_control_topic_name, geometry_msgs.msg.WrenchStamped, self.wrench_control_callback , queue_size=1)

        self.enable_body_joint_following = rospy.get_param("~enable_body_joint_following", False)
        self.enable_admittance = rospy.get_param("~enable_admittance", False)
        self.toggle_body_joint_following_service_name = rospy.get_param("~toggle_body_joint_following_service_name")
        self.toggle_admittance_service_name = rospy.get_param("~toggle_admittance_service_name")
        # Service to toggle the body joint following (enable/disable)
        self.srv_toggle_body_joint_following = rospy.Service(self.toggle_body_joint_following_service_name, SetBool, self.srv_toggle_body_joint_following_cb)
        # Service to toggle the admittance control (enable/disable)
        self.srv_toggle_admittance = rospy.Service(self.toggle_admittance_service_name, SetBool, self.srv_toggle_admittance_cb)
        
        # Service to reset the ft bias (It is called from here to reset the bias)
        self.reset_ft_bias_service_address = rospy.get_param("~reset_ft_bias_service_address", "imu_gravity_compensation/calibrate_bias")
        # it is a service to zero the bias of the force torque sensor readings, provided by the gravity_compensation package. 
        # Hence the type is std_srvs/Empty based on the service definition of gravity_compensation package.


        self.reset_desired_body_pose_service_name = rospy.get_param("~reset_desired_body_pose_service_name")
        # Service to reset the desired body poses
        self.srv_reset_desired_body_pose = rospy.Service(self.reset_desired_body_pose_service_name, Trigger, self.srv_reset_desired_body_pose_cb)

        # Control Law Parameters and Gains
        # Error deadzone 
        self.position_err_thres = rospy.get_param("~position_err_thres", 0.005) # 0.5 cm
        self.orientation_err_thres = rospy.get_param("~orientation_err_thres", 0.015) # 5 degrees = 0.09 radians

        # (Virtual spring)
        self.K_lin_x = rospy.get_param("~K_lin_x", 0.0)
        self.K_lin_y = rospy.get_param("~K_lin_y", 0.0)
        self.K_lin_z = rospy.get_param("~K_lin_z", 0.0)
        self.K_ang_x = rospy.get_param("~K_ang_x", 0.0)
        self.K_ang_y = rospy.get_param("~K_ang_y", 0.0)
        self.K_ang_z = rospy.get_param("~K_ang_z", 0.0)

        # (Virtual damping)
        self.D_lin_x = rospy.get_param("~D_lin_x", 0.0)
        self.D_lin_y = rospy.get_param("~D_lin_y", 0.0)
        self.D_lin_z = rospy.get_param("~D_lin_z", 0.0)
        self.D_ang_x = rospy.get_param("~D_ang_x", 0.0)
        self.D_ang_y = rospy.get_param("~D_ang_y", 0.0)
        self.D_ang_z = rospy.get_param("~D_ang_z", 0.0)

        # Admittance ratio between 0 to 1
        self.K_admittance_lin_x = rospy.get_param("~K_admittance_lin_x", 1.0)
        self.K_admittance_lin_y = rospy.get_param("~K_admittance_lin_y", 1.0)
        self.K_admittance_lin_z = rospy.get_param("~K_admittance_lin_z", 1.0)
        self.K_admittance_ang_x = rospy.get_param("~K_admittance_ang_x", 1.0)
        self.K_admittance_ang_y = rospy.get_param("~K_admittance_ang_y", 1.0)
        self.K_admittance_ang_z = rospy.get_param("~K_admittance_ang_z", 1.0)

        # (Virtual mass) matrix values
        self.M_lin_x = rospy.get_param("~M_lin_x", 1.0)
        self.M_lin_y = rospy.get_param("~M_lin_y", 1.0)
        self.M_lin_z = rospy.get_param("~M_lin_z", 1.0)
        self.M_ang_x = rospy.get_param("~M_ang_x", 1.0)
        self.M_ang_y = rospy.get_param("~M_ang_y", 1.0)
        self.M_ang_z = rospy.get_param("~M_ang_z", 1.0)

        # Maximum accelaration and velocity
        self.max_lin_acc = rospy.get_param("~max_lin_acc", 1000.0)
        self.max_lin_vel = rospy.get_param("~max_lin_vel", 1000.0)
        self.max_ang_acc = rospy.get_param("~max_ang_acc", 1000.0)
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 1000.0)

        # Specified body joint tf frame name to follow
        self.tf_body_joint_frame_name = rospy.get_param("~tf_followed_body_joint_frame_name", "JOINT_WRIST_LEFT").lower() 

        # Specified end effector tf frame name (id)
        self.tf_end_effector_frame_name = rospy.get_param("~tf_end_effector_frame_name", "j2n6s300_end_effector")

        # Specified arm base tf frame name 
        self.tf_robot_base_frame_name = rospy.get_param("~tf_robot_base_frame_name", "oarbot_blue_base")

        # Specified arm base tf frame name 
        self.tf_arm_base_frame_name = rospy.get_param("~tf_arm_base_frame_name", "j2n6s300_link_base")
        
        self.tf_followed_body_joint_frame_name_goal = rospy.get_param("~tf_followed_body_joint_frame_name_goal", "body_joint_neck_goal_right")

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.T_base2ee = None
        self.T_ee2joint = None
        self.T_base2joint = None
        self.T_base2armbase = None

        self.is_ok_tf_common = False
        self.is_ok_tf_body_follower = False
        self.is_ok_tf_body_follower_desired = False
        self.is_ok_tf_admittance = False

        self.T_ee2joint_desired = None
        self.T_base2ee_desired = None
        self.T_base2joint_desired = None

        # TF2 broadcaster (for showing purposes)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster

        self.initial_time = rospy.Time.now().to_sec()

        # Start control
        rospy.Timer(rospy.Duration(self.expected_duration), self.followJoint)

        # To store the published velocities
        self.Vx = 0.0
        self.Vy = 0.0
        self.Vz = 0.0
        self.Wx = 0.0
        self.Wy = 0.0
        self.Wz = 0.0

        # To store the applied force 
        self.F_lin_x_external = 0. 
        self.F_lin_y_external = 0. 
        self.F_lin_z_external = 0. 
        self.F_ang_x_external = 0. 
        self.F_ang_y_external = 0. 
        self.F_ang_z_external = 0. 

        # To store the control wrench as payload 
        self.F_lin_x_control = rospy.get_param("~F_lin_x_control", 0.0)
        self.F_lin_y_control = rospy.get_param("~F_lin_y_control", 0.0)
        self.F_lin_z_control = rospy.get_param("~F_lin_z_control", 0.0)
        self.F_ang_x_control = rospy.get_param("~F_ang_x_control", 0.0)
        self.F_ang_y_control = rospy.get_param("~F_ang_y_control", 0.0)
        self.F_ang_z_control = rospy.get_param("~F_ang_z_control", 0.0)        


    def followJoint(self, event=None):
        # Find the transform between the specified robot base and the end effector
        self.is_ok_tf_common = self.look_tfs_for_common()
        
        if not self.is_ok_tf_common:
            # Do not command the robot since the transformation could not found
            self.Vx = 0.0
            self.Vy = 0.0
            self.Vz = 0.0
            self.Wx = 0.0
            self.Wy = 0.0
            self.Wz = 0.0
            self.publishPoseVelCmd(self.Vx, self.Vy, self.Vz, self.Wx, self.Wy, self.Wz)
            return
        
        # -------------
        # Update the other transformations
        if self.enable_body_joint_following:
            self.is_ok_tf_body_follower = self.look_tfs_for_body_follower()
            
            # Check if the desired poses are set, if not set them
            if self.T_base2ee_desired is None or self.T_ee2joint_desired is None or self.T_base2joint_desired is None:
                # Save the current Pose as the desired pose btw end effector and the joint to be followed
                self.is_ok_tf_body_follower_desired = self.reset_desired_body_pose()
            else:
                # Otherwise, the desired poses are updated by the service
                pass
            
        if self.enable_admittance:
            self.is_ok_tf_admittance = self.look_tfs_for_admittance()
        # -------------

        # -------------
        if self.is_ok_tf_body_follower and self.is_ok_tf_body_follower_desired and self.enable_body_joint_following:
            # Calculate the error btw the desired and the current pose
            position_error, orientation_error = self.poseErrorCalculator()
        else:
            position_error = [0.0,0.0,0.0]
            orientation_error = [0.0,0.0,0.0]

        # rospy.logwarn("position_error: " + "{:.3f}".format(position_error[0]) + ", {:.3f}".format(position_error[1]) + ", {:.3f}".format(position_error[2])  )
        # rospy.logwarn("orientation_error: " + "{:.2f}".format(orientation_error[0]) + ", {:.2f}".format(orientation_error[1]) + ", {:.2f}".format(orientation_error[2])  )

        # With control law specify the command
        self.Vx, self.Vy, self.Vz, self.Wx, self.Wy, self.Wz = self.controlLaw(position_error, orientation_error)
        # rospy.logwarn("control law result : Vx, Vy, Vz, Wx, Wy, Wz = "+ str([self.Vx, self.Vy, self.Vz, self.Wx, self.Wy, self.Wz]))

        if self.enable_body_joint_following or self.enable_admittance:
            # Publish the command to move the end effector to the body joint
            self.publishPoseVelCmd(self.Vx, self.Vy, self.Vz, self.Wx, self.Wy, self.Wz)
        # -------------

    def look_tfs_for_common(self, timeout=0.0):
        try:
            # returns type geometry_msgs.msg.TransformStamped
            self.T_base2ee = self.tfBuffer.lookup_transform(self.tf_robot_base_frame_name, 
                                                            self.tf_end_effector_frame_name,  
                                                            rospy.Time(),
                                                            rospy.Duration(timeout)) # in base frame 
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            msg = "TFs_common: Waiting to find the transformation from %s to %s" \
                % (self.tf_robot_base_frame_name, self.tf_end_effector_frame_name)

            # rospy.logerr("TFs_common: Waiting to find the transformation from %s to %s" 
            #                 % (self.tf_robot_base_frame_name, self.tf_end_effector_frame_name))
            rospy.logerr_throttle(20.0, msg+" (throttled to 20s)")
            return False

    def look_tfs_for_body_follower(self, timeout=0.0):
        try:
            # returns type geometry_msgs.msg.TransformStamped
            self.T_ee2joint = self.tfBuffer.lookup_transform(self.tf_end_effector_frame_name, 
                                                             self.tf_body_joint_frame_name, 
                                                             rospy.Time(),
                                                             rospy.Duration(timeout)) # in ee frame     
            self.T_base2joint = self.tfBuffer.lookup_transform(self.tf_robot_base_frame_name, 
                                                               self.tf_body_joint_frame_name,
                                                               rospy.Time(),
                                                               rospy.Duration(timeout)) # in base frame  
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            # rospy.logwarn('TFs_body_follower: Waiting to find the transformations') 
            rospy.logwarn_throttle(20.0, "TFs_body_follower: Waiting to find the transformations (throttled to 20s)")
            return False

    def look_tfs_for_admittance(self, timeout=0.0):
        try:
            # returns type geometry_msgs.msg.TransformStamped
            self.T_base2armbase = self.tfBuffer.lookup_transform(self.tf_robot_base_frame_name, 
                                                                 self.tf_arm_base_frame_name,  
                                                                 rospy.Time(),
                                                                 rospy.Duration(timeout)) # in base frame 
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            # rospy.logwarn('TFs_admittance: Waiting to find the transformations') 
            rospy.logwarn_throttle(20.0, "TFs_admittance: Waiting to find the transformations (throttled to 20s)")
            return False


    def publishPoseVelCmd(self, Vx, Vy, Vz, Wx, Wy, Wz):
        if self.robot_cartesian_cmd_vel_msg_type == "geometry_msgs.msg.Twist":
            pose_vel_msg = geometry_msgs.msg.Twist()
            pose_vel_msg.linear.x = Vx
            pose_vel_msg.linear.y = Vy
            pose_vel_msg.linear.z = Vz
            pose_vel_msg.angular.x = Wx
            pose_vel_msg.angular.y = Wy
            pose_vel_msg.angular.z = Wz
            self.pub_pose_vel_cmd.publish(pose_vel_msg)

        elif self.robot_cartesian_cmd_vel_msg_type == "kinova_msgs.msg.PoseVelocity":
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
        
        P_ee2joint_desired_in_ee = np.array([self.T_ee2joint_desired.transform.translation.x,
                                            self.T_ee2joint_desired.transform.translation.y,
                                            self.T_ee2joint_desired.transform.translation.z]) # P_{EJ}(t=0) in E
        q_ee2joint_desired = [self.T_ee2joint_desired.transform.rotation.x,
                              self.T_ee2joint_desired.transform.rotation.y,
                              self.T_ee2joint_desired.transform.rotation.z,
                              self.T_ee2joint_desired.transform.rotation.w]
        q_ee2joint_desired_inv = tf_conversions.transformations.quaternion_inverse(q_ee2joint_desired)
        R_joint2ee_desired = tf_conversions.transformations.quaternion_matrix(q_ee2joint_desired_inv)[:3,:3] # R_{JE}(t=0)
        # R_joint2ee_desired = np.transpose(tf_conversions.transformations.quaternion_matrix(q_ee2joint_desired)[:3,:3]) # R_{JE}(t=0)
        P_joint2ee_desired = np.dot(-R_joint2ee_desired,P_ee2joint_desired_in_ee) # P_{JE}(t=0) in J
        
        self.broadcast_tf_goal(q_ee2joint_desired_inv,P_joint2ee_desired)
        
        P_ee2joint_in_ee = np.array([self.T_ee2joint.transform.translation.x,
                                     self.T_ee2joint.transform.translation.y,
                                     self.T_ee2joint.transform.translation.z]) # P_{EJ}(t) in E
        q_ee2joint = [self.T_ee2joint.transform.rotation.x,
                      self.T_ee2joint.transform.rotation.y,
                      self.T_ee2joint.transform.rotation.z,
                      self.T_ee2joint.transform.rotation.w]
        R_ee2joint = tf_conversions.transformations.quaternion_matrix(q_ee2joint)[:3,:3] # R_{EJ}(t)
        
        q_base2ee = [self.T_base2ee.transform.rotation.x,
                     self.T_base2ee.transform.rotation.y,
                     self.T_base2ee.transform.rotation.z,
                     self.T_base2ee.transform.rotation.w]
        R_base2ee = tf_conversions.transformations.quaternion_matrix(q_base2ee)[:3,:3] # R_{BE}(t)
        
        # Position error in base
        position_error = np.dot(R_base2ee, P_ee2joint_in_ee + np.dot(R_ee2joint, P_joint2ee_desired)).tolist() # (3,)
        
        # Orientation error (with quaternion vector)
        # based on http://www.cs.cmu.edu/~cga/dynopt/readings/Yuan88-quatfeedback.pdf eqn 27,28

        R_orientation_error = np.dot(R_ee2joint, R_joint2ee_desired) # R_{EJ}(t) * R_{JE}(t=0)
        q_orientation_error = tf_conversions.transformations.quaternion_from_matrix(R_orientation_error)
        orientation_error = q_orientation_error[0:3].tolist()

        return position_error, orientation_error

    def poseErrorCalculator_old(self):
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

    def controlLaw(self,position_error=[0.0,0.0,0.0], orientation_error=[0.0,0.0,0.0]):
        # if self.enable_body_joint_following:
        P_err = [self.allowence(n, self.position_err_thres) for n in position_error] # 0.5cm
        R_err = [self.allowence(n, self.orientation_err_thres) for n in orientation_error] 
        # else:
        #     P_err = [0.0,0.0,0.0]
        #     R_err = [0.0,0.0,0.0]
            
        # Virtual Spring (Proportinal Control) (F = K * deltaX)
        F_lin_x = P_err[0] * self.K_lin_x   
        F_lin_y = P_err[1] * self.K_lin_y  
        F_lin_z = P_err[2] * self.K_lin_z  
        F_ang_x = R_err[0] * self.K_ang_x  
        F_ang_y = R_err[1] * self.K_ang_y  
        F_ang_z = R_err[2] * self.K_ang_z  

        # Virtual Damping (Derivative Control) (F = F - D * deltaX_dot)
        F_lin_x = F_lin_x - (self.Vx * self.D_lin_x)  
        F_lin_y = F_lin_y - (self.Vy * self.D_lin_y) 
        F_lin_z = F_lin_z - (self.Vz * self.D_lin_z) 
        F_ang_x = F_ang_x - (self.Wx * self.D_ang_x) 
        F_ang_y = F_ang_y - (self.Wy * self.D_ang_y) 
        F_ang_z = F_ang_z - (self.Wz * self.D_ang_z) 

        if self.enable_admittance and self.is_ok_tf_admittance:
            # Calculate External Wrench wrt robot mobile base frame from arm base frame
            qw_cur = self.T_base2armbase.transform.rotation.w # Scalar part of quaternion
            qx_cur = self.T_base2armbase.transform.rotation.x
            qy_cur = self.T_base2armbase.transform.rotation.y
            qz_cur = self.T_base2armbase.transform.rotation.z
            q_base2armbase = [qx_cur,qy_cur,qz_cur, qw_cur]
            R_base2armbase = tf.transformations.quaternion_matrix(q_base2armbase)

            # Quaternion base2ee (current)
            qw_cur = self.T_base2ee.transform.rotation.w # Scalar part of quaternion
            qx_cur = self.T_base2ee.transform.rotation.x
            qy_cur = self.T_base2ee.transform.rotation.y
            qz_cur = self.T_base2ee.transform.rotation.z
            q_base2ee = [qx_cur,qy_cur,qz_cur, qw_cur]
            R_base2ee = tf.transformations.quaternion_matrix(q_base2ee)

            R_ee2armbase = np.dot(R_base2ee[:3,:3].T,R_base2armbase[:3,:3]) 

            F_lin_external = np.array([self.F_lin_x_external,self.F_lin_y_external,self.F_lin_z_external])
            F_ang_external = np.array([self.F_ang_x_external,self.F_ang_y_external,self.F_ang_z_external])
            F_lin_external = np.dot(R_base2armbase[:3,:3],F_lin_external) # Linear must be wrt mobile base
            F_ang_external = np.dot(R_ee2armbase,F_ang_external) # Angular must be wrt end effector

            # Adding External Force and Desired Control Force
            F_lin_x = F_lin_x + (self.K_admittance_lin_x * F_lin_external[0] + self.F_lin_x_control)
            F_lin_y = F_lin_y + (self.K_admittance_lin_y * F_lin_external[1] + self.F_lin_y_control)
            F_lin_z = F_lin_z + (self.K_admittance_lin_z * F_lin_external[2] + self.F_lin_z_control)
            F_ang_x = F_ang_x + (self.K_admittance_ang_x * F_ang_external[0] + self.F_ang_x_control)
            F_ang_y = F_ang_y + (self.K_admittance_ang_y * F_ang_external[1] + self.F_ang_y_control)
            F_ang_z = F_ang_z + (self.K_admittance_ang_z * F_ang_external[2] + self.F_ang_z_control)        

        # Virtual Mass (a = F/m)
        a_lin_x = F_lin_x / self.M_lin_x
        a_lin_y = F_lin_y / self.M_lin_y
        a_lin_z = F_lin_z / self.M_lin_z
        a_ang_x = F_ang_x / self.M_ang_x
        a_ang_y = F_ang_y / self.M_ang_y
        a_ang_z = F_ang_z / self.M_ang_z

        # Limiting acceleration
        a_lin = np.array([a_lin_x, a_lin_y, a_lin_z])
        a_lin_norm = np.linalg.norm(a_lin)
        a_ang = np.array([a_ang_x, a_ang_y, a_ang_z])
        a_ang_norm = np.linalg.norm(a_ang)
    
        if a_lin_norm > self.max_lin_acc:
            # rospy.logwarn("Follower generates high linear acceleration!")
            rospy.logwarn_throttle(20.0, "Follower generates high linear acceleration! (throttled to 20s)")
            # Normalize the acceleration
            a_lin *= (self.max_lin_acc / a_lin_norm) 
        if a_ang_norm > self.max_ang_acc:
            # rospy.logwarn("Follower generates high angular acceleration!")
            rospy.logwarn_throttle(20.0, "Follower generates high angular acceleration! (throttled to 20s)")
            # Normalize the acceleration
            a_ang *= (self.max_ang_acc / a_ang_norm) 

        # Time delay for acc to vel (V = V + a * delta_t)
        v_lin_x = self.Vx + (a_lin[0] * self.expected_duration)
        v_lin_y = self.Vy + (a_lin[1] * self.expected_duration)
        v_lin_z = self.Vz + (a_lin[2] * self.expected_duration)
        v_ang_x = self.Wx + (a_ang[0] * self.expected_duration)
        v_ang_y = self.Wy + (a_ang[1] * self.expected_duration)
        v_ang_z = self.Wz + (a_ang[2] * self.expected_duration)

        # Limiting velocity
        v_lin = np.array([v_lin_x, v_lin_y, v_lin_z])
        v_lin_norm = np.linalg.norm(v_lin)
        v_ang = np.array([v_ang_x, v_ang_y, v_ang_z])
        v_ang_norm = np.linalg.norm(v_ang)
    
        if v_lin_norm > self.max_lin_vel:
            # rospy.logwarn("Follower generates high linear velocity!")
            rospy.logwarn_throttle(20.0, "Follower generates high linear velocity! (throttled to 20s)")
            # Normalize the velocity
            v_lin *= (self.max_lin_vel / v_lin_norm) 
        if v_ang_norm > self.max_ang_vel:
            # rospy.logwarn("Follower generates high angular velocity!")
            rospy.logwarn_throttle(20.0, "Follower generates high angular velocity! (throttled to 20s)")
            # Normalize the velocity
            v_ang *= (self.max_ang_vel / v_ang_norm) 

        Vx = v_lin[0]
        Vy = v_lin[1]
        Vz = v_lin[2]
        Wx = v_ang[0]
        Wy = v_ang[1]
        Wz = v_ang[2]

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
        t.child_frame_id = self.tf_followed_body_joint_frame_name_goal

        t.transform.translation.x = p_joint2goal[0]
        t.transform.translation.y = p_joint2goal[1]
        t.transform.translation.z = p_joint2goal[2]
        
        t.transform.rotation.x = q_joint2goal[0]
        t.transform.rotation.y = q_joint2goal[1]
        t.transform.rotation.z = q_joint2goal[2]
        t.transform.rotation.w = q_joint2goal[3]

        self.tf_broadcaster.sendTransform(t)

    def wrench_external_callback(self, wrench_stamped_msg):
        self.F_lin_x_external = wrench_stamped_msg.wrench.force.x
        self.F_lin_y_external = wrench_stamped_msg.wrench.force.y
        self.F_lin_z_external = wrench_stamped_msg.wrench.force.z
        self.F_ang_x_external = wrench_stamped_msg.wrench.torque.x
        self.F_ang_y_external = wrench_stamped_msg.wrench.torque.y
        self.F_ang_z_external = wrench_stamped_msg.wrench.torque.z

    def wrench_control_callback(self, wrench_stamped_msg):
        self.F_lin_x_control = wrench_stamped_msg.wrench.force.x
        self.F_lin_y_control = wrench_stamped_msg.wrench.force.y
        self.F_lin_z_control = wrench_stamped_msg.wrench.force.z
        self.F_ang_x_control = wrench_stamped_msg.wrench.torque.x
        self.F_ang_y_control = wrench_stamped_msg.wrench.torque.y
        self.F_ang_z_control = wrench_stamped_msg.wrench.torque.z


    def srv_toggle_body_joint_following_cb(self,req):
        assert isinstance(req, SetBoolRequest)

        if req.data:
            self.enable_body_joint_following = True
            rospy.loginfo("Enable body following")
        else:
            self.enable_body_joint_following = False
            rospy.loginfo("Disable body following")

        return SetBoolResponse(True, "The body_joint_following is now set to: {}".format(self.enable_body_joint_following))


    def srv_toggle_admittance_cb(self,req):
        assert isinstance(req, SetBoolRequest)

        if req.data:
            try:
                rospy.loginfo("Calling the service to reset the FT sensor bias")
                
                rospy.wait_for_service(self.reset_ft_bias_service_address, timeout=1.0)
                service = rospy.ServiceProxy(self.reset_ft_bias_service_address, Empty)
                request = EmptyRequest()  # Create an empty request object (not necessary but shown for completeness)
                response = service(request)  # Call the service with the empty request
                
                rospy.sleep(1.0) # Wait for the bias to be reset
                self.enable_admittance = True
                rospy.loginfo("Enable admittance control")
                
            except rospy.ServiceException as e:
                rospy.logerr("Service call to reset the FT sensor bias failed: %s" % e)
                return SetBoolResponse(False, "The admittance toggle was not successful")   
            # except rospy.ROSException as e:
            #     rospy.logerr(f"Failed to contact service: {self.reset_ft_bias_service_address}")
            #     return SetBoolResponse(False, "The admittance toggle was not successful")   

        else:
            self.enable_admittance = False
            rospy.loginfo("Disable admittance control")

        return SetBoolResponse(True, "The admittance is now set to: {}".format(self.enable_admittance))

    def srv_reset_desired_body_pose_cb(self,req):
        assert isinstance(req, TriggerRequest)

        self.is_ok_tf_body_follower_desired = self.reset_desired_body_pose()
        rospy.loginfo("Resetting desired body poses if possible")

        return TriggerResponse(success=True, message="The desired body poses are reset!")

    def reset_desired_body_pose(self):
        # This is not required to be time critical, 
        # so we can wait for the tfs to be available by setting timeout
        self.is_ok_tf_common = self.look_tfs_for_common(timeout=1.0)
        self.is_ok_tf_body_follower = self.look_tfs_for_body_follower(timeout=1.0)
        
        if self.is_ok_tf_common and self.is_ok_tf_body_follower:
            # Save the current Pose as the desired pose btw end effector and the joint to be followed
            self.T_ee2joint_desired = self.T_ee2joint # in ee frame
            self.T_base2ee_desired = self.T_base2ee # in base frame
            self.T_base2joint_desired = self.T_base2joint # in base frame
            return True
        else:
            return False


if __name__ == '__main__':
    bodySingleJointFollower = BodySingleJointFollower()
    rospy.spin()