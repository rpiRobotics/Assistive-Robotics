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
Publishes to:
    - /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::PoseVelocity) 
        or /oarbot_xxx/cmd_vel (geometry_msgs::Twist)

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
        # Publishgin msg type (either Geometry_msgs::Twist or kinova_msgs::PoseVelocity)
        self.robot_cartesian_cmd_vel_msg_type = rospy.get_param("~robot_cartesian_cmd_vel_msg_type", "geometry_msgs.msg.Twist")

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0) # 100Hz needed for kinova arm
        self.rate = rospy.Rate(self.pub_rate)
        self.expected_duration = 1.00/self.pub_rate # seconds per cycle

        # Publisher
        if self.robot_cartesian_cmd_vel_msg_type == "geometry_msgs.msg.Twist":
            self.pub_pose_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, geometry_msgs.msg.Twist, queue_size=2)
        elif self.robot_cartesian_cmd_vel_msg_type == "kinova_msgs.msg.PoseVelocity":
            self.pub_pose_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, kinova_msgs.msg.PoseVelocity, queue_size=2)


        # Topic name to subscribe
        self.wrench_external_topic_name = rospy.get_param("~wrench_external_topic_name", "j2n6s300_driver/out/tool_wrench_filtered")
        self.wrench_control_topic_name = rospy.get_param("~wrench_control_topic_name", "j2n6s300_driver/out/tool_wrench_control")
        # Subscriber
        rospy.Subscriber(self.wrench_control_topic_name, geometry_msgs.msg.WrenchStamped, self.wrench_external_callback , queue_size=1)
        rospy.Subscriber(self.wrench_control_topic_name, geometry_msgs.msg.WrenchStamped, self.wrench_control_callback , queue_size=1)

        # Control Law Parameters and Gains
        # Error deadzone 
        self.position_err_thres = rospy.get_param("~position_err_thres", 0.005) # 0.5 cm
        self.orientation_err_thres = rospy.get_param("~orientation_err_thres", 0.015) # 5 degrees = 0.09 radians

        # (Virtual spring)
        self.K_lin_x = rospy.get_param("K_lin_x", 1.0)
        self.K_lin_y = rospy.get_param("K_lin_y", 1.0)
        self.K_lin_z = rospy.get_param("K_lin_z", 1.0)
        self.K_ang_x = rospy.get_param("K_ang_x", 1.0)
        self.K_ang_y = rospy.get_param("K_ang_y", 1.0)
        self.K_ang_z = rospy.get_param("K_ang_z", 1.0)

        # (Virtual damping)
        self.D_lin_x = rospy.get_param("D_lin_x", 0.0)
        self.D_lin_y = rospy.get_param("D_lin_y", 0.0)
        self.D_lin_z = rospy.get_param("D_lin_z", 0.0)
        self.D_ang_x = rospy.get_param("D_ang_x", 0.0)
        self.D_ang_y = rospy.get_param("D_ang_y", 0.0)
        self.D_ang_z = rospy.get_param("D_ang_z", 0.0)

        # Admittance ratio between 0 to 1
        self.admittance_ratio = rospy.get_param("admittance_ratio", 1.0)
        # Make sure self.admittance_ratio is between 0-1
        if self.self.admittance_ratio < 0.0:
            self.admittance_ratio = 0.
        if self.admittance_ratio > 1.0:
            self.admittance_ratio = 1.0

        # (Virtual mass) matrix values
        self.M_lin_x = rospy.get_param("M_lin_x", 1.0)
        self.M_lin_y = rospy.get_param("M_lin_y", 1.0)
        self.M_lin_z = rospy.get_param("M_lin_z", 1.0)
        self.M_ang_x = rospy.get_param("M_ang_x", 1.0)
        self.M_ang_y = rospy.get_param("M_ang_y", 1.0)
        self.M_ang_z = rospy.get_param("M_ang_z", 1.0)

        # Maximum accelaration and velocity
        self.max_lin_acc = rospy.get_param("max_lin_acc", 1000.0)
        self.max_lin_vel = rospy.get_param("max_lin_vel", 1000.0) # Not used
        self.max_ang_acc = rospy.get_param("max_ang_acc", 1000.0)
        self.max_ang_vel = rospy.get_param("max_ang_vel", 1000.0) # Not used

        # Specified body joint tf frame name to follow
        self.tf_body_joint_frame_name = rospy.get_param("~tf_followed_body_joint_frame_name", "JOINT_WRIST_LEFT").lower() 

        # Specified end effector tf frame name (id)
        self.tf_end_effector_frame_name = rospy.get_param("~tf_end_effector_frame_name", "j2n6s300_end_effector")

        # Specified arm base tf frame name 
        self.tf_robot_base_frame_name = rospy.get_param("~tf_robot_base_frame_name", "oarbot_blue_base")

        # Specified arm base tf frame name 
        self.tf_arm_base_frame_name = rospy.get_param("~tf_arm_base_frame_name", "j2n6s300_link_base")

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

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
        self.F_lin_x_control = rospy.get_param("F_lin_x_control", 0.0)
        self.F_lin_y_control = rospy.get_param("F_lin_y_control", 0.0)
        self.F_lin_z_control = rospy.get_param("F_lin_z_control", 0.0)
        self.F_ang_x_control = rospy.get_param("F_ang_x_control", 0.0)
        self.F_ang_y_control = rospy.get_param("F_ang_y_control", 0.0)
        self.F_ang_z_control = rospy.get_param("F_ang_z_control", 0.0)        


    def followJoint(self, event=None):
        # Find the transform between the specified joint and the end effector
        try:
            # returns type geometry_msgs.msg.TransformStamped
            self.T_ee2joint = self.tfBuffer.lookup_transform(self.tf_end_effector_frame_name, self.tf_body_joint_frame_name, rospy.Time()) # in ee frame 
            self.T_base2ee = self.tfBuffer.lookup_transform(self.tf_robot_base_frame_name, self.tf_end_effector_frame_name,  rospy.Time()) # in base frame 
            self.T_base2joint = self.tfBuffer.lookup_transform(self.tf_robot_base_frame_name, self.tf_body_joint_frame_name,  rospy.Time()) # in base frame 
            self.T_base2armbase = self.tfBuffer.lookup_transform(self.tf_robot_base_frame_name, self.tf_arm_base_frame_name,  rospy.Time()) # in base frame 

            if self.is_following_started:
                # Calculate the error btw the desired and the current pose
                position_error, orientation_error = self.poseErrorCalculator()

                # rospy.logwarn("position_error: " + "{:.3f}".format(position_error[0]) + ", {:.3f}".format(position_error[1]) + ", {:.3f}".format(position_error[2])  )
                # rospy.logwarn("orientation_error: " + "{:.2f}".format(orientation_error[0]) + ", {:.2f}".format(orientation_error[1]) + ", {:.2f}".format(orientation_error[2])  )

                # With control law specify the command
                self.Vx, self.Vy, self.Vz, self.Wx, self.Wy, self.Wz = self.controlLaw(position_error, orientation_error)
                # rospy.logwarn("control law result : Vx, Vy, Vz, Wx, Wy, Wz = "+ str([Vx, Vy, Vz, Wx, Wy, Wz]))

                # Publish the command to move the end effector to the body joint
                self.publishPoseVelCmd(self.Vx, self.Vy, self.Vz, self.Wx, self.Wy, self.Wz)

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
            rospy.logwarn('Waiting to find the transformation from %s to %s, OR transformation from %s to %s, OR transformation from %s to %s' 
                            % (self.tf_end_effector_frame_name, self.tf_body_joint_frame_name, 
                            self.tf_robot_base_frame_name, self.tf_end_effector_frame_name,
                            self.tf_robot_base_frame_name, self.tf_arm_base_frame_name)) 
            # Do not command the robot since the transformation could not found
            self.publishPoseVelCmd(0, 0, 0, 0, 0, 0)


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
        P_err = [self.allowence(n, self.position_err_thres) for n in position_error] # 0.5cm
        R_err = [self.allowence(n, self.orientation_err_thres) for n in orientation_error] 
        
        # Virtual Spring (Proportinal Control) (F = K * deltaX)
        F_lin_x = P_err[0] * self.K_lin_x   
        F_lin_y = P_err[1] * self.K_lin_y  
        F_lin_z = P_err[2] * self.K_lin_z  
        F_ang_x = R_err[0] * self.K_ang_x  
        F_ang_y = R_err[1] * self.K_ang_y  
        F_ang_z = R_err[2] * self.K_ang_z  

        # Virtual Damping (Derivative Control) (F = F - D * deltaX_dot)
        F_lin_x -= self.Vx * self.D_lin_x  
        F_lin_y -= self.Vy * self.D_lin_y 
        F_lin_z -= self.Vz * self.D_lin_z 
        F_ang_x -= self.Wx * self.D_ang_x 
        F_ang_y -= self.Wy * self.D_ang_y 
        F_ang_z -= self.Wz * self.D_ang_z 

        # Calculate External Wrench wrt robot mobile base frame from arm base frame
        qw_cur = self.T_base2armbase.transform.rotation.w # Scalar part of quaternion
        qx_cur = self.T_base2armbase.transform.rotation.x
        qy_cur = self.T_base2armbase.transform.rotation.y
        qz_cur = self.T_base2armbase.transform.rotation.z
        q_base2armbase = [qx_cur,qy_cur,qz_cur, qw_cur]
        R_base2armbase = tf.transformations.quaternion_matrix(q_base2armbase)

        F_lin_external = np.array([self.F_lin_x_external,self.F_lin_y_external,self.F_lin_z_external])
        F_ang_external = np.array([self.F_ang_x_external,self.F_ang_y_external,self.F_ang_z_external])
        F_lin_external = np.dot(R_base2armbase,F_lin_external)
        F_ang_external = np.dot(R_base2armbase,F_ang_external)

        # Adding External Force and Desired Control Force
        F_lin_x += (self.admittance_ratio * F_lin_external[0] + self.F_lin_x_control)
        F_lin_y += (self.admittance_ratio * F_lin_external[1] + self.F_lin_y_control)
        F_lin_z += (self.admittance_ratio * F_lin_external[2] + self.F_lin_z_control)
        F_ang_x += (self.admittance_ratio * F_ang_external[0] + self.F_ang_x_control)
        F_ang_y += (self.admittance_ratio * F_ang_external[1] + self.F_ang_y_control)
        F_ang_z += (self.admittance_ratio * F_ang_external[2] + self.F_ang_z_control)        

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
            rospy.logwarn("Admittance generates high linear acceleration!")
            # Normalize the acceleration
            a_lin *= (self.max_lin_acc / a_lin_norm) 
        if a_ang_norm > self.max_ang_acc:
            rospy.logwarn("Admittance generates high angular acceleration!")
            # Normalize the acceleration
            a_ang *= (self.max_ang_acc / a_ang_norm) 

        # Time delay for acc to vel (V = V + a * delta_t)
        Vx = self.Vx + (a_lin_x * self.expected_duration)
        Vy = self.Vy + (a_lin_y * self.expected_duration)
        Vz = self.Vz + (a_lin_z * self.expected_duration)
        Wx = self.Wx + (a_ang_x * self.expected_duration)
        Wy = self.Wy + (a_ang_y * self.expected_duration)
        Wz = self.Wz + (a_ang_z * self.expected_duration)

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


if __name__ == '__main__':
    bodySingleJointFollower = BodySingleJointFollower()
    rospy.spin()