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
    - tf2
Publishes to:
    - /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::PoseVelocity)
    - /$(robot)/cmd_vel (geometry_msgs::Twist)
    - /$(robot)/cmd_vel (geometry_msgs::Twist)
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
        rospy.init_node('vel_splitter', anonymous=True)
        self.is_following_started = False

        # Topic name to subscribe
        self.desired_cartesian_cmd_vel_topic_name = rospy.get_param("~desired_cartesian_cmd_vel_topic_name", "desired_vel")

        # Topic name to publish
        self.robot_cartesian_cmd_vel_topic_name = rospy.get_param("~robot_cartesian_cmd_vel_topic_name", "j2n6s300_driver/in/cartesian_velocity")
        self.base_cartesian_cmd_vel_topic_name = rospy.get_param("~base_cartesian_cmd_vel_topic_name", "cmd_vel")
        self.sup_cartesian_cmd_vel_topic_name = rospy.get_param("~sup_cartesian_cmd_vel_topic_name", "sup_vel")

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0) # 100Hz needed for kinova arm
        self.rate = rospy.Rate(self.pub_rate)

        # Publisher
        self.pub_robot_vel_cmd = rospy.Publisher(self.robot_cartesian_cmd_vel_topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)
        self.pub_base_vel_cmd = rospy.Publisher(self.base_cartesian_cmd_vel_topic_name, geometry_msgs.msg.Twist,queue_size=1)
        self.pub_sup_vel_cmd = rospy.Publisher(self.sup_cartesian_cmd_vel_topic_name, geometry_msgs.msg.Twist,queue_size=1)

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

        # TF2 broadcaster (for showing puposes)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster

        self.initial_time = rospy.Time.now().to_sec()

        # Start control (subscriber)
        # rospy.Timer(rospy.Duration(1.00/self.pub_rate), self.followJoint)
        self.sub_desired_vel = rospy.Subscriber(self.desired_cartesian_cmd_vel_topic_name, geometry_msgs.msg.Twist, self.splitVel_cb, queue_size=1)


    def splitVel_cb(self, cmd_msg):

        des_cmd = np.array([cmd_msg.linear.x,cmd_msg.linear.y,cmd_msg.linear.z,cmd_msg.angular.x,cmd_msg.angular.y,cmd_msg.angular.z])
        
        arm_cmd, sup_cmd, base_cmd = self.splitLaw(des_cmd)


    def splitLaw(self, des_vel):
        """
        Input: 
            des_vel: [linear_x,linear_y,linear_z,angular_x,angular_y,angular_z]
        Output:
            arm_cmd: Robot arm cartesian cmd (in the arm base frame)
            [linear_x,linear_y,linear_z,angular_x,angular_y,angular_z] (np.array)
            sup_cmd: Support height velocity (in the base frame. Note: it's 1DOF)
            linear_z (float)
            base_cmd: Mobile base velocity (in the base frame)
            [linear_x,linear_y,angular_z] (np.array)
        """
        pass

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