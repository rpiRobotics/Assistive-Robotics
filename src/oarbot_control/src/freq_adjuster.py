#!/usr/bin/env python

"""
Author: Burak Aksoy
Node: freq_adjuster
Description:
    A helper node to oarbot_redundancy_resolver to control the message types and rates
    for the arm and the base on oarbot.
    Needs to be spawned two times: one for the mobile base one for the arm.

    This node takes a velocity twist command for the arm of oarbot and converts it
    into a arm type velocity command, with the desired frequency. Also considers the 
    timeout frequency to stop publishing messages.
    
    Its idea is very similar to ROS's topic_tools/transform package, but differs from it since
    this node can emulate a publisher with a faster rate from a slower rate input (like zero order hold).
    However, this node cannot handle any types of messages, currently only designed for 
    geometry_msgs::Twist and kinova_msgs::CartesianVelocity.

Parameters:
    - in_cmd_vel_topic_name
    - in_cmd_vel_topic_type
    - out_cmd_vel_topic_name
    - out_cmd_vel_topic_type
    - out_rate    
    - in_rate
Subscribes to:
    - /$(robot)/cmd_vel_in (eg. geometry_msgs::Twist)
Publishes to:
    - /$(robot)/cmd_vel_out (eg. /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::CartesianVelocity))
"""

import rospy

import geometry_msgs.msg # for Twist
import kinova_msgs.msg  # for PoseVelocity ,JointVelocity

class FreqAdjuster():
    def __init__(self):

        rospy.init_node('freq_adjuster', anonymous=True)

        # Published topic names 
        self.out_cmd_vel_topic_name = rospy.get_param("~out_cmd_vel_topic_name")
        # Publishing msg type (either Geometry_msgs::Twist or kinova_msgs::PoseVelocity)
        self.out_cmd_vel_topic_type = rospy.get_param("~out_cmd_vel_topic_type", "geometry_msgs.msg.Twist")

        # Publishers
        if self.out_cmd_vel_topic_type == "geometry_msgs.msg.Twist":
            self.pub_cmd_vel = rospy.Publisher(self.out_cmd_vel_topic_name, geometry_msgs.msg.Twist, queue_size=1)
        elif self.out_cmd_vel_topic_type == "kinova_msgs.msg.PoseVelocity":
            self.pub_cmd_vel = rospy.Publisher(self.out_cmd_vel_topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)

        # Subscribed topic names
        self.in_cmd_vel_topic_name = rospy.get_param("~in_cmd_vel_topic_name", "/cmd_vel")
        # Subscribed msg type (either Geometry_msgs::Twist or kinova_msgs::PoseVelocity)
        self.in_cmd_vel_topic_type = rospy.get_param("~in_cmd_vel_topic_type", "geometry_msgs.msg.Twist")
        
        # Subscribers
        if self.in_cmd_vel_topic_type == "geometry_msgs.msg.Twist":
            self.sub_cmd_vel = rospy.Subscriber(self.in_cmd_vel_topic_name, geometry_msgs.msg.Twist, self.geometry_msgs_Twist_callback, queue_size=1)
        elif self.in_cmd_vel_topic_type == "kinova_msgs.msg.PoseVelocity":
            self.sub_cmd_vel = rospy.Subscriber(self.in_cmd_vel_topic_name, kinova_msgs.msg.PoseVelocity, self.kinova_msgs_PoseVelocity_callback, queue_size=1)

        self.Vx = None
        self.Vy = None
        self.Vz = None
        self.Wx = None
        self.Wy = None
        self.Wz = None

        # Publish rate arm
        self.out_rate = rospy.get_param("~out_rate", 100.0) # 100Hz needed for kinova arm
        self.in_rate = rospy.get_param("in_rate",25.0) # Minimum expected rate of input cmd
        
        self.is_zero_cmd_sent_ever = False
        self.velocity_command_sent = True
        
        self.time_last_cmd_vel = 0.0

        self.cmd_wait_timeout = 1.00/self.in_rate

        # Start publishing
        if self.out_cmd_vel_topic_type == "geometry_msgs.msg.Twist":
            rospy.Timer(rospy.Duration(1.00/self.out_rate), self.geometry_msgs_Twist_command)
        elif self.out_cmd_vel_topic_type == "kinova_msgs.msg.PoseVelocity":
            rospy.Timer(rospy.Duration(1.00/self.out_rate), self.kinova_msgs_PoseVelocity_command)


    def geometry_msgs_Twist_callback(self, msg):
        self.Vx = msg.linear.x
        self.Vy = msg.linear.y
        self.Vz = msg.linear.z
        self.Wx = msg.angular.x
        self.Wy = msg.angular.y
        self.Wz = msg.angular.z
        
        self.time_last_cmd_vel = rospy.Time.now().to_sec()
        self.velocity_command_sent = False

    def kinova_msgs_PoseVelocity_callback(self, msg):
        self.Vx = msg.twist_linear_x
        self.Vy = msg.twist_linear_y
        self.Vz = msg.twist_linear_z
        self.Wx = msg.twist_angular_x
        self.Wy = msg.twist_angular_y
        self.Wz = msg.twist_angular_z
        
        self.time_last_cmd_vel = rospy.Time.now().to_sec()
        self.velocity_command_sent = False
        

    def geometry_msgs_Twist_command(self, event=None):
        # If the velocity command for arm is sent and the time's been past more than the timeout amount
        if self.velocity_command_sent and (rospy.Time.now().to_sec() - self.time_last_cmd_vel > self.cmd_wait_timeout):
            # send zero velocity command (NOt necessary, just dont send anything)

            if not self.is_zero_cmd_sent_ever:
                rospy.logwarn_once("frequency_adjuster node: Zero velocities are sent for the first time")
                self.is_zero_cmd_sent_ever = True
        else:
            # send the latest velocity command
            # Generate and publish the Twist message
            cmd_vel = geometry_msgs.msg.Twist()
            cmd_vel.linear.x = self.Vx
            cmd_vel.linear.y = self.Vy
            cmd_vel.linear.z = self.Vz 
            cmd_vel.angular.x = self.Wx 
            cmd_vel.angular.y = self.Wy  
            cmd_vel.angular.z = self.Wz 
            self.pub_cmd_vel.publish(cmd_vel)

            self.velocity_command_sent = True    
            self.is_zero_cmd_sent_ever = False

    def kinova_msgs_PoseVelocity_command(self, event=None):
        # If the velocity command for arm is sent and the time's been past more than the timeout amount
        if self.velocity_command_sent and (rospy.Time.now().to_sec() - self.time_last_cmd_vel > self.cmd_wait_timeout):
            # send zero velocity command (NOt necessary, just dont send anything)

            if not self.is_zero_cmd_sent_ever:
                rospy.logwarn_once("frequency_adjuster node: Zero velocities are sent for the first time")
                self.is_zero_cmd_sent_ever = True
        else:
            # send the latest velocity command
            # Generate and publish the Twist message
            cmd_vel = kinova_msgs.msg.PoseVelocity()
            cmd_vel.twist_linear_x = self.Vx
            cmd_vel.twist_linear_y = self.Vy
            cmd_vel.twist_linear_z = self.Vz 
            cmd_vel.twist_angular_x = self.Wx 
            cmd_vel.twist_angular_y = self.Wy  
            cmd_vel.twist_angular_z = self.Wz 
            self.pub_cmd_vel.publish(cmd_vel)

            self.velocity_command_sent = True    
            self.is_zero_cmd_sent_ever = False

if __name__ == "__main__":
    freqAdjuster = FreqAdjuster()
    rospy.spin()
