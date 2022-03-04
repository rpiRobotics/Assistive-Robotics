#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from assistive_msgs.msg import MotorCmd
import math
import numpy as np

class OarbotControl_FwdKin():
    def __init__(self):
        rospy.init_node('oarbot_ctrl_fwd_kin', anonymous=True)

        self.vel_feedback_name=rospy.get_param('~velocity_feedback_topic_name')
        self.motor_feedback_topic_name = rospy.get_param('~motor_feedback_topic_name')
        
        self.l_x        = rospy.get_param("~l_x") # meter
        self.l_y        = rospy.get_param("~l_y") # meter
        self.r          = rospy.get_param("~radius_wheel") # meter
        self.total_gear_ratio  = rospy.get_param("~total_gear_ratio") # rad/s
        self.skid_steer_mode = rospy.get_param("~skid_steer_mode", False) # boolean
        self.x_vel_scale = rospy.get_param("~x_vel_scaling")
        self.y_vel_scale = rospy.get_param("~y_vel_scaling")
        self.th_vel_scale = rospy.get_param("~th_vel_scaling")

        self.covariance_diagonal = rospy.get_param("~twist_covariance_diagonal", [1.0e-8, 1.0e-8, 1.0e-8, 0., 0., 25.0e-10]) # [x_dot,y_dot,z_dot,rotx_dot,roty_dot,rotz_dot]
        self.covariance = np.zeros(36)
        self.covariance[0] =  self.covariance_diagonal[0] # x
        self.covariance[7] =  self.covariance_diagonal[1] # y
        self.covariance[14] =  self.covariance_diagonal[2] # z
        self.covariance[21] =  self.covariance_diagonal[3] # rot x
        self.covariance[28] =  self.covariance_diagonal[4] # rot y
        self.covariance[35] =  self.covariance_diagonal[5] # rot z
        self.covariance = list(self.covariance) # convert to list of 36 floats

        self.tf_mobile_base_frame_id = rospy.get_param("~tf_mobile_base_frame_id", "oarbot_blue_base")


        self.vel_pub = rospy.Publisher(self.vel_feedback_name, TwistWithCovarianceStamped, queue_size=1)
        rospy.Subscriber(self.motor_feedback_topic_name, MotorCmd, self.motor_feedback_callback, queue_size=1)

    def motor_feedback_callback(self, msg):
        if self.skid_steer_mode:
            self.forward_kin_skid_steer(msg)
        else:
            self.forward_kin(msg)

    def forward_kin(self,msg):    
        # rospy.loginfo("I am in omni directional mode")
        # Convert from RPM to rad/s
        v_fl = msg.v_fl/60*2*math.pi # / 200.0
        v_fr = msg.v_fr/60*2*math.pi # / 200.0
        v_bl = msg.v_bl/60*2*math.pi # / 200.0
        v_br = msg.v_br/60*2*math.pi # / 200.0

        # Generate and publish the Twist message
        vel_feedback = Twist()
        vel_feedback.linear.x = (self.r/4 * (v_fl + v_fr + v_bl + v_br)) / self.total_gear_ratio
        vel_feedback.linear.y = (self.r/4 * (-v_fl + v_fr + v_bl - v_br)) / self.total_gear_ratio
        vel_feedback.angular.z = ((self.r/(4*(self.l_x + self.l_y))) * (-v_fl + v_fr - v_bl + v_br)) / self.total_gear_ratio
        
        # Scaling
        vel_feedback.linear.x /= self.x_vel_scale
        vel_feedback.linear.y /= self.y_vel_scale
        vel_feedback.angular.z /= self.th_vel_scale

        # publish the twist with a stamped covariance
        vel_feedback_covariance_stamped = TwistWithCovarianceStamped()

        vel_feedback_covariance_stamped.header.stamp = rospy.Time.now()
        vel_feedback_covariance_stamped.header.frame_id = self.tf_mobile_base_frame_id

        vel_feedback_covariance_stamped.twist.twist = vel_feedback
        vel_feedback_covariance_stamped.twist.covariance = self.covariance

        self.vel_pub.publish(vel_feedback_covariance_stamped)

    def forward_kin_skid_steer(self,msg):    
        # rospy.loginfo("I am in skid steer")
        # Convert from RPM to rad/s
        v_fl = msg.v_fl/60*2*math.pi
        v_fr = msg.v_fr/60*2*math.pi
        v_bl = msg.v_bl/60*2*math.pi
        v_br = msg.v_br/60*2*math.pi

        # Generate and publish the Twist message
        vel_feedback = Twist()
        vel_feedback.linear.x = (self.r/4 * (v_fl + v_fr + v_bl + v_br)) / self.total_gear_ratio
        vel_feedback.linear.y = 0.0
        vel_feedback.angular.z = (self.r/(4*self.l_y) * (-v_fl + v_fr - v_bl + v_br)) / self.total_gear_ratio

        # Scaling
        vel_feedback.linear.x /= self.x_vel_scale
        vel_feedback.linear.y /= self.y_vel_scale
        vel_feedback.angular.z /= self.th_vel_scale

        # publish the twist with a stamped covariance
        vel_feedback_covariance_stamped = TwistWithCovarianceStamped()

        vel_feedback_covariance_stamped.header.stamp = rospy.Time.now()
        vel_feedback_covariance_stamped.header.frame_id = self.tf_mobile_base_frame_id

        vel_feedback_covariance_stamped.twist.twist = vel_feedback
        vel_feedback_covariance_stamped.twist.covariance = self.covariance

        self.vel_pub.publish(vel_feedback_covariance_stamped)

if __name__ == "__main__":
    oarbotControl_FwdKin = OarbotControl_FwdKin()
    rospy.spin()
