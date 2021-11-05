#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from assistive_msgs.msg import MotorCmd
import math

class OarbotControl_InvKin():
    def __init__(self):
        rospy.init_node('oarbot_ctrl_inv_kin', anonymous=True)

        self.motor_command_name=rospy.get_param('~motor_command_topic_name')
        self.teleop_command_name=rospy.get_param('~republished_spacemouse')

        self.l_x        = rospy.get_param("~l_x") # meter
        self.l_y        = rospy.get_param("~l_y") # meter
        self.r          = rospy.get_param("~radius_wheel") # meter
        self.total_gear_ratio  = rospy.get_param("~total_gear_ratio") # rad/s
        self.skid_steer_mode = rospy.get_param("~skid_steer_mode", False) # boolean
        self.x_vel_scale = rospy.get_param("~x_vel_scaling")
        self.y_vel_scale = rospy.get_param("~y_vel_scaling")
        self.th_vel_scale = rospy.get_param("~th_vel_scaling")
 
        self.motor_cmd_pub = rospy.Publisher(self.motor_command_name, MotorCmd, queue_size=1)
        rospy.Subscriber(self.teleop_command_name, Twist, self.callback, queue_size=1)

    def callback(self, msg):
        
        msg.linear.x *= self.x_vel_scale
        msg.linear.y *= self.y_vel_scale
        msg.angular.z *= self.th_vel_scale

        if self.skid_steer_mode:
            self.inverse_kin_skid_steer(msg)
        else:
            self.inverse_kin(msg)

    def inverse_kin(self, msg):
        v_lin = msg.linear
        v_ang = msg.angular
        
        v_fl = 1/self.r * (v_lin.x - v_lin.y - (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        # angular velocity of front right motor
        v_fr = 1/self.r * (v_lin.x + v_lin.y + (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        # angular velocity of rear right motor
        v_bl = 1/self.r * (v_lin.x + v_lin.y - (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        # angular velocity of rear left motor
        v_br = 1/self.r * (v_lin.x - v_lin.y + (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        
        # Convert from rad/s to RPM
        v_fl = v_fl*60/(2*math.pi)
        v_fr = v_fr*60/(2*math.pi)
        v_bl = v_bl*60/(2*math.pi)
        v_br = v_br*60/(2*math.pi)

        # Generate and publish the MotorCmd message
        motor_cmd = MotorCmd()
        motor_cmd.v_fl = int(v_fl) #* 200.0
        motor_cmd.v_fr = int(v_fr) # *200.0
        motor_cmd.v_bl = int(v_bl) # *200.0
        motor_cmd.v_br = int(v_br)# *200.0
        rospy.loginfo("1111111111111111111111111111111111111111111111")

        try:
            self.motor_cmd_pub.publish(motor_cmd)
        except Exception as e:
            rospy.logerr(str(e))
        rospy.loginfo("2222222222222222222222222222222222222222222222222")

    def inverse_kin_skid_steer(self, msg):
        v_lin = msg.linear
        v_ang = msg.angular
        
        v_fl = 1/self.r * (v_lin.x  - self.l_y * v_ang.z) * self.total_gear_ratio
        # angular velocity of front right motor
        v_fr = 1/self.r * (v_lin.x  + self.l_y * v_ang.z) * self.total_gear_ratio
        # angular velocity of rear right motor
        v_bl = 1/self.r * (v_lin.x  - self.l_y * v_ang.z) * self.total_gear_ratio
        # angular velocity of rear left motor
        v_br = 1/self.r * (v_lin.x  + self.l_y * v_ang.z) * self.total_gear_ratio
        
        # Convert from rad/s to RPM
        v_fl = v_fl*60/(2*math.pi)
        v_fr = v_fr*60/(2*math.pi)
        v_bl = v_bl*60/(2*math.pi)
        v_br = v_br*60/(2*math.pi)

        # Generate and publish the MotorCmd message
        motor_cmd = MotorCmd()
        motor_cmd.v_fl = v_fl
        motor_cmd.v_fr = v_fr
        motor_cmd.v_bl = v_bl
        motor_cmd.v_br = v_br

        self.motor_cmd_pub.publish(motor_cmd)
        
if __name__ == "__main__":
    oarbotControl_InvKin = OarbotControl_InvKin()
    rospy.spin()