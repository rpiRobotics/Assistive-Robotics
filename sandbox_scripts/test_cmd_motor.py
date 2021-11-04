#!/usr/bin/env python
import rospy
from swarm_msgs.msg import MotorCmd


import threading

class CmdMotorTalker():
    def __init__(self):
        rospy.init_node('cmd_motor_talker', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher("/oarbot_blue/motor_command", MotorCmd, queue_size=1)
        self.direction = 1

        rospy.Timer(rospy.Duration(0.01), self.talk)
        rospy.Timer(rospy.Duration(0.5), self.switch)

    def talk(self,event):    
        motor_feedback_msg = MotorCmd()
        motor_feedback_msg.v_fl = 1.0
        motor_feedback_msg.v_fr = 0.0
        motor_feedback_msg.v_bl = 0.0
        motor_feedback_msg.v_br = 0.0
        self.cmd_vel_pub.publish(motor_feedback_msg)

    def switch(self,event):
        self.direction *= 1 


if __name__ == "__main__":
    CmdMotorTalker = CmdMotorTalker()
    rospy.spin()