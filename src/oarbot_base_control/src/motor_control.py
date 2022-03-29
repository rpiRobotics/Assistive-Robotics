#!/usr/bin/env python

from __future__ import absolute_import

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 # For battery voltages
from assistive_msgs.msg import MotorStatus, MotorCmd
from roboteq import RoboteqHandler
from roboteq import roboteq_commands as cmds
import threading

class OarbotControl_Motor():
    def __init__(self):
        self.last_vel_lock = threading.Lock()
        rospy.init_node('oarbot_ctrl_motor', anonymous=True)

        self.serial_front = rospy.get_param('~serial_front')
        self.serial_back = rospy.get_param('~serial_back')
        self.motor_command_topic_name = rospy.get_param('~motor_command_topic_name')
        self.motor_feedback_name = rospy.get_param('~motor_feedback_topic_name')
        self.battery_voltage_f_name = rospy.get_param('~battery_voltage_f_topic_name')
        self.battery_voltage_b_name = rospy.get_param('~battery_voltage_b_topic_name')

        self.stat_flag_name = rospy.get_param('~stat_flag_topic_name')

        rospy.Subscriber(self.motor_command_topic_name, MotorCmd, self.motor_cmd_callback, queue_size=1)
        self.motor_feedback_pub = rospy.Publisher(self.motor_feedback_name, MotorCmd, queue_size=1)

        self.voltage_f_pub = rospy.Publisher(self.battery_voltage_f_name, Float32, queue_size=1)
        self.voltage_b_pub = rospy.Publisher(self.battery_voltage_b_name, Float32, queue_size=1)

        self.stat_flag_pub = rospy.Publisher(self.stat_flag_name, MotorStatus, queue_size=1)

        self.is_zero_cmd_vel_sent_ever = False

        # connection to Roboteq motor controller
        self.connect_Roboteq_controller()

        self.velocity_command_sent = True
        self.time_last_motor_cmd = 0.0

        self.motor_control_rate = rospy.get_param('~motor_control_rate')
        self.motor_cmd_wait_timeout = 4.0*(1.0/self.motor_control_rate) #seconds
        rospy.Timer(rospy.Duration(1.0/self.motor_control_rate), self.motor_feedback)


        ########## for debug
        # self.last_sub_time = rospy.Time.now().to_sec()

    def connect_Roboteq_controller(self):
        self.controller_f = RoboteqHandler()
        self.controller_b = RoboteqHandler() 
        self.connected_f = self.controller_f.connect(self.serial_front)
        self.connected_b = self.controller_b.connect(self.serial_back)

    def motor_cmd_callback(self, msg):
        with self.last_vel_lock:
            self.motor_cmd_msg = msg
            self.velocity_command_sent = False

            self.time_last_motor_cmd = rospy.Time.now().to_sec()
        
        # sub_time = rospy.Time.now().to_sec()
        # rospy.loginfo(str(sub_time - self.last_sub_time) + " motor_cmd period")
        # self.last_sub_time = sub_time

        
    def format_speed(self, speed_message):
        # Formats the speed message (RPM) obtained from roboteq driver into float
        try:
            rpm = speed_message.split('=')
            assert rpm[0] == 'S'# or rpm[0] == '?s' # To make sure that is a speed reading
            return float(rpm[1])
        except Exception as e:
            rospy.logwarn("Improper motor speed message:" + speed_message)
            raise e

    def format_voltage(self, voltage_message):
        # Formats the speed message (RPM) obtained from roboteq driver into float
        try:
            V = voltage_message.split('=')
            assert V[0] == 'V'
            return float(V[1])/10.0
        except Exception as e:
            rospy.logwarn("Improper  battery voltage message:" + voltage_message)
            raise e
            

    def format_stat_flag(self, stat_flag_message, motor_name):
        # Formats the speed message (RPM) obtained from roboteq driver into float
        try:
            fm = stat_flag_message.split('=')
            assert fm[0] == 'FM'
            if int(fm[1]) != 0:
                rospy.logwarn("Status flag is non zero, it is:" + str(int(fm[1])) + ", on motor: " + motor_name )
            return int(fm[1])
        except Exception as e:
            rospy.logwarn("Improper status flag message:" + stat_flag_message)
            raise e

    def motor_feedback(self,event):    
        # Execute the motor velocities 
        with self.last_vel_lock:
            if self.velocity_command_sent and (rospy.Time.now().to_sec() - self.time_last_motor_cmd > self.motor_cmd_wait_timeout):
                self.controller_f.send_command(cmds.DUAL_DRIVE, 0.0, 0.0)
                self.controller_b.send_command(cmds.DUAL_DRIVE, 0.0, 0.0)
                
                if self.is_zero_cmd_vel_sent_ever == False:
                    rospy.logwarn("Zero velocities to the motors are sent for the first time")
                    self.is_zero_cmd_vel_sent_ever = True
            else:
                # self.controller_f.send_command(cmds.DUAL_DRIVE, self.motor_cmd_msg.v_fr, -self.motor_cmd_msg.v_fl)
                # self.controller_b.send_command(cmds.DUAL_DRIVE, -self.motor_cmd_msg.v_bl, self.motor_cmd_msg.v_br)

                self.controller_f.send_command(cmds.SET_SPEED, 2, -self.motor_cmd_msg.v_fl) # FL
                self.controller_f.send_command(cmds.SET_SPEED, 1, self.motor_cmd_msg.v_fr) # FR
                self.controller_b.send_command(cmds.SET_SPEED, 1, -self.motor_cmd_msg.v_bl) # BL
                self.controller_b.send_command(cmds.SET_SPEED, 2, self.motor_cmd_msg.v_br) # BR
                
                self.velocity_command_sent = True
                
                self.is_zero_cmd_vel_sent_ever = False

        # Read the executed motor velocities
        try:
            motor_feedback_msg = MotorCmd()
            motor_feedback_msg.v_fl = int(-self.format_speed(self.controller_f.read_value(cmds.READ_SPEED, 2)))
            motor_feedback_msg.v_fr = int(self.format_speed(self.controller_f.read_value(cmds.READ_SPEED, 1)))
            motor_feedback_msg.v_bl = int(-self.format_speed(self.controller_b.read_value(cmds.READ_SPEED, 1)))
            motor_feedback_msg.v_br = int(self.format_speed(self.controller_b.read_value(cmds.READ_SPEED, 2)))
            self.motor_feedback_pub.publish(motor_feedback_msg)
        except Exception as ex:
            template = "An exception of type {0} occurred while reading the executed motor velocities. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            rospy.logwarn(message)

        # Read voltages of the batterties (Just in case we read from both of the drivers, they are expected to be the same)
        try:
            battery_voltage_f = Float32() 
            battery_voltage_b = Float32()
            battery_voltage_f.data = self.format_voltage(self.controller_f.read_value(cmds.READ_VOLTS, 2))
            battery_voltage_b.data = self.format_voltage(self.controller_b.read_value(cmds.READ_VOLTS, 2))
            self.voltage_f_pub.publish(battery_voltage_f)
            self.voltage_b_pub.publish(battery_voltage_b)
        except Exception as ex:
            template = "An exception of type {0} occurred while reading voltages of the batterties. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            rospy.logwarn(message)

        # Read runtime status flags
        try:
            stat_flag_msg = MotorStatus() 
            stat_flag_msg.s_fl = self.format_stat_flag(self.controller_f.read_value(cmds.READ_RUNTIME_STATUS_FLAG, 2),"FL")
            stat_flag_msg.s_fr = self.format_stat_flag(self.controller_f.read_value(cmds.READ_RUNTIME_STATUS_FLAG, 1),"FR")
            stat_flag_msg.s_bl = self.format_stat_flag(self.controller_b.read_value(cmds.READ_RUNTIME_STATUS_FLAG, 1),"BL")
            stat_flag_msg.s_br = self.format_stat_flag(self.controller_b.read_value(cmds.READ_RUNTIME_STATUS_FLAG, 2),"BR")
            self.stat_flag_pub.publish(stat_flag_msg)
        except Exception as ex:
            template = "An exception of type {0} occurred while reading runtime status flags. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            rospy.logwarn(message)



        


if __name__ == "__main__":
    oarbotControl_Motor = OarbotControl_Motor()
    rospy.spin()