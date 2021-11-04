#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import serial
import time


class arduinoread:
    def __init__(self):
        rospy.init_node('arduino_talker', anonymous=True)
        self.arduino_deadman_switch_topic=rospy.get_param('~arduino_deadman_switch_topic')
        self.arduino_e_stop_topic=rospy.get_param('~arduino_e_stop_topic')
        
        self.comport=rospy.get_param('~com_port')
        
        #self.ser = serial.Serial(self.comport,115200,timeout=0.04)
        self.ser = None
        
        self.deadman_switch_pub = rospy.Publisher(self.arduino_deadman_switch_topic, Bool, queue_size=1)
        self.e_stop_pub = rospy.Publisher(self.arduino_e_stop_topic, Bool, queue_size=1)
        time.sleep(1)
        rospy.Timer(rospy.Duration(0.04),self.read_serial)


    def read_serial(self,event):
        if not rospy.is_shutdown():
            try:
                if(self.ser == None):
                    rospy.loginfo("Trying to reconnect to serial")
                    self.ser = serial.Serial(self.comport,115200,timeout=0.04)
                    rospy.loginfo("Connected to serial")

                self.handle_serial_data(self.ser.read())
                self.ser.reset_input_buffer()
            except serial.serialutil.SerialException:
                if(not(self.ser == None)):
                    self.ser.close()
                    self.ser = None
                    rospy.logwarn("Disconnecting from serial")
                rospy.logwarn("Serial disconnected")
        else:
            rospy.signal_shutdown("system shutdown")

    def handle_serial_data(self, raw_serial_data):
        try:
            output = int(raw_serial_data.decode('utf-8'))
        except:
            rospy.logwarn("Garbage serial data")
            return
        mes_deadman_switch = Bool()
        mes_e_stop = Bool()

        if(output == 0):
            mes_e_stop.data = 0
            mes_deadman_switch.data = 0
        if(output == 1):
            mes_e_stop.data = 0
            mes_deadman_switch.data = 1
        if(output == 2):
            mes_e_stop.data = 1
            mes_deadman_switch.data = 0
        if(output == 3):
            mes_e_stop.data = 1
            mes_deadman_switch.data = 1


        self.deadman_switch_pub.publish(mes_deadman_switch)
        self.e_stop_pub.publish(mes_e_stop)


if __name__ == '__main__':
    arduinoreader=arduinoread()
    rospy.spin()