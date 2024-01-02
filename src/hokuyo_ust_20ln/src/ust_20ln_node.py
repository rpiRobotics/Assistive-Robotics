#!/usr/bin/env python

from __future__ import absolute_import

import rospy
from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Float32

from hokuyo import UST20LN_Handler
from hokuyo import ust_commands as cmds
import numpy as np

from rospy_log_controller import LogController

class UST20LN_Node():
    def __init__(self):
        rospy.init_node('ust_20ln_node', anonymous=True)

        self.logger = LogController() # To Manage the maximum rate of rospy log data

        self.serial_port        = rospy.get_param("~serial_port",       "/dev/ttyACM0")
        
        self.laser_scan_topic_name = rospy.get_param('~laser_scan_topic_name', "/scan")
        self.tf_frame_id        = rospy.get_param("~frame_id",          "laser"       )
        
        self.publish_intensity  = rospy.get_param("~publish_intensity", False )
        
        self.angle_min          = rospy.get_param("~angle_min",         np.deg2rad(-135.)  )
        self.angle_max          = rospy.get_param("~angle_max",         np.deg2rad(135.)   )
        self.angle_increment    = rospy.get_param("~angle_max",         np.deg2rad(0.25)   )

        self.scan_time = rospy.get_param("~scan_time", 0.025)

        self.range_max = rospy.get_param("~range_max", 22.0 )
        self.range_min = rospy.get_param("~range_min", 0.02 )


        self.laser_scan_pub = rospy.Publisher(self.laser_scan_topic_name, LaserScan, queue_size=3)

        self.is_scanned_ever = False

        # connection to Hokuyo Laser scanner and start ranging
        self.connect_Hokuyo()
        rospy.Timer(rospy.Duration(0.025), self.get_scan_data)

    def connect_Hokuyo(self):
        self.scanner = UST20LN_Handler()
        self.scanner.connect(self.serial_port)
        ret = self.scanner.send_command(cmds.ID)
        rospy.loginfo("Hokuyo info: " + str(ret.decode()))
        self.scanner.start_ranging()
        

    def get_scan_data(self,event):    
        try:
            data = self.scanner.get_measures()
            if len(data[1]) == cmds.MEASURE_STEPS_UST_20LN : # Number of steps for measurements for 05LN = 541, for 20LN = 1081
                # rospy.loginfo("ok at {:.06f} s".format(data[0]))
            
                ranges = [x[0]/1000.0 for x in data[1]] # in meters
                intensities = [x[1] for x in data[1]] 

                laser_scan_msg = LaserScan()
                laser_scan_msg.header.stamp = rospy.Time.now()
                laser_scan_msg.header.frame_id = self.tf_frame_id

                laser_scan_msg.angle_min = self.angle_min
                laser_scan_msg.angle_max = self.angle_max
                laser_scan_msg.angle_increment = self.angle_increment

                laser_scan_msg.scan_time = self.scan_time
                laser_scan_msg.time_increment = self.scan_time/360.0

                laser_scan_msg.range_max = self.range_max
                laser_scan_msg.range_min = self.range_min

                laser_scan_msg.ranges = ranges

                if self.publish_intensity:
                    laser_scan_msg.intensities = intensities


                self.laser_scan_pub.publish(laser_scan_msg)
            else:
                # rospy.logwarn("Hokuyo: measured number of steps less than the specified number" )
                self.logger.log("Hokuyo: measured number of steps less than the specified number",
                                log_type='warning', 
                                min_period=2.0)
        except Exception as ex:
            template = "An exception of type {0} occurred while reading the laser scan measurements. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            # rospy.logwarn(message)
            self.logger.log(message,
                            log_type='error', 
                            min_period=2.0)
            

        


if __name__ == "__main__":
    UST20LN_node = UST20LN_Node()
    rospy.spin()