#!/usr/bin/env python  

"""
Author: Burak Aksoy
Description:

"""

from itertools import cycle
import rospy
import actionlib
# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import kinova_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

import argparse
import sys
import pandas as pd
import math
import os
import numpy as np

import time

class FTCalib():
    def __init__(self):
        self.H = None # stacked measurement matrices
        self.Z = None # stacked F/T measurements
        self.num_measure = 0 # number of stacked measurements

    def get_calib(self):
        """
        Least squares to estimate the F/T sensor parameters
        The estimated parameters are :
        [m m*cx m*cy m*cz FBx FBy FBz TBx TBy TBz]
        m: mass of the gripper
        [cx, cy, cz] are the coordinates of the center of mass of the gripper
        FB : force bias
        TB: torque bias
        All expressed in the FT sensor frame
        """

        # H @ ft_calib_params = Z
        ft_calib_params = np.linalg.lstsq(self.H, self.Z)
        return ft_calib_params

    def add_measurement(self, gravity, ft_raw):
        """
        adds a F/T measurement and the corresponding measurement matrix from the gravity
        measurements of the accelerometer
        gravity is assumed to be expressed in the F/T sensor frame
        """
        
        self.num_measure = self.num_measure + 1
        h = self.get_measurement_mat(gravity)
        z = -ft_raw

        # print(str(h))
        # print(str(z))

        if self.num_measure==1:
            self.H = h
            self.Z = z
        else:
            # append the new h and z to existing matrices
            self.H = np.vstack((self.H,h))
            self.Z = np.hstack((self.Z,z))
        # print(str(self.H))
        # print(str(self.Z))    
        return


    def get_measurement_mat(self,gravity):
        """
        measurement matrix based on "On-line Rigid Object Recognition and Pose Estimation
	    Based on Inertial Parameters", D. Kubus, T. Kroger, F. Wahl, IROS 2008
        """
        H = np.zeros((6,10))
        g = gravity

        w = np.zeros(3)
        alpha = np.zeros(3)
        a = np.zeros(3)

        for i in range(3):
            for j in range(4,10):
                if(i==j-4):
                    H[i,j] = 1.0
                else:
                    H[i,j] = 0.0

        for i in range(3,6):
            H[i,0] = 0.0

        H[3,1] = 0.0
        H[4,2] = 0.0
        H[5,3] = 0.0

        for i in range(3):
            H[i,0] = a[i] - g[i]

        H[0,1] = -w[1]*w[1] - w[2]*w[2]
        H[0,2] = w[0]*w[1] - alpha[2]
        H[0,3] = w[0]*w[2] + alpha[1]

        H[1,1] = w[0]*w[1] + alpha[2]
        H[1,2] = -w[0]*w[0] - w[2]*w[2]
        H[1,3] = w[1]*w[2] - alpha[0]

        H[2,1] = w[0]*w[2] - alpha[1]
        H[2,2] = w[1]*w[2] + alpha[0]
        H[2,3] = -w[1]*w[1] - w[0]*w[0]

        H[3,2] = a[2] - g[2]
        H[3,3] = -a[1] + g[1]

        H[4,1] = -a[2] + g[2]
        H[4,3] = a[0] - g[0]

        H[5,1] = a[1] - g[1]
        H[5,2] = -a[0] + g[0]

        for i in range(3,6):
            for j in range(4,10):
                if(i==(j-4)):
                    H[i,j] = 1.0
                else:
                    H[i,j] = 0.0
        
        return H


class ROS_imu_subscriber():
    def __init__(self,ros_topic):

        # rospy.init_node('ros_imu_subscriber', anonymous=True)
    
        # topic name that this node subscribes
        self.topic_name = rospy.get_param('~topic_name', ros_topic)
        self.timeout = 1 # seconds to wait for a msg

        self.num_for_avr = 50 # takes 100 imu data to take the avr.

    def get_latest_gravity(self):
        success = False
        while (not success) and (not rospy.is_shutdown()):
            try:
                # read the imu msg from the topic:
                msg = rospy.wait_for_message(self.topic_name, sensor_msgs.msg.Imu, timeout=self.timeout)
                # Convert a ROS imu message to a gravity vector reading from acceleration
                gravity = np.zeros(3)
                gravity[0] = -msg.linear_acceleration.x # IMU will measure gravity in the opposite direction from F/T sensor, check https://github.com/kth-ros-pkg/force_torque_tools/pull/18
                gravity[1] = -msg.linear_acceleration.y
                gravity[2] = -msg.linear_acceleration.z

                success = True
            except:
                print("Something went wrong when reading imu msg")

        return gravity

    def get_average_gravity(self):
        gravity_avr = np.zeros(3)
        for i in range(self.num_for_avr):
            gravity_avr = gravity_avr + self.get_latest_gravity()
        gravity_avr = gravity_avr/float(self.num_for_avr)
        
        return gravity_avr

class ROS_ft_subscriber():
    def __init__(self,ros_topic):

        # rospy.init_node('ros_ft_subscriber', anonymous=True)
    
        # topic name that this node subscribes
        self.topic_name = rospy.get_param('~topic_name', ros_topic)
        self.timeout = 1 # seconds to wait for a msg

        self.num_for_avr = 50 # takes 100 ft data to take the avr.

    def get_latest_wrench(self):
        success = False
        while (not success) and (not rospy.is_shutdown()):
            try:
                # read the wrench msg from the topic:
                msg = rospy.wait_for_message(self.topic_name, geometry_msgs.msg.WrenchStamped, timeout=self.timeout)
                # Convert a ROS wrench message to a wrench vector 
                wrench = np.zeros(6) # [F;T]
                wrench[0] = msg.wrench.force.x 
                wrench[1] = msg.wrench.force.y
                wrench[2] = msg.wrench.force.z
                wrench[3] = msg.wrench.torque.x 
                wrench[4] = msg.wrench.torque.y
                wrench[5] = msg.wrench.torque.z

                success = True
            except:
                print("Something went wrong when reading wrench msg")

        return wrench

    def get_average_wrench(self):
        wrench_avr = np.zeros(6)
        for i in range(self.num_for_avr):
            wrench_avr = wrench_avr + self.get_latest_wrench()
        wrench_avr = wrench_avr/float(self.num_for_avr)

        return wrench_avr


def main():
    file_name = "./ft_calibration_joint_poses_right2.csv"
    ros_topic_imu = "/oarbot_blue/j2n6s300_right_ft_imu"
    ros_topic_ft = "/oarbot_blue/ft_sensor/j2n6s300_right/ft_sensor_readings/wrench"
    # action_address = '/oarbot_silver/j2n6s300_left_driver/joints_action/joint_angles' # For oarbot silver, left
    action_address = '/oarbot_blue/j2n6s300_right_driver/joints_action/joint_angles' # For oarbot blue, right

    # initialize ros node
    rospy.init_node('kinova_ft_calibrator', anonymous=True)

    # read csv file
    file_name = os.path.expanduser(file_name)
    df = pd.read_csv(file_name)
    joint_poses_all = df.values.tolist()
    
    # kinova action client
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmJointAnglesAction)

    ros_imu_subscriber = ROS_imu_subscriber(ros_topic_imu)
    ros_ft_subscriber = ROS_ft_subscriber(ros_topic_ft)
    ftCalib = FTCalib()

    i = 0
    speed = 12 # a number to execute a pose every that location 
    for joint_pose in joint_poses_all:
        if (i % speed == 0):
            #####################################################################
            ## GO to joint pose
            # print(np.rad2deg(joint_pose))
            joint_angles = np.rad2deg(joint_pose)

            goal = kinova_msgs.msg.ArmJointAnglesGoal()
            goal.angles.joint1 = joint_angles[0]
            goal.angles.joint2 = joint_angles[1]
            goal.angles.joint3 = joint_angles[2]
            goal.angles.joint4 = joint_angles[3]
            goal.angles.joint5 = joint_angles[4]
            goal.angles.joint6 = joint_angles[5]
            goal.angles.joint7 = 0.0

            try:
                client.wait_for_server(timeout = rospy.Duration(1.0))

                client.send_goal(goal)
                if not client.wait_for_result(rospy.Duration(10.0)):
                    rospy.logerr('the joint angle action timed-out')
                    client.cancel_all_goals()
            except:
                rospy.logerr("Somethingh went wrong while sending joint command")

            # print("Sent to the joint goal, collecting Gravity and Wrench data..")
            #####################################################################
            ## Start collecting data
            time.sleep(1) # wait one second to robot stabilizes itself.

            ## Get gravity avr at that pose
            gravity_avr = ros_imu_subscriber.get_average_gravity()

            ## Get wrench avr at that pose
            wrench_avr = ros_ft_subscriber.get_average_wrench()

            ## debug
            # print("Gravity:\n")
            # print(str(gravity_avr))
            # print("Wrench:\n")
            # print(str(wrench_avr))

            # print("Collected Gravity and Wrench data, saving data..")
            #####################################################################
            ## Save data for calibrations
            ftCalib.add_measurement(gravity_avr,wrench_avr) 

            # TODO
            
            # print("Data is saved.")
            #####################################################################
            # #Wait for the user to press a key
            # if (sys.version_info > (3, 0)):
            #     input("Press any key to continue..\n")
            # else:
            #     raw_input("Press any key to continue..")
            #####################################################################
        i = i + 1

    #####################################################################
    ## Execute calibration with saved data
    calib_results = ftCalib.get_calib() 
    ft_calib_params = calib_results[0] # 10x1 as [m m*cx m*cy m*cz FBx FBy FBz TBx TBy TBz]
    calib_residuals = calib_results[1] # 
    # print(ft_calib_params)
    print("calib_residuals: " + str(calib_residuals))

    #####################################################################
    ## Calculate parameters
    mass = ft_calib_params[0]
    if(mass<=0.0):
        print("Error in estimated mass (<= 0)")

    cx = ft_calib_params[1]/mass
    cy = ft_calib_params[2]/mass
    cz = ft_calib_params[3]/mass
    COM_pos = np.array([cx,cy,cz]) # wrt wrench frame

    f_bias = np.zeros(3)
    f_bias[0] = -ft_calib_params[4]
    f_bias[1] = -ft_calib_params[5]
    f_bias[2] = -ft_calib_params[6]
    
    t_bias = np.zeros(3)
    t_bias[0] = -ft_calib_params[7]
    t_bias[1] = -ft_calib_params[8]
    t_bias[2] = -ft_calib_params[9]

    #####################################################################
    ## Print parameters
    print("mass: " + str(mass))    
    print("COM_pos: " + str(COM_pos))
    print("F_bias: " + str(f_bias))
    print("T_bias: " + str(t_bias))    

    #####################################################################
    ## Export the calibration data
    # TODO
    print("Calibration process ended successfully. You need to save these values to your ft calib data / gravity compensation parameter file..")

if __name__ == '__main__':
    main()