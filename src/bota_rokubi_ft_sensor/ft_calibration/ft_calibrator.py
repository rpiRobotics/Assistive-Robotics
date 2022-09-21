#!/usr/bin/env python  

"""
Author: Burak Aksoy
Description:

"""

import rospy
import actionlib
# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import kinova_msgs.msg
import sensor_msgs.msg

import argparse
import sys
import pandas as pd
import math
import os
import numpy as np

import time

def main():
    rospy.init_node('kinova_ft_calibrator', anonymous=True)

    file_name = "./ft_calibration_joint_poses_right2.csv"
    # read csv file
    file_name = os.path.expanduser(file_name)
    df = pd.read_csv(file_name)
    joint_poses_all = df.values.tolist()

    # action_address = '/oarbot_silver/j2n6s300_left_driver/joints_action/joint_angles' # For oarbot silver, left
    action_address = '/oarbot_blue/j2n6s300_right_driver/joints_action/joint_angles' # For oarbot blue, right

    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmJointAnglesAction)

    i = 0
    speed = 6 # a number to execute a pose every that location 
    for joint_pose in joint_poses_all:
        if (i % speed == 0):
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

            print("Sent to the joint goal, saving data..")
            time.sleep(0.25)
            print("Data is saved. Press any key to continue..")
        i = i + 1

    
    print("Calibration process ended successfully. Saved file at: ....")


if __name__ == '__main__':
    main()