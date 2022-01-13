#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: tf_overhead_camera_aruco_broadcaster
Description:
    Broadcasts to tf2 the trasformation between the world (floor plane) frame
    ''
    and the robot center frames detected through aruco-tags attached to the robots
    by reading yaml files named:
    '.yaml' for camera parameters
    '.yaml' for undistorted camera parameters and world extrinsics
    '.yaml' for and world and floor extrinsic parameters
    '.yaml' for measured aruco tag - robot pose relations
    for the pre-calculated pose btw end efector and the camera 
    'arm2camera_pose' # TODO

Parameters: # TODO
    - end_effector_pose_topic_name: topic that is the pose of end effector is published by kinova eg."/j2n6s300_driver/out/tool_pose"
    - tf_end_effector_frame_id: frame name that is the end effector frame of the Kinova Arm, eg, "j2n6s300_end_effector"
    - tf_root_camera_frame_id: frame name that is the root frame of the camera by Kinect ROS drivers, eg, "camera_base"
    - arm2camera_pose: the pre-calculated pose btw end efector and the camera with a structure of pose.orientation = [w,x,y,z] and pose.position = [x,y,z]
Subscribes to:
    - /j2n6s300_driver/out/tool_pose (geometry_msgs::PoseStamped)
Publishes to:
    - NONE
Broadcasts to:
    - tf2
"""

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
# import kinova_msgs.msg

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import argparse
import sys
import pandas as pd
import math
import os

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

class ArucoRobots2Floor():
    def __init__(self):
        rospy.init_node('tf_overhead_camera_aruco_broadcaster', anonymous=True)

        self.image_topic_name = rospy.get_param('~image_topic_name', "/rgb/image_raw")

        self.path_to_camera_parameters = rospy.get_param('~path_to_camera_parameters', 
            "~/catkin_ws_assistive/assistive_launch/vision_parameters/camera.yml")
        self.path_to_camera_parameters = os.path.expanduser(self.path_to_camera_parameters)

        self.path_to_undistorted_camera_parameters = rospy.get_param('~path_to_undistorted_camera_parameters', 
            "~/catkin_ws_assistive/assistive_launch/vision_parameters/camera_undistorted.yml")
        self.path_to_undistorted_camera_parameters = os.path.expanduser(self.path_to_undistorted_camera_parameters)

        self.path_to_extrinsic_parameters = rospy.get_param('~path_to_extrinsic_parameters', 
            "~/catkin_ws_assistive/assistive_launch/vision_parameters/camera_aruco.yml")
        self.path_to_extrinsic_parameters = os.path.expanduser(self.path_to_extrinsic_parameters)

        self.path_to_aruco_tags_info =  rospy.get_param('~path_to_aruco_tags_info', 
            "~/catkin_ws_assistive/assistive_launch/vision_parameters/aruco_tags_info3.csv")
        self.path_to_aruco_tags_info = os.path.expanduser(self.path_to_aruco_tags_info)
        
        # Read those yaml and csv files
        [self.mtx, self.dist, self.R_co, self.R_oc, self.T_co, self.T_oc] = self.load_coefficients(
            self.path_to_camera_parameters)
        [self.mtx_new, self.dist_new, self.R_co, self.R_oc, self.T_co, self.T_oc] = self.load_coefficients(
            self.path_to_undistorted_camera_parameters)
        [self.R_op, self.R_po, self.T_op, self.T_po] = self.load_coefficients_best_fit_plane(
            self.path_to_extrinsic_parameters)
        # read csv file that includes places of the aruco tags, their aruco type, ids, sizes and locations wrt their places
        self.df = pd.read_csv(self.path_to_aruco_tags_info)
        # drop rows that are on the floor so that there are only robots to be localized left
        self.df = self.df[self.df["place"]!="floor"]
        # drop duplicate rows with the same place (robot) name
        self.df = self.df.drop_duplicates(subset='place', keep="first")
        # get aruco dictionary types on the robots as a list
        self.aruco_types = list(self.df["aruco_type"].unique())
        print(self.aruco_types)

        # TODO        
        self.tf_end_effector_frame_id = rospy.get_param("~tf_end_effector_frame_id", "j2n6s300_end_effector")
        self.tf_root_camera_frame_id = rospy.get_param("~tf_root_camera_frame_id", "camera_base")

        self.bridge = CvBridge() # To convert ROS images to openCV imgs.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        rospy.Subscriber(self.image_topic_name, sensor_msgs.msg.Image, self.handle_mobile_robot_pose)

    def load_coefficients(self,path):
        """ Loads camera matrix and distortion coefficients. """
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode("K").mat()
        dist_matrix = cv_file.getNode("D").mat()

        try:
            R_co = cv_file.getNode("R_co").mat()
            R_oc = cv_file.getNode("R_oc").mat()
            T_co = cv_file.getNode("T_co").mat()
            T_oc = cv_file.getNode("T_oc").mat()
        except:
            print("[INFO]: could not read R_co, R_oc, T_co, T_oc from: {}".format(path))
            print(str(R_co), str(R_oc), str(T_co), str(T_oc))
            cv_file.release()
            return [camera_matrix, dist_matrix]

        cv_file.release()
        return [camera_matrix, dist_matrix, R_co, R_oc, T_co, T_oc]

    def load_coefficients_best_fit_plane(self,path):
        """ Loads best fitting plane transformation parameters relative to the world frame. """
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        R_op = cv_file.getNode("R_op").mat()
        R_po = cv_file.getNode("R_po").mat()
        T_op = cv_file.getNode("T_op").mat()
        T_po = cv_file.getNode("T_po").mat()
        
        cv_file.release()
        return [R_op, R_po, T_op, T_po]

    def handle_mobile_robot_pose(self, msg):
        # Convert a ROS image message into an cv::Mat, module cv_bridge
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)


        cv2.imshow("Image window", frame)
        cv2.waitKey(1)

        # t = geometry_msgs.msg.TransformStamped()
        # t.header.stamp = rospy.Time.now()

        # t.header.frame_id = self.tf_end_effector_frame_id
        # rospy.logwarn("header: "+ str(t.header.frame_id))
        # t.child_frame_id = self.tf_root_camera_frame_id
        # rospy.logwarn("child: "+ str(t.child_frame_id))

        # # print(self.arm2camera_pose)
        # t.transform.translation.x = self.arm2camera_pose['position']['x']
        # t.transform.translation.y = self.arm2camera_pose['position']['y']
        # t.transform.translation.z = self.arm2camera_pose['position']['z']
        # # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        # t.transform.rotation.x = self.arm2camera_pose['orientation']['x']
        # t.transform.rotation.y = self.arm2camera_pose['orientation']['y']
        # t.transform.rotation.z = self.arm2camera_pose['orientation']['z']
        # t.transform.rotation.w = self.arm2camera_pose['orientation']['w']

        # self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    arucoRobots2Floor = ArucoRobots2Floor()
    rospy.spin()