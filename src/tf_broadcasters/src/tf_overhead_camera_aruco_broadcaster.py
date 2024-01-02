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
    Also, broadcasts the static tf btw. the world floor and the camera rgb frame

Parameters: # TODO for explanations
    - debug_image_view
    - debug_image_topic_name
    - debug_image_scale_percent
    - image_topic_name
    - using_rectified_image
    - path_to_camera_parameters
    - path_to_undistorted_camera_parameters
    - path_to_extrinsic_parameters
    - path_to_aruco_tags_info
    - tf_rgb_camera_frame_id
    - tf_world_floor_frame_id
    - robot_bases_tf_prefix
    - robot_bases_tf_postfix
Subscribes to:
    - "/rgb/image_raw" or "/rgb/image_rect_color" (sensor_msgs::Image)
Publishes to:
    - NONE
Broadcasts to:
    - tf2
    - TODO for PoseWithCovarianceStamped msgs
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
import time

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

from rospy_log_controller import LogController

class ArucoRobots2Floor():
    def __init__(self):
        rospy.init_node('tf_overhead_camera_aruco_broadcaster', anonymous=True)

        self.logger = LogController() # To Manage the maximum rate of rospy log data

        self.debug_image_view = rospy.get_param('~debug_image_view', False)
        self.debug_image_topic_name = rospy.get_param('~debug_image_topic_name', "debug_aruco_detected_image")
        self.debug_image_scale_percent = rospy.get_param('~debug_image_scale_percent', 50) # percent of original size
        # Debug Image publisher
        self.pub_image = rospy.Publisher(self.debug_image_topic_name, sensor_msgs.msg.Image, queue_size=1)
        # Raw Image topic name that this node subscribes
        self.image_topic_name = rospy.get_param('~image_topic_name', "/rgb/image_raw")
        self.using_rectified_image = rospy.get_param('~using_rectified_image', False)

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
        rospy.loginfo("User aruco types on robots: " + str(self.aruco_types))

        # Initialization of undistortion maps for efficient undistortions
        self.map1 = None 
        self.map2 = None 
        # Initialization of Image width and height
        self.width = None
        self.height = None

        self.tf_rgb_camera_frame_id = rospy.get_param ("~tf_rgb_camera_frame_id", "cage_rgb_camera_link")
        self.tf_world_floor_frame_id = rospy.get_param("~tf_world_floor_frame_id", "world_floor")
        self.robot_bases_tf_prefix = rospy.get_param("~robot_bases_tf_prefix", "")
        self.robot_bases_tf_postfix = rospy.get_param("~robot_bases_tf_postfix", "_base")

        # get all robot names to be potentially published for their poses from "place" names
        self.robot_names = list(self.df["place"])
        # rospy.loginfo("Robot names: " + str(self.robot_names))
        self.num_of_robots = len(self.robot_names)
        # create robot pose publishers
        self.pubs_PoseWithCovarianceStamped = []
        for i in range(self.num_of_robots):
            topic_name = 'Pose_'+ self.robot_bases_tf_prefix + self.robot_names[i] + self.robot_bases_tf_postfix
            publisher = rospy.Publisher(topic_name, geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=2)
            self.pubs_PoseWithCovarianceStamped.append(publisher)

        # Create covariance vector with size 36 = 6x6 for (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis), # Assumed to be the same for all robots!
        self.covariance_diagonal = rospy.get_param('~pose_covariance_diagonal',[1.,1.,1.,1.,1.,1.]) 

        # rospy.loginfo("self.covariance_diagonal: " + str(self.covariance_diagonal))
        self.covariance = np.zeros(36)
        self.covariance[0] =  self.covariance_diagonal[0] # x
        self.covariance[7] =  self.covariance_diagonal[1] # y
        self.covariance[14] =  self.covariance_diagonal[2] # z
        self.covariance[21] =  self.covariance_diagonal[3] # rot x
        self.covariance[28] =  self.covariance_diagonal[4] # rot y
        self.covariance[35] =  self.covariance_diagonal[5] # rot z
        self.covariance = list(self.covariance) # convert to list of 36 floats
        
        # rospy.loginfo("self.covariance: " + str(self.covariance))

        self.tf_broadcast_enable = rospy.get_param("~tf_broadcast_enable", False)

        self.bridge = CvBridge() # To convert ROS images to openCV imgs.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster for robots and world frame
        self.tf_broadcaster_static = tf2_ros.StaticTransformBroadcaster() # Create a static tf broadcster for rbg camera and the world frame
        self.publish_rgb2world_floor_tf_static()
        rospy.Subscriber(self.image_topic_name, sensor_msgs.msg.Image, self.handle_mobile_robot_pose, queue_size=1)

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
            rospy.logwarn("[INFO]: could not read R_co, R_oc, T_co, T_oc from: {}".format(path))
            rospy.logwarn(str(R_co), str(R_oc), str(T_co), str(T_oc))
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
            rospy.logerr(e)

        # cv2.imshow("Image window", frame)
        # cv2.waitKey(1)

        if  self.map1 is None: # or  self.map2 == None or self.width == None or self.height == None
            # Get the image height and width
            self.height, self.width = frame.shape[:2]
            # Create only once the undistortion maps for efficient undistortions
            self.map1, self.map2 = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, self.mtx, (self.width, self.height), cv2.CV_32FC1)


        time_stamp = rospy.Time.now()
        # start_time0 = time.time()
        # start_time = start_time0
        
        if not self.using_rectified_image:
            # try to undistort image
            # frame = cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)
            # try to undistort image with remapping (since more efficient)
            frame = cv2.remap(frame,self.map1,self.map2,cv2.INTER_LINEAR)

        # rospy.logwarn("-- 001 --- %s seconds ---" % (time.time() - start_time))
        # start_time = time.time()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # rospy.logwarn("-- 002 --- %s seconds ---" % (time.time() - start_time))
        # start_time = time.time()

        places_all = []
        types_all = []
        corners_all = []
        ids_all = []
        sizes_all = []
        xs_all = []
        ys_all = []
        zs_all = []

        for aruco_type in self.aruco_types:
            arucoType = ARUCO_DICT[aruco_type]

            # verify that the supplied ArUCo tag exists and is supported by OpenCV
            if ARUCO_DICT.get(aruco_type, None) is None:
                rospy.logerr("[ERROR] ArUCo tag of '{}' is not supported".format(aruco_type))
                sys.exit(0)

            # load the ArUCo dictionary, grab the ArUCo parameters, and detect the markers
            # rospy.loginfo("[INFO] detecting '{}' tags...".format(aruco_type))
            arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
            
            # only keep the detections that are NOT on the floor by looking at the IDs
            ids_on_robots_with_current_aruco_type = list(self.df[(self.df["aruco_type"]==aruco_type)]["id"])
            if len(corners) > 0: # verify *at least* one ArUco marker was detected            
                for i, markerID in enumerate(list(ids.flatten())): # loop over the detected ArUCo corners
                    if markerID in ids_on_robots_with_current_aruco_type:
                        robot_row = self.df[(self.df["aruco_type"]==aruco_type) & (self.df["id"]==markerID)]
                        place = robot_row["place"].item()
                        places_all.append(place)

                        types_all.append(aruco_type)

                        corners_all.append(corners[i])
                        
                        ids_all.append(markerID)
                        
                        markerSize = float(robot_row["size_mm"])
                        sizes_all.append(markerSize)

                        x = float(robot_row["x"])
                        xs_all.append(x)

                        y = float(robot_row["y"])
                        ys_all.append(y)

                        z = float(robot_row["z"])
                        zs_all.append(z)

            # print(places_all)
            # print(types_all)
            # print(corners_all)
            # print(ids_all)
            # print(sizes_all)
            # print(xs_all)
            # print(ys_all)
            # print(zs_all)

        # rospy.loginfo("Num of detected Tags: " + str(len(corners_all)))

        # rospy.logwarn("-- 003 --- %s seconds ---" % (time.time() - start_time))
        # start_time = time.time()
        
        corners_all = np.array(corners_all)
        ids_all = np.array(ids_all)
        sizes_all = np.array(sizes_all)
        xs_all = np.array(xs_all)
        ys_all = np.array(ys_all)
        zs_all = np.array(zs_all)

        # verify *at least* one ArUco marker was remained on robots
        if len(corners_all) > 0:
            rvecs_all = []
            tvecs_all = []
            # loop over the detected ArUCo corners and draw ids and bounding boxes around the detected markers with the robot information
            for (place, aruco_type, markerCorner, markerID, markerSize, x,y,z) in zip(places_all, types_all, corners_all, ids_all, sizes_all, xs_all,ys_all,zs_all):
                if self.debug_image_view:
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))

                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # draw the bounding box of the ArUCo detection
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 1)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 1)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 1)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 1)
                    # compute and draw the center (x, y)-coordinates of the ArUco
                    # marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                    # draw the ArUco marker ID on the image
                    place_info = str(place) + ", "+ str(aruco_type) + ", id:" +str(markerID) + ", " + str(markerSize) + " mm"
                    cv2.putText(frame, place_info,
                        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

                # Estimate the pose of the detected marker in camera frame
                rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(markerCorner, markerSize, self.mtx_new, self.dist_new)
                
                if self.debug_image_view:
                    cv2.aruco.drawAxis(frame, self.mtx_new, self.dist_new, rvec, tvec, markerSize*0.75)  # Draw Axis

                # Add the Transform from robot frame to marker frame
                R_cm = cv2.Rodrigues(rvec.flatten())[0] # 3x3
                T_cm = tvec[0].T # 3x1

                R_rm = np.eye(3)# Marker and the Robot has the same orientation assumption, otherwise we had to parse it from csv file adding orientation paramaters
                T_rm = np.array([x,y,z]).reshape(3,1)*1000.0 # 3x1 # Multiply with 1000 to convert to mm. ( recall reading from csv is in meters)

                R_cr = np.matmul(R_cm,R_rm.T)
                T_cr = T_cm - np.matmul(R_cr,T_rm)

                # print("[INFO] ArUco marker ID: {}".format(markerID))
                # print(tvec[0].flatten()) # in camera's frame)
                # print(rvec[0].flatten()) # in camera's frame)
                rvecs_all.append(R_cr)
                tvecs_all.append(T_cr)

            rvecs_all = np.array(rvecs_all) # (N,3,3)
            # print("rvecs_all.shape:", rvecs_all.shape)
            tvecs_all = np.array(tvecs_all) # (N,3,1)
            # print("tvecs_all.shape:",tvecs_all.shape)
            tvecs_all = np.squeeze(tvecs_all, axis=2).T # (3,N)

            if self.debug_image_view:
                # show the output image
                # cv2.imshow("Image", frame)
                # cv2.waitKey(1)

                # Scale down the image efficiently
                width = int(self.width * self.debug_image_scale_percent / 100)
                height = int(self.height * self.debug_image_scale_percent / 100)
                dim = (width, height)
                
                # resize image
                frame = cv2.resize(frame, dim, interpolation = cv2.INTER_NEAREST)

                # Publish this image to ROS as a scaled down image
                try:
                    self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                except CvBridgeError as e:
                    rospy.logwarn("Could not publish the aruco detected debug image")

            # rospy.logwarn("-- 004 --- %s seconds ---" % (time.time() - start_time))
            # start_time = time.time()

            # Transform detected robot locations from camera frame to World frame
            rvecs_all = np.matmul(self.R_oc,rvecs_all) # (N,3,3) # R_or 

            tvecs_all = np.matmul(self.R_oc,tvecs_all) # (3,N)
            tvecs_all = self.T_oc + tvecs_all # (3,N) # T_or 
            # print(np.shape(tvecs_all))

            # Transform detected robot locations from World frame to plane frame
            rvecs_all = np.matmul(self.R_po,rvecs_all) # (N,3,3) # R_pr 
            # rospy.logwarn("rvecs_all.shape:" + str(rvecs_all.shape))

            tvecs_all = np.matmul(self.R_po,tvecs_all) # (3,N)
            tvecs_all = self.T_po + tvecs_all # (3,N) # T_pr
            # rospy.logwarn("tvecs_all.shape:" + str(tvecs_all.shape))

            # rospy.logwarn("-- 005 --- %s seconds ---" % (time.time() - start_time))
            # start_time = time.time()

            # Finally, create and send tf robot poses wrt floor plane
            for (place, translation, rot_mat) in zip(places_all, tvecs_all.T, rvecs_all):
                t = geometry_msgs.msg.TransformStamped()
                # t.header.stamp = rospy.Time.now()
                t.header.stamp = time_stamp

                t.header.frame_id = self.tf_world_floor_frame_id
                t.child_frame_id = self.robot_bases_tf_prefix + place + self.robot_bases_tf_postfix

                # Translation 
                t.transform.translation.x = translation[0]/1000.0 # convert to meters from mm
                t.transform.translation.y = translation[1]/1000.0 # convert to meters from mm
                t.transform.translation.z = translation[2]/1000.0 # convert to meters from mm

                # Convert R rotation matrix to quaternion
                rot_mat2 = np.eye(4)
                rot_mat2[:3,:3] = rot_mat
                # rospy.logwarn("rot_mat.shape:" + str(rot_mat2.shape))
                # rospy.logwarn("rot_mat:" + str(rot_mat2))
                q = tf_conversions.transformations.quaternion_from_matrix(rot_mat2)

                # Rotation
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                if self.tf_broadcast_enable:
                    self.tf_broadcaster.sendTransform(t)

                # Also publish the PoseWithCovarianceStamped msgs
                # Create the PoseWithCovarianceStamped msg
                pose_msg = geometry_msgs.msg.PoseWithCovarianceStamped()
                pose_msg.header = t.header

                pose_msg.pose.pose.position.x = t.transform.translation.x
                pose_msg.pose.pose.position.y = t.transform.translation.y
                pose_msg.pose.pose.position.z = t.transform.translation.z

                pose_msg.pose.pose.orientation.x = t.transform.rotation.x
                pose_msg.pose.pose.orientation.y = t.transform.rotation.y
                pose_msg.pose.pose.orientation.z = t.transform.rotation.z
                pose_msg.pose.pose.orientation.w = t.transform.rotation.w

                pose_msg.pose.covariance = self.covariance

                pub_index = self.robot_names.index(place)
                self.pubs_PoseWithCovarianceStamped[pub_index].publish(pose_msg)


            # rospy.logwarn("-- 006 --- %s seconds ---" % (time.time() - start_time))
            # rospy.logwarn("--------------- ALL --- %s seconds ---" % (time.time() - start_time0))
        else:
            # rospy.logwarn("No robot could be detected, waiting to detect..")
            self.logger.log("No robot could be detected, waiting to detect..",
                            log_type='warning', 
                            min_period=2.0) 



    def publish_rgb2world_floor_tf_static(self):
        translation = self.T_co + np.matmul(self.R_co,self.T_op) 
        rot_mat = np.matmul(self.R_co,self.R_op)
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = self.tf_rgb_camera_frame_id
        t.child_frame_id = self.tf_world_floor_frame_id

        # Translation 
        t.transform.translation.x = translation[0]/1000.0 # convert to meters from mm
        t.transform.translation.y = translation[1]/1000.0 # convert to meters from mm
        t.transform.translation.z = translation[2]/1000.0 # convert to meters from mm

        # Convert R rotation matrix to quaternion
        rot_mat2 = np.eye(4)
        rot_mat2[:3,:3] = rot_mat
        # rospy.logwarn("rot_mat.shape:" + str(rot_mat2.shape))
        # rospy.logwarn("rot_mat:" + str(rot_mat2))
        q = tf_conversions.transformations.quaternion_from_matrix(rot_mat2)

        # Rotation
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster_static.sendTransform(t)


if __name__ == '__main__':
    arucoRobots2Floor = ArucoRobots2Floor()
    rospy.spin()