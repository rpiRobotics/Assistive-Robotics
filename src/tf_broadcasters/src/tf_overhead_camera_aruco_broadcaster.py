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
        self.debug_image_view = True

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

        # cv2.imshow("Image window", frame)
        # cv2.waitKey(1)

        time_stamp = rospy.Time.now()
            
        # try undistorted image
        # h,  w = frame.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        # undistort
        # # frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        frame = cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)
        # # # crop the image
        # # x, y, w, h = roi
        # # frame = frame[y:y+h, x:x+w]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
                print("[ERROR] ArUCo tag of '{}' is not supported".format(aruco_type))
                sys.exit(0)

            # load the ArUCo dictionary, grab the ArUCo parameters, and detect the markers
            print("[INFO] detecting '{}' tags...".format(aruco_type))
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

        rospy.loginfo("Num of detected Tags: " + str(len(corners_all)))
        
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
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, markerSize, self.mtx_new, self.dist_new)
                
                if self.debug_image_view:
                    cv2.aruco.drawAxis(frame, self.mtx_new, self.dist_new, rvec, tvec, markerSize*0.75)  # Draw Axis

                # Add the Transform from robot frame to marker frame
                R_cm = cv2.Rodrigues(rvec.flatten())[0] # 3x3
                T_cm = tvec[0].T # 3x1

                R_rm = np.eye(3)# Marker and the Robot has the same orientation assumption, otherwise we had to parse it from csv file adding orientation paramaters
                T_rm = np.array([x,y,z]).reshape(3,1) # 3x1

                R_cr = R_cm.dot(R_rm.T)
                T_cr = T_cm - R_cr.dot(T_rm)

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
                cv2.imshow("Image", frame)
                cv2.waitKey(1)
                # k = cv2.waitKey(1)

                # TODO: Publish this image to ROS


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