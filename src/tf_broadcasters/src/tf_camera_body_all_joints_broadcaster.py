#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: tf_camera_body_all_joints_broadcaster
Description:
    Broadcasts to tf2 the trasformation between the Kinect Azure camera's
    'depth_camera_link'
    and the first detected persons' all joints with prefix "JOINT_" 
    defined in https://docs.microsoft.com/en-us/azure/kinect-dk/body-joints, eg, "JOINT_WRIST_LEFT" 

Parameters:
    - kinect_body_tracking_data_topic_name: topic that is the body tracking data is published by a kinect camera eg."/body_tracking_data"
    - tf_camera_frame_id: frame name that is the joints are related to by Kinect ROS drivers, eg, "depth_camera_link"
    - body_joints_tf_prefix: tf prefix for the body joints that will allow to distinguish the source of published body joint tf frames, if specified usually ends with an underscore "_" char
Subscribes to:
    - /body_tracking_data (visualization_msgs::MarkerArray)
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
import visualization_msgs.msg
# import kinova_msgs.msg
import numpy as np
import math

class Kinect2BodyAllJointsTf():
    def __init__(self):
        rospy.init_node('tf_camera_body_all_joints_broadcaster', anonymous=True)

        self.kinect_body_tracking_data_topic_name = rospy.get_param('~kinect_body_tracking_data_topic_name', "/body_tracking_data")
        self.parent_frame_id = rospy.get_param("~tf_camera_frame_id", "depth_camera_link")
        self.body_joints_tf_prefix = rospy.get_param("~body_joints_tf_prefix", "")

        # Create covariance vector with size 36 = 6x6 for (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        self.covariance_diagonal_max = rospy.get_param('~pose_covariance_diagonal_max',[1.,1.,1.,1.,1.,1.]) 
        self.covariance_diagonal_min = rospy.get_param('~pose_covariance_diagonal_min',[1.,1.,1.,1.,1.,1.])
        self.tf_broadcast_enable = rospy.get_param("~tf_broadcast_enable", True)

        # Get reliability function parameters m and s
        self.reliability_func_param_m = rospy.get_param('~reliability_func_param_m',0.0) 
        self.reliability_func_param_s = rospy.get_param('~reliability_func_param_s',0.42)

        # create body joint pose publishers
        self.pubs_PoseWithCovarianceStamped = {}
        for joint_name, joint_num in KINECT_JOINT_DICT.items():
            topic_name = 'Pose_'+ self.body_joints_tf_prefix + joint_name.lower()
            publisher = rospy.Publisher(topic_name, geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=2)
            self.pubs_PoseWithCovarianceStamped[joint_name] =  publisher

        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        rospy.Subscriber(self.kinect_body_tracking_data_topic_name, visualization_msgs.msg.MarkerArray, self.handle_joint_pose, queue_size=1)

    def handle_joint_pose(self, msg):
        if len(msg.markers) > 0: # If at least one body is detected 
            for joint_name, joint_num in KINECT_JOINT_DICT.items():
                joint_marker = msg.markers[joint_num]
                
                t = geometry_msgs.msg.TransformStamped()
                # t.header.stamp = rospy.Time.now()
                t.header.stamp = joint_marker.header.stamp

                t.header.frame_id = self.parent_frame_id
                t.child_frame_id = self.body_joints_tf_prefix + joint_name.lower()

                t.transform.translation = joint_marker.pose.position
                t.transform.rotation = joint_marker.pose.orientation

                if self.tf_broadcast_enable:
                    self.tf_broadcaster.sendTransform(t)

                ## Publish pose with covariance of that body joint
                # calculate its distance to camera
                distance = self.calculate_distance(joint_marker.pose.position)
                # calculate reliability score corresponding to that distance
                score = self.calculate_reliability(distance)
                # calculate corresponding covariance based on the reliability score
                covariance = self.calculate_covariance(score)

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

                pose_msg.pose.covariance = covariance

                self.pubs_PoseWithCovarianceStamped[joint_name].publish(pose_msg)
        else:
            rospy.logerr_throttle(5, "No body could be detected, waiting to detect..")

    def calculate_distance(self,position):
        return np.linalg.norm(np.array([position.x, position.y, position.z])) 

    def calculate_reliability(self,distance):
        # returns score between 0 - 1. See https://www.desmos.com/calculator/dprpp1kcu0 for details.
        # 1: max reliable, 0: min reliable.

        x = distance
        m = self.reliability_func_param_m
        s = self.reliability_func_param_s

        log_normal = (1.0/(x*s*math.sqrt(2*math.pi))) * math.exp(-((math.log(x)-m)**2) / (2 * s**2) )

        k = (1.0/(s*math.sqrt(2*math.pi))) * math.exp( (s**2 - 2*m) / 2.0 )# normalization factor 

        # p = math.exp( m - s**2 ) # most reliable distance for debug purposes

        score = log_normal / k
        return score

    def calculate_covariance(self,reliability_score):
        covariance =  np.zeros(36)

        # self.covariance_diagonal_max
        # self.covariance_diagonal_min
        
        covariance[0]  = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[0],self.covariance_diagonal_max[0]) # x
        covariance[7]  = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[1],self.covariance_diagonal_max[1]) # y
        covariance[14] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[2],self.covariance_diagonal_max[2]) # z
        covariance[21] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[3],self.covariance_diagonal_max[3]) # rot x
        covariance[28] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[4],self.covariance_diagonal_max[4]) # rot y
        covariance[35] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[5],self.covariance_diagonal_max[5]) # rot z
        covariance = list(covariance) # convert to list of 36 floats

        return covariance

    def interpolate_linear(self,score,min,max):
        return min + (1.0-score)*(max-min)
        
    


KINECT_JOINT_DICT = {
    "JOINT_PELVIS" : 0, 
    "JOINT_SPINE_NAVEL" : 1, 
    "JOINT_SPINE_CHEST" : 2, 
    "JOINT_NECK" : 3,
    "JOINT_CLAVICLE_LEFT" : 4, 
    "JOINT_SHOULDER_LEFT" : 5, 
    "JOINT_ELBOW_LEFT" : 6, 
    "JOINT_WRIST_LEFT" : 7,
    "JOINT_HAND_LEFT" : 8,
    "JOINT_HANDTIP_LEFT" : 9, 
    "JOINT_THUMB_LEFT" : 10, 
    "JOINT_CLAVICLE_RIGHT" : 11,
    "JOINT_SHOULDER_RIGHT" : 12, 
    "JOINT_ELBOW_RIGHT" : 13, 
    "JOINT_WRIST_RIGHT" : 14, 
    "JOINT_HAND_RIGHT" : 15,
    "JOINT_HANDTIP_RIGHT" : 16, 
    "JOINT_THUMB_RIGHT" : 17, 
    "JOINT_HIP_LEFT" : 18, 
    "JOINT_KNEE_LEFT" : 19,
    "JOINT_ANKLE_LEFT" : 20, 
    "JOINT_FOOT_LEFT" : 21, 
    "JOINT_HIP_RIGHT" : 22, 
    "JOINT_KNEE_RIGHT" : 23,
    "JOINT_ANKLE_RIGHT" : 24, 
    "JOINT_FOOT_RIGHT" : 25, 
    "JOINT_HEAD" : 26, 
    "JOINT_NOSE" : 27,
    "JOINT_EYE_LEFT" : 28, 
    "JOINT_EAR_LEFT" : 29, 
    "JOINT_EYE_RIGHT" : 30, 
    "JOINT_EAR_RIGHT" : 31
}

if __name__ == '__main__':
    kinect2BodyAllJointsTf = Kinect2BodyAllJointsTf()
    rospy.spin()