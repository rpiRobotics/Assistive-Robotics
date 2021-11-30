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

class Kinect2BodyAllJointsTf():
    def __init__(self):
        rospy.init_node('tf_camera_body_all_joints_broadcaster', anonymous=True)

        self.kinect_body_tracking_data_topic_name = rospy.get_param('~kinect_body_tracking_data_topic_name', "/body_tracking_data")
        self.parent_frame_id = rospy.get_param("~tf_camera_frame_id", "depth_camera_link")
        self.body_joints_tf_prefix = rospy.get_param("~body_joints_tf_prefix", "")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        rospy.Subscriber(self.kinect_body_tracking_data_topic_name, visualization_msgs.msg.MarkerArray, self.handle_joint_pose)
        pass

    def handle_joint_pose(self, msg):
        for joint_name, joint_num in KINECT_JOINT_DICT.items():
            joint_marker = msg.markers[joint_num]
            
            t = geometry_msgs.msg.TransformStamped()
            # t.header.stamp = rospy.Time.now()
            t.header.stamp = joint_marker.header.stamp

            t.header.frame_id = self.parent_frame_id
            t.child_frame_id = self.body_joints_tf_prefix + joint_name.lower()

            t.transform.translation = joint_marker.pose.position
            t.transform.rotation = joint_marker.pose.orientation

            self.tf_broadcaster.sendTransform(t)

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