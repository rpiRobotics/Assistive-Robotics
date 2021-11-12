#!/usr/bin/env python  

"""
Node: tf_camera_body_hand_right_broadcaster
Description:
    Broadcasts to tf2 the trasformation between the Kinect Azure camera's
    'depth_camera_link'
    and the first detected persons' specified joint number in the yaml file

Used yaml file:
    - TODO
Subscribes to:
    - /body_tracking_data (visualization_msgs::MarkerArray)
Publishes to:
    -
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

class Kinect2BodySingleJointTf():
    def __init__(self):
        rospy.init_node('tf_camera_body_single_joint_broadcaster', anonymous=True)

        self.kinect_body_tracking_data_topic_name = rospy.get_param('~kinect_body_tracking_data_topic_name', "/body_tracking_data")
        self.joint_name = rospy.get_param("~tracked_single_joint_name", "JOINT_WRIST_LEFT")
        self.joint_num = KINECT_JOINT_DICT[self.joint_name]
        self.parent_frame_id = rospy.get_param("~tf_camera_frame_id", "depth_camera_link")

        rospy.Subscriber(self.kinect_body_tracking_data_topic_name, visualization_msgs.msg.MarkerArray, self.handle_joint_pose)
        pass

    def handle_joint_pose(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        
        joint_marker = msg.markers[self.joint_num]
        
        # t.header.stamp = rospy.Time.now()
        t.header.stamp = joint_marker.header.stamp

        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.joint_name.lower()

        t.transform.translation.x = joint_marker.pose.position.x
        t.transform.translation.y = joint_marker.pose.position.y
        t.transform.translation.z = joint_marker.pose.position.z  

        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = joint_marker.pose.orientation.x # q[0]
        t.transform.rotation.y = joint_marker.pose.orientation.y
        t.transform.rotation.z = joint_marker.pose.orientation.z
        t.transform.rotation.w = joint_marker.pose.orientation.w

        br.sendTransform(t)

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
    kinect2BodySingleJointTf = Kinect2BodySingleJointTf()
    rospy.spin()