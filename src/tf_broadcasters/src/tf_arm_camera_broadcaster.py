#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: tf_arm_camera_broadcaster
Description:
    Broadcasts to tf2 the trasformation between the Kinova arm's end effector frame
    'j2n6s300_end_effector'
    and the Kinect Azure camera's root frame
    'camera_base'
    by reading a yaml file named
    'oarbot_xxx_tf_arm_camera.yaml'
    for the pre-calculated pose btw end efector and the camera 
    'arm2camera_pose'

Parameters:
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
# import kinova_msgs.msg

class Arm2CameraTf():
    def __init__(self):
        rospy.init_node('tf_arm_camera_broadcaster', anonymous=True)

        self.end_effector_pose_topic_name = rospy.get_param('~end_effector_pose_topic_name', "/j2n6s300_driver/out/tool_pose")
        self.tf_end_effector_frame_id = rospy.get_param("~tf_end_effector_frame_id", "j2n6s300_end_effector")
        self.tf_root_camera_frame_id = rospy.get_param("~tf_root_camera_frame_id", "camera_base")
        self.arm2camera_pose = rospy.get_param("~arm2camera_pose") 

        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        rospy.Subscriber(self.end_effector_pose_topic_name, geometry_msgs.msg.PoseStamped, self.handle_camera_pose)

    def handle_camera_pose(self, msg):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = self.tf_end_effector_frame_id
        # rospy.logwarn("header: "+ str(t.header.frame_id))
        t.child_frame_id = self.tf_root_camera_frame_id
        # rospy.logwarn("child: "+ str(t.child_frame_id))

        # print(self.arm2camera_pose)
        t.transform.translation.x = self.arm2camera_pose['position']['x']
        t.transform.translation.y = self.arm2camera_pose['position']['y']
        t.transform.translation.z = self.arm2camera_pose['position']['z']
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = self.arm2camera_pose['orientation']['x']
        t.transform.rotation.y = self.arm2camera_pose['orientation']['y']
        t.transform.rotation.z = self.arm2camera_pose['orientation']['z']
        t.transform.rotation.w = self.arm2camera_pose['orientation']['w']

        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    arm2CameraTf = Arm2CameraTf()
    rospy.spin()