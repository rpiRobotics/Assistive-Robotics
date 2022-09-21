#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: tf_arm_ft_broadcaster
Description:
    Broadcasts to tf2 the trasformation between the FT sensor wrench frame
    and arm's physical wrench frame
    
Parameters:
    - TODO
    - tf_arm_frame_id: Arm's physical wrench frame mounted
    - tf_ft_frame_id: FT sensors tf frame that is published
    - arm2ft_pose: 
Subscribes to:
    - NONE
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

class Arm2FtTf():
    def __init__(self):
        rospy.init_node('tf_arm_ft_broadcaster', anonymous=True)


        self.tf_arm_frame_id = rospy.get_param("~tf_arm_frame_id", "j2n6s300_right_wrench")
        self.tf_ft_frame_id = rospy.get_param("~tf_ft_frame_id", "ft_right_wrench")
    
        self.arm2ft_pose = rospy.get_param("~arm2ft_pose") 

        # self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        self.tf_broadcaster_static = tf2_ros.StaticTransformBroadcaster() # Create a static tf broadcster for rbg camera and the world frame
        self.publish_tf_static()

    def publish_tf_static(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = self.tf_arm_frame_id
        t.child_frame_id = self.tf_ft_frame_id

        t.transform.translation.x = self.arm2ft_pose['position']['x']
        t.transform.translation.y = self.arm2ft_pose['position']['y']
        t.transform.translation.z = self.arm2ft_pose['position']['z'] 
        
        # # Convert R rotation matrix to quaternion
        # rot_mat = np.eye(4)
        # rot_mat[:3,:3] = rot_mat
        # q = tf_conversions.transformations.quaternion_from_matrix(rot_mat)

        t.transform.rotation.x = self.arm2ft_pose['orientation']['x']
        t.transform.rotation.y = self.arm2ft_pose['orientation']['y']
        t.transform.rotation.z = self.arm2ft_pose['orientation']['z']
        t.transform.rotation.w = self.arm2ft_pose['orientation']['w']

        self.tf_broadcaster_static.sendTransform(t)

if __name__ == '__main__':
    arm2FtTf = Arm2FtTf()
    rospy.spin()