#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: tf_mobile_base_arm_base_broadcaster
Description:
    Broadcasts to tf2 the trasformation between the Kinova arm's base (root) frame
    'root_left_arm' or 'root_right_arm' (specified in oarbot_xxx_arm_with_ft.launch)
    and mobile robot's base (root frame)
    'oarbot_silver_base' or 'oarbot_blue_base' 
    ("_base" part is specified in nuc_tf_overhead_camera_aruco.yaml, 
    but it will be defined somewhere else after the switch to Lidar + IMU based localization is done from vision aruco localization)
    ("oarbot_xxx" part is specified in aruco_tags_info.csv) but again it will be defined somewhere else later.
    So it is best to get these names as parameters of this node named:
    'oarbot_xxx_tf_mobile_base_arm_base.yaml'
    This yaml file also used to get the pre-calculated pose btw the mobile_base and the arm_base named
    'mobile_base2arm_base_pose'

    Since the z height of the oarbots are adjustable, the node will listen for a topic for current z heights 
    named: "TODO"
    But this z height can be assumed to be fixed for other types of robots by using the switch defined in the yaml file
    'is_fixed_z' True or False
    If is_fixed_z is true, the translational z value in the 'mobile_base2arm_base_pose' will be used.
    Otherwise, z height that is listened from the subscriber will be added the specified z height.

Parameters:
    - TODO
    - z_height_topic_name: 
    - is_fixed_z: 
    - tf_mobile_base_frame_id: 
    - tf_arm_base_frame_id: 
    - mobile_base2arm_base_pose: 
Subscribes to:
    - TODO /z_height_driver/z_height (geometry_msgs::PointStamped)
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

class MobileBase2ArmBaseTf():
    def __init__(self):
        rospy.init_node('tf_mobile_base_arm_base_broadcaster', anonymous=True)

        self.z_height_topic_name = rospy.get_param('~z_height_topic_name', "z_height_driver/z_height")
        self.is_fixed_z = rospy.get_param('~is_fixed_z', False)

        self.tf_mobile_base_frame_id = rospy.get_param("~tf_mobile_base_frame_id", "oarbot_blue_base")
        self.tf_arm_base_frame_id = rospy.get_param("~tf_arm_base_frame_id", "root_right_arm")
    
        self.mobile_base2arm_base_pose = rospy.get_param("~mobile_base2arm_base_pose") 
    
        self.z_height = 0.0
        if not self.is_fixed_z:
            rospy.Subscriber(self.z_height_topic_name, geometry_msgs.msg.PointStamped, self.handle_z_height, queue_size=1)

        # self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        self.tf_broadcaster_static = tf2_ros.StaticTransformBroadcaster() # Create a static tf broadcster for rbg camera and the world frame
        self.publish_rgb2world_floor_tf_static()
    
    def handle_z_height(self,msg):
        self.z_height = msg.point.z

    def publish_rgb2world_floor_tf_static(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = self.tf_mobile_base_frame_id
        t.child_frame_id = self.tf_arm_base_frame_id

        t.transform.translation.x = self.mobile_base2arm_base_pose['position']['x']
        t.transform.translation.y = self.mobile_base2arm_base_pose['position']['y']
        t.transform.translation.z = self.mobile_base2arm_base_pose['position']['z'] + self.z_height
        
        # # Convert R rotation matrix to quaternion
        # rot_mat = np.eye(4)
        # rot_mat[:3,:3] = rot_mat
        # q = tf_conversions.transformations.quaternion_from_matrix(rot_mat)

        t.transform.rotation.x = self.mobile_base2arm_base_pose['orientation']['x']
        t.transform.rotation.y = self.mobile_base2arm_base_pose['orientation']['y']
        t.transform.rotation.z = self.mobile_base2arm_base_pose['orientation']['z']
        t.transform.rotation.w = self.mobile_base2arm_base_pose['orientation']['w']

        self.tf_broadcaster_static.sendTransform(t)

if __name__ == '__main__':
    mobileBase2ArmBaseTf = MobileBase2ArmBaseTf()
    rospy.spin()