#!/usr/bin/env python3

import rospy
from copy import deepcopy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg 

import kinova_msgs.msg

# # Because of transformations
import tf_conversions

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class SimpleSwarmMove():
    def __init__(self):
        
        # init node
        rospy.init_node('swarm_move_simple_trajectory', anonymous=True)

        # trajectory transformation publish rate
        self.traj_pub_rate = rospy.get_param("~swarm_traj_pub_rate", 200.0) # 200Hz for swarm (larger than 100hz kinova controller rate)

        # get all swarm specific parameters
        try:
            self.number_of_bots=rospy.get_param('~number_of_robots')
            self.robot_tfs_id=rospy.get_param('~robot_tf_frames')
            assert self.number_of_bots==len(self.robot_tfs_id), "Number of robots does not match the number of robot tf frames"
        except Exception as e:
            rospy.logerr(f"Error getting swarm parameters: {e}")
            self.number_of_bots=2
            self.robot_tfs_id=["j2n6s300_left_end_effector","j2n6s300_right_end_effector"]
            rospy.logwarn("Could not find all swarm parameters, setting to default 2 robots with tf frames j2n6s300_left_end_effector and j2n6s300_right_end_effector")
        # Specified swarm tf frame name to follow
        self.tf_swarm_frame_name = rospy.get_param("~tf_swarm_frame_name", "swarm_center").lower()
        rospy.set_param("/tf_swarm_frame_name", self.tf_swarm_frame_name) # set it to the static param name without namespace
        self.tf_swarm_map_frame_name = rospy.get_param("~tf_swarm_map_frame_name", "map").lower()
        rospy.set_param("/tf_swarm_map_frame_name", self.tf_swarm_map_frame_name) # set it to the static param name without namespace
        # Trajectory parameters
        self.enable_swarm_move = False # enable/disable swarm move
        self.trajectory_step_counts = 0 # counts the number of trajectory steps sent
        self.trajectory_static_filename = rospy.get_param("~trajectory_static_filename", "swarm_move_traj") # name of the static trajectory to be followed
        self.trajectory_static = np.loadtxt(self.trajectory_static_filename, delimiter=",", skiprows=1) # pre-load the static trajectory

        # tf2 listener and buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # TF2 broadcaster (for showing purposes)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        # transformations
        self.T_map2robots = None # list of transformations from map to each robot base
        self.T_map2swarm = None # transformation from map to swarm frame
        self.T_map2swarm_start = None # transformation from map to swarm frame at the start of the trajectory
        self.is_ok_tf_robots = False

        # swarm trajectory service
        self.swarm_frame_reset_service_name = rospy.get_param("~reset_swarm_frame_service_name")
        self.srv_call_swarm_frame_reset = rospy.Service(self.swarm_frame_reset_service_name, SetBool, self.srv_swarm_frame_reset_cb)
        self.swarm_move_service_name = rospy.get_param("~swarm_move_service_name")
        self.srv_call_swarm_move = rospy.Service(self.swarm_move_service_name, SetBool, self.srv_swarm_move_cb)

        # start the trajectory streaming loop
        rospy.Timer(rospy.Duration(1.0/self.traj_pub_rate), self.traj_stream_loop)

    def traj_stream_loop(self, event):

        if not self.enable_swarm_move or self.T_map2swarm_start is None:
            self.trajectory_step_counts = 0
            if self.T_map2swarm is not None:
                # broadcast the last known swarm frame
                self.broadcast_tf_goal(self.T_map2swarm[3:], self.T_map2swarm[:3], tf_frame=self.tf_swarm_map_frame_name, tf_child_frame=self.tf_swarm_frame_name)
            return
        
        if self.trajectory_step_counts >= self.trajectory_static.shape[0]:
            rospy.loginfo("Swarm trajectory completed")
            self.enable_swarm_move = False
            self.trajectory_step_counts = 0
            return
        
        # find the starting swarm frame in the map frame homogeneous transformation
        T_map2swarm_start_HMat = tf_conversions.transformations.quaternion_matrix(self.T_map2swarm_start[3:])
        T_map2swarm_start_HMat[0:3, 3] = deepcopy(self.T_map2swarm_start[0:3])
        # get the next trajectory point
        traj_point = self.trajectory_static[self.trajectory_step_counts, :]
        # traj_point = [x, y, z, qx, qy, qz, qw] in the swarm frame
        traj_T_start2point = tf_conversions.transformations.quaternion_matrix(traj_point[3:7])
        traj_T_start2point[0:3, 3] = deepcopy(traj_point[0:3])
        # calculate the next swarm frame in the map frame
        T_map2swarm_next = T_map2swarm_start_HMat @ traj_T_start2point
        # broadcast the next swarm frame
        rot_map2swarm_next = tf_conversions.transformations.quaternion_from_matrix(T_map2swarm_next)
        pos_map2swarm_next = T_map2swarm_next[0:3, 3]
        self.broadcast_tf_goal(rot_map2swarm_next, pos_map2swarm_next, tf_frame=self.tf_swarm_map_frame_name, tf_child_frame=self.tf_swarm_frame_name)
        # update the current swarm frame
        self.T_map2swarm = np.append(pos_map2swarm_next, rot_map2swarm_next)
        # increment the trajectory step count
        self.trajectory_step_counts += 1
    
    def broadcast_tf_goal(self, frame_rot_quaternion, frame_pos, tf_frame=None, tf_child_frame=None):
        """
        Broadcasts a frame tf.
        """
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        if tf_frame is not None:
            t.header.frame_id = tf_frame
        else:
            t.header.frame_id = self.tf_body_joint_frame_name
        if tf_child_frame is not None:
            t.child_frame_id = tf_child_frame
        else:
            t.child_frame_id = self.tf_followed_body_joint_frame_name_goal

        t.transform.translation.x = frame_pos[0]
        t.transform.translation.y = frame_pos[1]
        t.transform.translation.z = frame_pos[2]
        
        t.transform.rotation.x = frame_rot_quaternion[0]
        t.transform.rotation.y = frame_rot_quaternion[1]
        t.transform.rotation.z = frame_rot_quaternion[2]
        t.transform.rotation.w = frame_rot_quaternion[3]

        self.tf_broadcaster.sendTransform(t)
    
    def look_tfs_for_robots(self, timeout=0.0, map_frame_id=None):

        if map_frame_id is None:
            map_frame_id = self.tf_swarm_map_frame_name

        # Look up the transformations for each robot
        self.T_map2robots = {}
        for robot_tf in self.robot_tfs_id:
            try:
                trans = self.tfBuffer.lookup_transform(map_frame_id, robot_tf, rospy.Time(0), rospy.Duration(timeout))
                self.T_map2robots[robot_tf] = trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Put a warning which says that the transformation could not found
                rospy.logwarn_throttle(20.0, f"TFs_swarm_move: Waiting to find the transformation from {map_frame_id} to {robot_tf} (throttled to 20s)")
                self.T_map2robots = None
                return False
        return True

    def srv_swarm_frame_reset_cb(self, req):
        assert isinstance(req, SetBoolRequest)

        # This is not required to be time critical, 
        # so we can wait for the tfs to be available by setting timeout
        self.is_ok_tf_robots = self.look_tfs_for_robots(timeout=1.0)

        if self.is_ok_tf_robots:
            # use the center of the robots as the swarm frame
            center_p = []
            for robot_tf in self.robot_tfs_id:
                center_p.append([self.T_map2robots[robot_tf].transform.translation.x,
                                 self.T_map2robots[robot_tf].transform.translation.y,
                                 self.T_map2robots[robot_tf].transform.translation.z])
            center_p = np.mean(np.array(center_p), axis=0)
            center_R = np.array([0.0, 0.0, 0.0, 1.0]) # no rotation
            self.broadcast_tf_goal(center_R, center_p, tf_frame=self.tf_swarm_map_frame_name, tf_child_frame=self.tf_swarm_frame_name)
            self.T_map2swarm = np.append(np.array(center_p), np.array(center_R))
            rospy.loginfo(f"Swarm frame reset to the center of the robots at position {center_p}")
            return SetBoolResponse(success=True, message="Swarm frame reset successful")
        else:
            # rospy.logwarn("Swarm frame reset failed. Cannot find all robot transformations.")
            # return SetBoolResponse(success=False, message="Swarm frame reset failed")

            center_p = np.array([1.0, 1.0, 0.0])
            center_R = np.array([0.0, 0.0, 0.0, 1.0])
            self.broadcast_tf_goal(center_R, center_p, tf_frame=self.tf_swarm_map_frame_name, tf_child_frame=self.tf_swarm_frame_name)
            self.T_map2swarm = np.append(np.array(center_p), np.array(center_R))
            rospy.logwarn("Swarm frame reset failed. Cannot find all robot transformations. Setting to default position [1, 1, 0]")
            return SetBoolResponse(success=False, message="Swarm frame reset failed. Set to default position [1, 1, 0]")
        
    def srv_swarm_move_cb(self, req):
        assert isinstance(req, SetBoolRequest)

        if req.data:
            # start swarm move
            rospy.loginfo("Swarm move started")
            try:
                self.T_map2swarm_start = deepcopy(self.T_map2swarm)
                self.enable_swarm_move = True
            except Exception as e:
                rospy.logerr(f"Error occurred while starting swarm move: {e}")
                self.enable_swarm_move = False
                return SetBoolResponse(success=False, message="Swarm move failed to start")
            return SetBoolResponse(success=True, message="Swarm move started")
        else:
            # stop swarm move
            self.enable_swarm_move = False
            rospy.loginfo("Swarm move stopped")
            return SetBoolResponse(success=True, message="Swarm move stopped")

if __name__ == '__main__':
    collab_move = SimpleSwarmMove()
    rospy.spin()