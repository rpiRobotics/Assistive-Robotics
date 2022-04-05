#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: admittance_controller_collaborative
Description:
    Assume 2 robots rigidly grasping a rigid object.
    Read the wrench values from the robots, convert them into cooperative task wrenches
    Extract the user intent for the admittance control
    Control is achieved with the velocity command to the whole oarbot (or to fixed based robot arms TODO).
Parameters:
    - TODO
Subscribes to:
    - tf2
    - TODO
Publishes to:
    - /oarbot_xxx/cmd_vel (geometry_msgs::Twist) 
        (or /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::PoseVelocity) TODO)

Broadcasts to:
    - 
"""

import rospy

import numpy as np
np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg 
# from geometry_msgs.msg import PoseStamped, TransformStamped    

import kinova_msgs.msg

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# # Because of transformations
import tf_conversions
#  tf_conversions.transformations.euler_from_quaternion(Q_eg)
import tf.transformations 

class AdmittanceControllerCollaborative():
    def __init__(self):
        rospy.init_node('admittance_controller_collaborative', anonymous=True)

        self.num_robots = rospy.get_param("~num_robots",2)
        assert self.num_robots >= 2
        
        ##############################################
        # To store the published velocities
        # Create a 2D array to store the published Velocity values to the robots (num_robots x 6)
        # 6 is ordered as [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]
        self.V_pub = np.zeros((self.num_robots,6))

        # Topic names to publish (velocities)
        self.robot_cartesian_cmd_vel_topic_names = rospy.get_param("~robot_cartesian_cmd_vel_topic_names")

        # Published msg type (either Geometry_msgs::Twist or kinova_msgs::PoseVelocity)
        self.robot_cartesian_cmd_vel_msg_type = rospy.get_param("~robot_cartesian_cmd_vel_msg_type", "geometry_msgs.msg.Twist")

        # Publishers
        self.pubs_cmd_vel = []
        for topic_name in self.robot_cartesian_cmd_vel_topic_names:
            if self.robot_cartesian_cmd_vel_msg_type == "geometry_msgs.msg.Twist":
                self.pubs_cmd_vel.append(rospy.Publisher(topic_name, geometry_msgs.msg.Twist, queue_size=1))
            elif self.robot_cartesian_cmd_vel_msg_type == "kinova_msgs.msg.PoseVelocity":
                self.pubs_cmd_vel.append(rospy.Publisher(topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1))

        ##############################################
        # To store the force readings on the robots
        # Create a 2D array to store the measured Wrench values of the robots (num_robots x 6)
        # 6 is ordered as [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]
        self.W_meas = np.zeros((self.num_robots,6))

        # Topic names to subscribe (wrenches)
        self.wrench_external_topic_names = rospy.get_param("~wrench_external_topic_names")
        
        # Subscribers (wrenches)
        self.subs_wrenches = [] # To prevent garbage collector to capture the subscribers
        for i in range(self.num_robots):
            topic_name = self.wrench_external_topic_names[i]
            self.subs_wrenches.append(rospy.Subscriber(topic_name, geometry_msgs.msg.WrenchStamped, self.wrench_external_callback, (i), queue_size=1))

        ##############################################
        self.enable_admittance = rospy.get_param("~enable_admittance", True)
        self.toggle_admittance_service_name = rospy.get_param("~toggle_admittance_service_name")

        # Service to toggle the admittance control (enable/disable)
        self.srv_toggle_admittance = rospy.Service(self.toggle_admittance_service_name, SetBool, self.srv_toggle_admittance_cb)

        ##############################################
        # Control Law Parameters and Gains
        # Error deadzone 
        self.force_err_thres = rospy.get_param("~force_err_thres")
        self.torque_err_thres = rospy.get_param("~torque_err_thres")

        # (Admittance spring constants)
        self.K_a = rospy.get_param("~K_a") # absolute task
        self.K_a = np.array(self.K_a)
        self.K_r = rospy.get_param("~K_r") # relative task
        self.K_r = np.array(self.K_r)

        # (Admittance damper constants)
        self.B_a = rospy.get_param("~B_a") # absolute task
        self.B_a = np.array(self.B_a)
        self.B_r = rospy.get_param("~B_r") # relative task
        self.B_r = np.array(self.B_r)

        # Desired wrenches
        self.W_a_desired = rospy.get_param("~W_a_desired") # absolute task
        self.W_a_desired = np.array(self.W_a_desired)

        self.W_r_desired = rospy.get_param("~W_r_desired") # relative task
        self.W_r_desired = np.array(self.W_r_desired)

        # Maximum accelaration and velocities of the robots
        self.max_lin_accs = rospy.get_param("~max_lin_accs")
        self.max_lin_vels = rospy.get_param("~max_lin_vels")
        self.max_ang_accs = rospy.get_param("~max_ang_accs")
        self.max_ang_vels = rospy.get_param("~max_ang_vels")

        ##############################################
        # Carried object properties
        # Mass of the carried object
        self.mass_of_carried_obj = rospy.get_param("~mass_of_carried_obj")

        # Gravity constant
        self.gravity_const = rospy.get_param("~gravity_const", 9.8)

        # Object center of mass wrt absolute task frame
        self.COM = rospy.get_param("~center_of_mass_position")
        self.COM = np.array([self.COM['x'],self.COM['y'],self.COM['z']])

        # TF frame names
        # Specified end effector tf frame name (id)
        self.tf_end_effector_frame_names = rospy.get_param("~tf_end_effector_frame_names")
        # Specified robot base tf frame name 
        self.tf_robot_base_frame_names = rospy.get_param("~tf_robot_base_frame_names")
        # Specified arm base tf frame names 
        self.tf_arm_base_frame_names = rospy.get_param("~tf_arm_base_frame_names")

        self.tf_interaction_point_frame_name = rospy.get_param("~tf_interaction_point_frame_name")
        self.tf_absolute_task_frame_name = rospy.get_param("~tf_absolute_task_frame_name")
        self.tf_relative_task_frame_name = rospy.get_param("~tf_relative_task_frame_name")
        self.tf_carried_obj_frame_name = rospy.get_param("~tf_carried_obj_frame_name")
        self.tf_world_frame_name = rospy.get_param("~tf_world_frame_name")

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #################### ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self.T_world2ees = [None for i in range(self.num_robots)]
        self.T_ees2absolute = [None for i in range(self.num_robots)]
        self.T_world2armbases = [None for i in range(self.num_robots)]
        self.T_ees2armbases = [None for i in range(self.num_robots)]

        self.T_absolute2relative = None
        self.T_absolute2interaction = None
        self.T_world2object = None
        self.T_world2absolute = None

        self.T_world2bases = [None for i in range(self.num_robots)]
        # self.T_bases2armbases = [None for i in range(self.num_robots)]
        # self.T_bases2ees = [None for i in range(self.num_robots)]
        
        self.TFs_are_ready = False

        #################### ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0) # 100Hz needed for kinova arm
        # self.rate = rospy.Rate(self.pub_rate)
        self.expected_duration = 1.00/self.pub_rate # seconds per cycle

        self.initial_time = rospy.Time.now().to_sec()

        self.time_last_wrench = [0. for i in range(self.num_robots)] # last time wrench input time keepers
        # Set a timeout to wait for incoming msgs. If there is no incoming msg more than this timeout
        # the node will publish 0 velocities for safety.
        self.wrench_wait_timeout = 10.0 * self.expected_duration

        # Start control
        rospy.Timer(rospy.Duration(self.expected_duration), self.run)


    def run(self, event=None):
        if self.enable_admittance:
            self.TFs_are_ready = self.get_TFs()
            if self.TFs_are_ready:
                rospy.logwarn("0)W_robot_1: " + str(self.W_meas[0]))
                rospy.logwarn("0)W_robot_2: " + str(self.W_meas[1]))
                rospy.logwarn("~")

                Wa,Wr = self.wrench_adapter()
                rospy.logwarn("1)Wa: " + str(Wa))
                rospy.logwarn("1)Wr: " + str(Wr))
                rospy.logwarn("~")

                # With control law specify the command
                Va,Vr = self.control_law_force(Wa,Wr)
                rospy.logwarn("2)Va: " + str(Va))
                rospy.logwarn("2)Vr: " + str(Vr))
                rospy.logwarn("~")

                V_world = self.task2world_velocities(Va,Vr)
                rospy.logwarn("3)V_world_1: " + str(V_world[0]))
                rospy.logwarn("3)V_world_2: " + str(V_world[1]))
                rospy.logwarn("~")

                self.V_pub = self.world2robot_velocities(V_world)
                rospy.logwarn("4)V_robot_1: " + str(self.V_pub[0]))
                rospy.logwarn("4)V_robot_2: " + str(self.V_pub[1]))
                rospy.logwarn("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

                # Publish the command to move the end effector to the body joint
                self.publishPoseVelCmd(self.V_pub)

            else:
                pass
                # # Do not command the robots since the transformation could not found
                # self.V_pub = np.zeros((self.num_robots,6))
                # self.publishPoseVelCmd(self.V_pub)

        # Check for timeout for incoming msgs
        for i in range(self.num_robots):
            if ((rospy.Time.now().to_sec() - self.time_last_wrench[i]) > self.wrench_wait_timeout):
                rospy.logerr("Admittance controller timed out")
                # If the timeouts, reset all wrenches/velocities to zero prevent any damage
                self.W_meas = np.zeros((self.num_robots,6))
                self.V_pub = np.zeros((self.num_robots,6))
                break

    def wrench_adapter(self):
        Wa,Wr = self.task_space_conversion()
        Wa = self.cancel_objects_gravity(Wa)
        Wa = self.retrieve_human_intent(Wa)
        return Wa, Wr

    def task_space_conversion(self):
        # We have the measured wrenches at the end effectors wrt. their arm bases in variable self.W_meas 
        # recall: self.W_meas is a 2D array with shape (num_robots x 6)
        # 6 is ordered as [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]

        Wa = np.zeros(6)

        # This function represents these wrenches in the absolute task frame (world)
        W1 = self.W_meas[0] # represented in 1st arm base frame
        W2 = self.W_meas[1] # represented in 2nd arm base frame

        # Transform these wrenches to world frame
        W1_in_world = self.transform_wrench(W1,self.T_world2armbases[0])
        W2_in_world = self.transform_wrench(W2,self.T_world2armbases[1])

        F1_in_world = W1_in_world[:3]
        M1_in_world = W1_in_world[3:]
        F2_in_world = W2_in_world[:3]
        M2_in_world = W2_in_world[3:]

        P1a_in_1 = self.get_position_from_transform(self.T_ees2absolute[0]) # Note: must be zero vector since absolute frame and the end effector of robot is assumed to be the same
        P2a_in_2 = self.get_position_from_transform(self.T_ees2absolute[1])

        P1a_in_world = self.transform_vector_rotate_only(P1a_in_1,self.T_world2ees[0])
        P2a_in_world = self.transform_vector_rotate_only(P2a_in_2,self.T_world2ees[1])

        Fa = F1_in_world + F2_in_world
        Ma = M1_in_world + np.cross(P1a_in_world,F1_in_world) + M2_in_world + np.cross(P2a_in_world,F2_in_world)

        Wa[:3] = Fa
        Wa[3:] = Ma

        # Wa is calculated 
        # Now calculate Wr
        Wr = np.zeros(6)

        W1_in_1 = self.transform_wrench(W1, self.T_ees2armbases[0])
        W2_in_2 = self.transform_wrench(W2, self.T_ees2armbases[1])
        W2_in_1 = self.transform_wrench(W2_in_2, self.T_absolute2relative)

        Wr = 0.5 * (W2_in_1 - W1_in_1)

        return Wa, Wr

    def cancel_objects_gravity(self,Wa):
        Wa_new = np.zeros(6)
        Fa = Wa[:3]
        Ma = Wa[3:]

        # assume gravity is pointing in -z direction in world frame
        mg = self.mass_of_carried_obj*self.gravity_const*np.array([0.,0.,-1.])
        Fa = Fa - mg

        # represent COM in world frame (it was at absolute frame/1st end effector frame)
        COM_in_world = self.transform_vector_rotate_only(self.COM, self.T_world2object)
        Ma = Ma - np.cross(COM_in_world,mg)

        Wa_new[:3] = Fa
        Wa_new[3:] = Ma        
        return Wa_new

    def retrieve_human_intent(self, Wa):
        Wa_new = np.zeros(6)
        Fa = Wa[:3]
        Ma = Wa[3:]

        Pa3_in_a = self.get_position_from_transform(self.T_absolute2interaction)
        Pa3_in_world = self.transform_vector_rotate_only(Pa3_in_a,self.T_world2absolute)
        Ma = Ma + np.cross(Fa,Pa3_in_world)

        Wa_new[:3] = Fa
        Wa_new[3:] = Ma   
        return Wa_new

    def get_TFs(self):
        # Note: lookup_transform func returns type geometry_msgs.msg.TransformStamped
        try:
            for i in range(self.num_robots):
                self.T_world2ees[i] = self.tfBuffer.lookup_transform(self.tf_world_frame_name, self.tf_end_effector_frame_names[i],  rospy.Time()) # in world frame 

                self.T_ees2absolute[i] = self.tfBuffer.lookup_transform(self.tf_end_effector_frame_names[i], self.tf_absolute_task_frame_name,  rospy.Time())

                self.T_world2armbases[i] = self.tfBuffer.lookup_transform(self.tf_world_frame_name, self.tf_arm_base_frame_names[i],  rospy.Time())

                self.T_ees2armbases[i] = self.tfBuffer.lookup_transform(self.tf_end_effector_frame_names[i], self.tf_arm_base_frame_names[i],  rospy.Time())

                self.T_world2bases[i] = self.tfBuffer.lookup_transform(self.tf_world_frame_name, self.tf_robot_base_frame_names[i],  rospy.Time())

            self.T_absolute2relative = self.tfBuffer.lookup_transform(self.tf_absolute_task_frame_name, self.tf_relative_task_frame_name,  rospy.Time())

            self.T_absolute2interaction = self.tfBuffer.lookup_transform(self.tf_absolute_task_frame_name, self.tf_interaction_point_frame_name,  rospy.Time())

            self.T_world2object = self.tfBuffer.lookup_transform(self.tf_world_frame_name, self.tf_carried_obj_frame_name,  rospy.Time())

            self.T_world2absolute = self.tfBuffer.lookup_transform(self.tf_world_frame_name, self.tf_absolute_task_frame_name,  rospy.Time())

            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            rospy.logerr('TFs in admittance_controller_colloborative: Waiting to find the all the necessary transformations') 
            return False


    def publishPoseVelCmd(self, V_pub):
        # V_pub: 2D array with shape (num_robots x 6)
        # 6 is ordered as [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]

        for i in range(self.num_robots):
            if self.robot_cartesian_cmd_vel_msg_type == "geometry_msgs.msg.Twist":
                pose_vel_msg = geometry_msgs.msg.Twist()
                pose_vel_msg.linear.x =  V_pub[i][0]
                pose_vel_msg.linear.y =  V_pub[i][1]
                pose_vel_msg.linear.z =  V_pub[i][2]
                pose_vel_msg.angular.x = V_pub[i][3]
                pose_vel_msg.angular.y = V_pub[i][4]
                pose_vel_msg.angular.z = V_pub[i][5]
                self.pubs_cmd_vel[i].publish(pose_vel_msg)

            elif self.robot_cartesian_cmd_vel_msg_type == "kinova_msgs.msg.PoseVelocity":
                pose_vel_msg = kinova_msgs.msg.PoseVelocity()
                pose_vel_msg.twist_linear_x =  V_pub[i][0]
                pose_vel_msg.twist_linear_y =  V_pub[i][1]
                pose_vel_msg.twist_linear_z =  V_pub[i][2]
                pose_vel_msg.twist_angular_x = V_pub[i][3]
                pose_vel_msg.twist_angular_y = V_pub[i][4]
                pose_vel_msg.twist_angular_z = V_pub[i][5]
                self.pubs_cmd_vel[i].publish(pose_vel_msg)

    def control_law_force(self, Wa,Wr):
        Va = np.zeros(6) # Absolute task velocity generated command
        Vr = np.zeros(6) # Relative task velocity generated command

        # Given task specification, desired velocities for the absolute and relative tasks
        # TODO: take these values from external trajectory generator topic
        Va_desired = np.zeros(6) 
        Vr_desired = np.zeros(6)

        # TODO: think something for desired poses as well (maybe)
        Xa_desired = np.zeros(6) 
        Xr_desired = np.zeros(6)
        Xa_cur = np.zeros(6) 
        Xr_cur = np.zeros(6)

        # Pose control term
        ctr_pose_a = self.K_a*(Xa_cur - Xa_desired) # element wise multiply
        ctr_wrench_a = (Wa-self.W_a_desired)
        Va = Va_desired + (1/self.B_a)*(ctr_wrench_a - ctr_pose_a) # element wise multiply

        ctr_pose_r = self.K_r*(Xr_cur - Xr_desired) # element wise multiply
        ctr_wrench_r = (Wr-self.W_r_desired)
        Vr = Vr_desired + (1/self.B_r)*(ctr_wrench_r - ctr_pose_r) # element wise multiply

        return Va,Vr

    def task2world_velocities(self,Va,Vr):
        # calculates the robot specific velocity commands in world frame (V_pub) from the given absolute and relative task velocities (Va,Vr)
        # Va: Velocity of absolute frame (a)
        # Vr: Velocity of relative frame (r)

        V_pub = np.zeros((self.num_robots,6))
        V_pub[0] = Va # since Va is already with respect to world

        va_o = Va[:3] # o: in world frame
        wa_o = Va[3:] # o: in world frame

        vr_a = Vr[:3] # a: in absolute frame
        wr_a = Vr[3:] # a: in absolute frame

        Par_a = self.get_position_from_transform(self.T_absolute2relative)
        Par_o = self.transform_vector_rotate_only(Par_a, self.T_world2absolute)
        vr_o = va_o + self.transform_vector_rotate_only(vr_a, self.T_world2absolute) - np.cross(Par_o,wa_o)

        wr_o = self.transform_vector_rotate_only(wr_a,self.T_world2absolute)

        V_pub[1][:3] = vr_o # o: in_world frame
        V_pub[1][3:] = wr_o # o: in_world frame

        return V_pub

    def world2robot_velocities(self,V_world):
        # calculates the robot specific velocity commands as each robot desired in their base frames (V_pub) from the given world frame velocities (V_world)
        V_pub = np.zeros((self.num_robots,6))

        for i in range(self.num_robots):
            V = V_world[i]
            v = V[:3]
            w = V[3:]
            
            v_mb = self.transform_vector_rotate_only(v,self.T_world2bases[i],inverse_tf=True) # mb: mobile base
            w_ee = self.transform_vector_rotate_only(w,self.T_world2ees[i],inverse_tf=True) # ee: end effector

            V_pub[i][:3] = v_mb
            V_pub[i][3:] = w_ee

        return V_pub

    def wrench_external_callback(self, wrench_stamped_msg, args):
        # args[0]: i # (Robot index)
        self.time_last_wrench[args] = rospy.Time.now().to_sec() 
        self.W_meas[args][0] = wrench_stamped_msg.wrench.force.x
        self.W_meas[args][1] = wrench_stamped_msg.wrench.force.y
        self.W_meas[args][2] = wrench_stamped_msg.wrench.force.z
        self.W_meas[args][3] = wrench_stamped_msg.wrench.torque.x
        self.W_meas[args][4] = wrench_stamped_msg.wrench.torque.y
        self.W_meas[args][5] = wrench_stamped_msg.wrench.torque.z

        #debug: check whether the self.W_meas is really updated with this callback
        # rospy.logwarn( "self.W_meas: " + str(self.W_meas)) 


    def srv_toggle_admittance_cb(self,req):
        assert isinstance(req, SetBoolRequest)

        if req.data:
            self.enable_admittance = True
            rospy.loginfo("Enable admittance control")
        else:
            self.enable_admittance = False
            rospy.loginfo("Disable admittance control")

        return SetBoolResponse(True, "The admittance is now set to: {}".format(self.enable_admittance))


    # HELPER FUNCTIONS
    def allowence(self, x, allowed):
        if abs(x) <= allowed:
            return 0
        else:
            return x

    def transform_wrench(self, W, T):
        # W: wrench 1D np array with 6 elements
        # T: transform
        # returns W_transformed: 1D np array with 6 elements

        qw = T.transform.rotation.w # Scalar part of quaternion
        qx = T.transform.rotation.x
        qy = T.transform.rotation.y
        qz = T.transform.rotation.z
        q = [qx,qy,qz,qw]
        R = tf.transformations.quaternion_matrix(q)
        R = R[:3,:3]

        W_transformed = np.zeros(6)
        W_transformed[:3] = np.dot(R,W[:3])
        W_transformed[3:] = np.dot(R,W[3:])

        return W_transformed

    # def transform_wrench2(self, W, T):
    #     # W: wrench 1D np array with 6 elements (Wa_in_a = [Ma_in_a, Fa_in_a]^T)
    #     # T: transform (T_b_to_a)
    #     # returns W_transformed: 1D np array with 6 elements (Wb_in_b)

    #     qw = T.transform.rotation.w # Scalar part of quaternion
    #     qx = T.transform.rotation.x
    #     qy = T.transform.rotation.y
    #     qz = T.transform.rotation.z
    #     q = [qx,qy,qz,qw]
    #     R = tf.transformations.quaternion_matrix(q)
    #     Rba = R[:3,:3]

    #     Pba_in_a = np.array([T.transform.translation.x,T.transform.translation.y,T.transform.translation.z])

    #     F = # force
    #     M = # torque

    #     W_transformed = np.zeros(6)
    #     W_transformed[:3] = np.dot(R,W[:3])
    #     W_transformed[3:] = np.dot(R,W[3:])

    #     return W_transformed

    def get_position_from_transform(self,T):
        return np.array([T.transform.translation.x,T.transform.translation.y,T.transform.translation.z])

    def transform_vector_rotate_only(self,V,T,inverse_tf=False):
        # WARNING: just rotates the vector with the given transform T, DOES NOT TRANSLATE!
        # V: 1D np array with size 3
        # T: transform
        # returns: 1D np array with size 3
        
        qw = T.transform.rotation.w # Scalar part of quaternion
        qx = T.transform.rotation.x
        qy = T.transform.rotation.y
        qz = T.transform.rotation.z
        q = [qx,qy,qz,qw]
        R = tf.transformations.quaternion_matrix(q)
        R = R[:3,:3]

        if inverse_tf:
            return np.dot(R.T,V)
        else:
            return np.dot(R,V)

if __name__ == '__main__':
    admittanceControllerCollaborative = AdmittanceControllerCollaborative()
    rospy.spin()