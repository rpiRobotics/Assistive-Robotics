#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped,TransformStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from kinova_msgs.msg import JointVelocity, PoseVelocity
from oarbot_moveit.oarbot_moveit import Oarbot
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np
from math import pi
from copy import deepcopy as dp

class baseState(object):
    def __init__(self) -> None:
        super().__init__()

        # Parameters
        self.home_q = np.array([0,0,0,0,0,pi/8,-pi/8,pi/6,pi/6,pi/6,0])

        # Variables
        self.joint_base_start = None
        self.joint_sup_start = None
        self.joint_arm_start = None
        self.joint_finger_start = None

        self.joint_state = None
        self.base_vel = None
        self.sup_vel = None
        self.arm_joint_vel = None

        self.bot = Oarbot()

        # Subscriber for joint state
        self.joint_sub = rospy.Subscriber("joint_states",JointState,self.joint_cb,queue_size=1)

        rospy.sleep(rospy.Duration(3))

        # Publishers
        self.joint_pub = rospy.Publisher("to_joint_states",JointState,queue_size=1)

        # Subscribers
        self.pose_sub = rospy.Subscriber("oarbot_base_pose",PoseStamped,self.pose_cb,queue_size=1)

        # Go home
        self.go_home()

        self.base_vel_sub = rospy.Subscriber("cmd_vel",Twist,self.base_vel_cb,queue_size=1)
        self.sup_vel_sub = rospy.Subscriber("sup_vel",Float64,self.sup_vel_cb,queue_size=1)
        self.joint_vel_sub = rospy.Subscriber("joint_vel",JointVelocity,self.joint_vel_cb,queue_size=1)
        # self.ee_vel_sub = rospy.Subscriber("arm_cartesian_vel",PoseVelocity,self.ee_vel_cb,queue_size=1)
        self.ee_vel_sub = rospy.Subscriber("arm_cartesian_vel",Twist,self.ee_vel_cb,queue_size=1)
        
        # Service Server
        self.home_srv = rospy.Service('go_home',Trigger,self.home_srv_cb)

        # Control Loop Timer
        self.c_rate = 0.01
        self.ctrl_timer = rospy.Timer(rospy.Duration(self.c_rate),self.control_timer)

    def control_timer(self, event):
        
        if self.joint_base_start is None:
            return
        
        if self.base_vel is not None:
            dx = self.base_vel.linear.x*self.c_rate
            dy = self.base_vel.linear.y*self.c_rate
            dth = self.base_vel.angular.z*self.c_rate
            joint_array = np.asarray(self.joint_state.position)
            joint_array[self.joint_base_start] += dx
            joint_array[self.joint_base_start+1] += dy
            joint_array[self.joint_base_start+2] += dth
            self.joint_state.position = tuple(joint_array)
        if self.sup_vel is not None:
            dh = self.sup_vel*self.c_rate
            joint_array = np.asarray(self.joint_state.position)
            joint_array[self.joint_sup_start] += dh
            self.joint_state.position = tuple(joint_array)
        if self.arm_joint_vel is not None:
            dq = self.arm_joint_vel*self.c_rate
            joint_array = np.asarray(self.joint_state.position)
            joint_array[self.joint_arm_start:self.joint_arm_start+6] += dq
            self.joint_state.position = tuple(joint_array)

        joint_msg = dp(self.joint_state)
        joint_array = np.asarray(joint_msg.position)
        joint_array[self.joint_arm_start:self.joint_arm_start+6] = joint_array[self.joint_arm_start:self.joint_arm_start+6]+self.bot.q_zeros[4:10]
        joint_msg.position = tuple(joint_array)
        joint_msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(joint_msg)

        self.base_vel = None
        self.sup_vel = None
        self.arm_joint_vel = None
    
    def joint_cb(self,msg):

        if self.joint_arm_start is None:
            for i in range(len(msg.name)):
                if msg.name[i] == 'j2n6s300_joint_1':
                    self.joint_arm_start = i
                elif msg.name[i] == 'connect_base_and_world_x':
                    self.joint_base_start = i
                elif msg.name[i] == 'base_to_support':
                    self.joint_sup_start = i
                elif msg.name[i] == 'j2n6s300_joint_finger_1':
                    self.joint_finger_start = i
            self.joint_state = msg
    
    def base_vel_cb(self,msg):
        self.base_vel = msg
    
    def sup_vel_cb(self,msg):
        self.sup_vel = msg.data
    
    def joint_vel_cb(self,msg):
        self.arm_joint_vel = msg
    
    def ee_vel_cb(self,msg):

        if self.joint_state is None:
            return
        J = self.bot.arm_jacobian(self.joint_state.position[self.joint_arm_start:self.joint_arm_start+6])
        nu = np.array([msg.angular.x,msg.angular.y,msg.angular.z,msg.linear.x,msg.linear.y,msg.linear.z])
        self.arm_joint_vel = np.reshape(np.matmul(np.linalg.pinv(J),np.reshape(nu,(6,1))),(6,))

        # print("J",J)
        # print("nu",nu)
        # print("vel",self.arm_joint_vel)

    def pose_cb(self, msg):
        pass

    def home_srv_cb(self, req):
        self.go_home()

        return TriggerResponse(
            success=True,
            message="Go home"
        )
    
    def go_home(self):
        self.set_pose(self.home_q)

    def set_pose(self,q):

        if self.joint_state is None:
            return

        if len(q) != 11:
            rospy.logwarn("Oarbot has 11 joint (including finger). Ignoring this msg.")
            return
        pass

        joint_msg = self.joint_state
        joint_array = np.asarray(joint_msg.position)
        joint_array[self.joint_base_start] = q[0]
        joint_array[self.joint_base_start+1] = q[1]
        joint_array[self.joint_base_start+2] = q[2]
        joint_array[self.joint_sup_start] = q[3]
        joint_array[self.joint_arm_start:self.joint_arm_start+6] = q[4:10]
        joint_array[self.joint_finger_start] = q[10]
        joint_array[self.joint_finger_start+2] = q[10]
        joint_array[self.joint_finger_start+4] = q[10]

        joint_msg.position = tuple(joint_array)
        self.joint_state = joint_msg

if __name__ == '__main__':
    rospy.init_node('oarbot_base_pose')
    p = baseState()
    rospy.spin()