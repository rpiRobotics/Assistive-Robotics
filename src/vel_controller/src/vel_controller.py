#!/usr/bin/env python

import numpy as np
import rospy

from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Joy

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

from assistive_msgs.msg import State2D # Custom message

import tf_conversions # quaternion stuff

import velocity_control_law  as vcl

'''
closed_loop_velocity_controller.py
Alex Elias

Drives a mobile robot to a desired position/velocity

Parameters:
	feedback_gain_xy: Proportional gain in (m/s) / m
	feedback_gain_theta: Proportional gain in (rad/s) / rad

	cmd_input_type: 'Twist' or 'State2D'
		Twist ==> Desired position is integrated (e.g. single robot control)
		State2D ==> Full state control (e.g. swarm control)
	cmd_input_topic_name: Desired input
		
	position_feedback_topic_name: Topic for position estimation (e.g. from sensor fusion)
	control_cmd_publish_topic_name: Topic for motor control input
	frame_name: Name used to send tf_frame of desired position
	is_offset_mode: True means that you are controlling only x and y, not theta

	vel_lim_x: Velocity limit in X
	vel_lim_y: Velocity limit in Y
	vel_lim_theta: Velocity limit in theta

	space_mouse_topic_name: e.g. 'spacenav/joy'
		Used for dead man's switch
'''

class Controller:
	def __init__(self):
		rospy.init_node('closed_loop_velocity_controller', anonymous=True)

		input_is_State2D = ("State2D" == rospy.get_param('~cmd_input_type'))
		
		cmd_input_topic_name = rospy.get_param('~cmd_input_topic_name')
		control_cmd_publish_topic_name = rospy.get_param('~control_cmd_publish_topic_name')
		space_mouse_topic_name = rospy.get_param('~space_mouse_topic_name')
		
		self.is_skid_steer_mode = rospy.get_param('~is_skid_steer_mode', False)
		
		if self.is_skid_steer_mode:
			position_feedback_topic_name = rospy.get_param('~turntable_position_topic_name')
		else:
			position_feedback_topic_name = rospy.get_param('~position_feedback_topic_name')

		self.frame_name = rospy.get_param('~frame_name')

		if self.is_skid_steer_mode is False:
			vel_lim_x = rospy.get_param('~vel_lim_x')
			vel_lim_y = rospy.get_param('~vel_lim_y')
			vel_lim_theta = rospy.get_param('~vel_lim_theta')
			self.vel_limit = np.array([[vel_lim_x],[vel_lim_y],[vel_lim_theta]])
			self.feedback_gain_theta = rospy.get_param('~feedback_gain_theta')

		self.feedback_gain_xy = rospy.get_param('~feedback_gain_xy')
		
		self.vel_cmd_pub = rospy.Publisher(control_cmd_publish_topic_name, Twist, queue_size=1)
		self.output_enable = False 
		self.state_pos = np.array([[0.0],[0.0],[0.0]])
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()

		
		if input_is_State2D:
			rospy.Subscriber(cmd_input_topic_name, State2D, self.desired_state_callback, queue_size=1)
		else:
			rospy.Subscriber(cmd_input_topic_name, Twist, self.desired_vel_callback, queue_size=1)
			# Integrated desired position
			self.q_desired = np.array([[0.0],[0.0],[0.0]])
			self.prev_integration_time = rospy.Time.now().to_sec()

		# Subscribe to space mouse buttons
		rospy.Subscriber(space_mouse_topic_name, Joy, self.space_mouse_button_callback, queue_size=1)

		# Subscribe to Kalman Filter position
		rospy.Subscriber(position_feedback_topic_name, Pose2D, self.state_feedback_callback, queue_size=1)

	def desired_vel_callback(self, data):
		current_time = rospy.Time.now().to_sec()
		dt = current_time - self.prev_integration_time
		self.prev_integration_time = current_time
		if(dt > 1):
			dt = 0

		# Data input is q_desired_dot
		q_desired_dot = np.array([[0.0],[0.0],[0.0]])
		q_desired_dot[0] = data.linear.x
		q_desired_dot[1] = data.linear.y
		q_desired_dot[2] = data.angular.z

		# Update q_desired with integration
		self.q_desired = self.q_desired + dt * q_desired_dot
		self.q_desired[2] = wrapToPi(self.q_desired[2])

		desired_state = np.block([[self.q_desired],[q_desired_dot]])
		self.process_desired_state(desired_state)
		# Publish TF frame
		tf_desired = xyt2TF(self.q_desired, "map", self.frame_name);
		self.tf_broadcaster.sendTransform(tf_desired)

	def desired_state_callback(self, data):
		desired_state = np.zeros((6,1)) 
		desired_state[0] = data.pose.x
		desired_state[1] = data.pose.y
		desired_state[2] = data.pose.theta
		desired_state[3] = data.twist.linear.x
		desired_state[4] = data.twist.linear.y
		desired_state[5] = data.twist.angular.z
		self.process_desired_state(desired_state)

	def process_desired_state(self, desired_state):
		if self.is_skid_steer_mode:
			K = self.feedback_gain_xy
			cmd_vel = vcl.control_law_skid_steer_mode(desired_state, self.state_pos,K)
		else:
			K = np.diag([self.feedback_gain_xy, self.feedback_gain_xy, self.feedback_gain_theta])
			cmd_vel = vcl.control_law(desired_state, self.state_pos, self.vel_limit, K)

		# Publish commanded velocity
		cmd_vel_msg = Twist()
		if(self.output_enable):
			cmd_vel_msg.linear.x = cmd_vel[0][0]
			cmd_vel_msg.linear.y = cmd_vel[1][0]
			cmd_vel_msg.angular.z = cmd_vel[2][0]
		else:
			cmd_vel_msg.linear.x = 0
			cmd_vel_msg.linear.y = 0
			cmd_vel_msg.angular.z = 0
		self.vel_cmd_pub.publish(cmd_vel_msg)

		

	def space_mouse_button_callback(self, data):
		if data.buttons[0]:
			self.output_enable = True
		elif data.buttons[1]:
			self.output_enable = False

	def state_feedback_callback(self, data):
		self.state_pos[0][0] = data.x
		self.state_pos[1][0] = data.y
		self.state_pos[2][0] = data.theta

def xyt2TF(xyt, header_frame_id, child_frame_id):
	'''
	Converts a numpy vector [x; y; z; theta]
	into a tf2_msgs.msg.TFMessage message
	'''
	xyt = xyt.flatten()

	t = geometry_msgs.msg.TransformStamped()

	t.header.frame_id = header_frame_id
	t.header.stamp = rospy.Time.now()
	t.child_frame_id = child_frame_id
	t.transform.translation.x = xyt[0]
	t.transform.translation.y = xyt[1]
	t.transform.translation.z = 0

	q = tf_conversions.transformations.quaternion_from_euler(0, 0,xyt[2])
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	return t

def wrapToPi(a):
	'''
	Wraps angle to [-pi,pi)
	'''
	return ((a+np.pi) % (2*np.pi))-np.pi

if __name__ == '__main__':
	Controller()
	rospy.spin()