#!/usr/bin/env python3

"""
auto_disable_ekf_localization.py

Author: Burak Aksoy
Date: 2024-07-03 (YYYY-MM-DD)

Description: This script disables the robot_localization package EKF localization when there is no signal from the sensors for a certain amount of time.
"""

import rospy

import rosgraph
import rostopic

import std_msgs.msg 
import sensor_msgs.msg 
import geometry_msgs.msg 
import nav_msgs.msg

from robot_localization.srv import ToggleFilterProcessing, ToggleFilterProcessingRequest

import numpy as np
import threading

class AutoDisableLocalization():
    def __init__(self):
        rospy.init_node('auto_disable_ekf_localization', anonymous=False)

        ## Parameters
        # Timeout for the last message from the sensors
        self.timeout = rospy.get_param('~timeout', 0.2)
        
        # Get the name of the localization node that we want to disable
        self.localization_node_name = rospy.get_param('~localization_node_name', '/human_localization_ekf_se')
        
        # Last message time from the sensors
        self.last_message_time = rospy.Time.now().to_sec()
                
        # List of topics that the localization node is subscribed to
        self.subscribed_topics = [] 
        # List of subscribers of this node to keep them alive
        self.subscribers = [] 

        # Flag to check if the all the auto disable parameters and subscribers are set
        self.is_auto_disable_set = False         
        # Flag to check if the node is disabled
        self.is_node_disabled = False 
        
        # Locks        
        self.setup_auto_disable_lock = threading.Lock()
        self.toggle_request_lock = threading.Lock()
        
        rospy.Timer(rospy.Duration(2.0*self.timeout), self.auto_disable_timer_callback)
        
    def auto_disable_timer_callback(self, event):
        if not self.is_auto_disable_set:
            if self.setup_auto_disable_lock.acquire(blocking=False):
                try:
                    self.is_auto_disable_set = self.setup_auto_disable()
                except:
                    rospy.logerr_throttle(10, "Error while setting up auto disable localization (Throttled 10s)")                    
                    self.is_auto_disable_set = False
                finally:
                    self.setup_auto_disable_lock.release()
        else:
            # Check if the last message time is greater than the timeout
            if (rospy.Time.now().to_sec() - self.last_message_time > self.timeout):
                # Check if the node is enabled
                if not self.is_node_disabled:
                    # Request to Disable the localization node
                    if self.toggle_request_lock.acquire(blocking=False):                    
                        try:
                            success = self.toggle_node_state(is_requested_to_enable=False)
                            if success:
                                self.is_node_disabled = True # If the node is disabled, set the flag to True
                        except:
                            rospy.logerr_throttle(10, "Error while DISABLING the localization node (Throttled 10s)")
                            self.is_node_disabled = False
                        finally:
                            self.toggle_request_lock.release()
                # Since the node is already disabled, Do nothing, keep the node disabled
                else:
                    pass
            # If the last message time is less than the timeout
            else:
                # Check if the node is disabled
                if self.is_node_disabled:
                    # Request to Enable the localization node
                    if self.toggle_request_lock.acquire(blocking=False):                    
                        try:
                            success = self.toggle_node_state(is_requested_to_enable=True)
                            if success:
                                self.is_node_disabled = False # If the node is enabled, set the flag to False
                        except:
                            rospy.logerr_throttle(10, "Error while ENABLING the localization node (Throttled 10s)")
                            self.is_node_disabled = True
                        finally:
                            self.toggle_request_lock.release()
                # Since the node is already enabled, Do nothing, keep the node enabled
                else:
                    pass

    def setup_auto_disable(self):
        # Find all the "relevant" topics that the localization node is subscribed to with the help of rosgraph
        self.subscribed_topics = self.get_node_subscriptions(self.localization_node_name)
        
        # print("Topics subscribed by node '{}':".format(self.localization_node_name))
        # print(self.subscribed_topics)
        
        # Create a subscriber for each topic with its respective message type
        for topic, msg_class in self.subscribed_topics:
            subscriber = rospy.Subscriber(topic, msg_class, self.message_callback)
            self.subscribers.append(subscriber)
        
        return True
    
    def get_node_subscriptions(self, node_name):
        master = rosgraph.Master(caller_id=rospy.get_name())
        node_subscriptions = []

        # Retrieve all system state information
        pubs, subs, _ = master.getSystemState()
        for topic, nodes in subs:
            if node_name in nodes:
                topic_type, real_topic, _ = rostopic.get_topic_type(topic)
                if topic_type:
                    msg_class, _, _ = rostopic.get_topic_class(real_topic)
                    if msg_class:
                        # Filter topics based on the message types that localization node is interested in
                        if msg_class._type in ['nav_msgs/Odometry',
                                                'geometry_msgs/PoseWithCovarianceStamped',
                                                'geometry_msgs/TwistWithCovarianceStamped',
                                                'sensor_msgs/Imu']:
                            node_subscriptions.append((topic, msg_class))
        if len(node_subscriptions) == 0:
            rospy.logwarn("No relevant topics found for node name:'{}'. Make sure the specified node name parameter is correct".format(node_name))
        return node_subscriptions
    
    def message_callback(self, msg):
        # Regardless of the message content, update the last message time
        self.last_message_time = rospy.Time.now().to_sec()
                        
    def toggle_node_state(self, is_requested_to_enable):
        # Request to enable or disable the localization node
        rospy.loginfo_throttle(10, "Request to {} the localization node '{}' (Throttled 10s)".format("ENABLE" if is_requested_to_enable else "DISABLE", self.localization_node_name))
        
        # Service address based on the localization node name
        toggle_service_address = self.localization_node_name + '/toggle'

        # Setup a service proxy to the 'toggle' service
        try:
            rospy.wait_for_service(toggle_service_address, timeout=1.0)
        except:
            rospy.logerr("Service '{}' is not available".format(toggle_service_address))
            return False
        
        toggle_service = rospy.ServiceProxy(toggle_service_address, ToggleFilterProcessing)

        # Create a request object
        request = ToggleFilterProcessingRequest(on=is_requested_to_enable)

        # Attempt to call the service with the created request
        try:
            response = toggle_service(request)
            if response.status:
                rospy.loginfo_throttle(10, "Successfully {} the localization node (Throttled 10s)".format(
                    "enabled" if is_requested_to_enable else "disabled"))
            return True
            # else:
            #     rospy.logwarn("Failed to {} the localization node; service reported false status".format(
            #         "enable" if is_requested_to_enable else "disable"))
            #     return False
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False
            
    
if __name__ == "__main__":
    autoDisableLocalization = AutoDisableLocalization()
    rospy.spin()