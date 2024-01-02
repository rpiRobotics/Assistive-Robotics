#!/usr/bin/env python

"""
Author: Burak Aksoy
Description:
    The LogController class provides a way to control logging in ROS. 
    It allows for logging messages once, or at specified intervals (minimum periods). 
    This is useful for reducing log clutter and controlling the frequency of log messages.

Usage:
    logger = LogController()
    logger.log("This is an error message", log_type='error', log_once=True)
    logger.log("This is a warning message", log_type='warning', min_period=10)
    logger.log("This is an info message", log_type='info', log_once=True)
    logger.reset_once("This is an error message", log_type='error')
"""

import rospy
import time

class LogController:
    def __init__(self):
        self.last_logged_time = {}
        self.min_period = {}
        self.logged_once = {}

    def log(self, message, log_type, min_period=None, log_once=False):
        current_time = time.time()

        if log_once:
            if self.logged_once.get((message, log_type), False):
                return
            self.logged_once[(message, log_type)] = True

        if min_period is not None:
            last_time = self.last_logged_time.get((message, log_type), 0)
            if current_time - last_time < min_period:
                return

        self._log_message(message, log_type)
        self.last_logged_time[(message, log_type)] = current_time

    def _log_message(self, message, log_type):
        if log_type == 'error':
            rospy.logerr(message)
        elif log_type == 'warning':
            rospy.logwarn(message)
        elif log_type == 'info':
            rospy.loginfo(message)

    def reset_once(self, message, log_type):
        self.logged_once[(message, log_type)] = False
