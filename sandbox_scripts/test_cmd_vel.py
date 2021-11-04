#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import time
import threading
class WallTimer():
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback

        self.last_start_time = time.time()

    def run(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            # rospy.loginfo(start_time - self.last_start_time)
            self.last_start_time = start_time

            self.callback()

            finish_time = time.time()
            if (finish_time - start_time) < self.duration:
                try:
                    time.sleep(self.duration*1.00 - (finish_time - start_time))
                except:
                    pass
            else:
                rospy.logwarn("The loop takes more than defined duration time: " + str(self.duration) +" seconds." )
        rospy.logwarn("ROS is shutdown" )

    def start(self):
        threading.Thread(target=self.run).start()


class CmdVelTalker():
    def __init__(self):
        rospy.init_node('cmd_vel_talker', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher("/tablet/deadman_switch_spacenav_twist", Twist, queue_size=1)
        self.direction = 1

        # rospy.Timer(rospy.Duration(0.5), self.switch)

        self.duration = 0.01 # loop duration
        self.duration2 = 1.0 # switch duration

        self.timer_talk = WallTimer(self.duration,self.talk)
        self.timer_switch = WallTimer(self.duration2, self.switch)


    def talk(self):    
        msg = Twist()
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = self.direction * 0.5        

        self.cmd_vel_pub.publish(msg)

    def switch(self):
        self.direction *= -1
        # rospy.logwarn("Direction changed" )


if __name__ == "__main__":
    CmdVelTalker = CmdVelTalker()
    CmdVelTalker.timer_talk.start()
    CmdVelTalker.timer_switch.start()
    rospy.spin()