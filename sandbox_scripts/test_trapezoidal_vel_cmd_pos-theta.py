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

        self.cmd_vel_pub = rospy.Publisher("/husky_blue/cmd_vel", Twist, queue_size=1)
        self.direction = 1

        # rospy.Timer(rospy.Duration(0.5), self.switch)

        self.duration = 0.01 # loop duration

        self.timer_talk = WallTimer(self.duration,self.talk)

        num_rot = 1.0
        self.delta_x = num_rot*2.0*3.14159265359 # meters # Amount of displacement
        # OARBOTs Velocity/Acc limits (experimentally found)
        self.v_lim = 1.5
        self.a_lim = 0.6 #0.6
        
        self.t_acc, self.t_cons, self.t_dec = self.trap(self.v_lim, self.a_lim, self.delta_x)
        print(self.t_acc, self.t_cons, self.t_dec)

        self.t0 = None


    def trap(self,v_lim,a_lim,delta_x):
        v_lim = float(v_lim)
        a_lim = float(a_lim)
        delta_x = float(delta_x)

        t_acc = v_lim / a_lim
        t_cons = (delta_x/v_lim) - t_acc
        t_dec = t_acc

        return t_acc,t_cons,t_dec


    def talk(self):
        if self.t0 is None:
            self.t0 = time.time()

        t =  time.time() - self.t0 - 2.0 # current time since started

        if t < 0:
            v = 0.
        elif t < self.t_acc:
            v = t*self.a_lim
        elif t < self.t_acc + self.t_cons:
            v = self.v_lim
        elif t < self.t_acc + self.t_cons + self.t_dec:
            v = self.v_lim - (t-(self.t_acc + self.t_cons))*self.a_lim
        else:
            v = 0.


                
        msg = Twist()
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = v*(1.04431493763349/1.04431493763349)

        self.cmd_vel_pub.publish(msg)



if __name__ == "__main__":
    CmdVelTalker = CmdVelTalker()
    CmdVelTalker.timer_talk.start()
    # CmdVelTalker.timer_switch.start()
    rospy.spin()