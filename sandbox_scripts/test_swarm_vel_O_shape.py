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

        self.cmd_vel_pub = rospy.Publisher("/desired_swarm_vel", Twist, queue_size=1)
        self.direction = 1

        # rospy.Timer(rospy.Duration(0.5), self.switch)

        self.duration = 0.01 # loop duration

        self.timer_talk = WallTimer(self.duration,self.talk)

        self.is_t_calculated = False
        
        self.is_done_1 = False
        self.is_done_2 = False
        self.is_done_3 = False
        self.is_done_4 = False


    def trap(self,v_lim,a_lim,delta_x):
        v_lim = float(v_lim)
        a_lim = float(a_lim)
        delta_x = float(delta_x)

        t_acc = v_lim / a_lim
        t_cons = (delta_x/v_lim) - t_acc
        t_dec = t_acc

        return t_acc,t_cons,t_dec

    def command(self,direction_vec):
        if self.t0 is None:
            self.t0 = time.time()

        t =  time.time() - self.t0 - self.t_pause # current time since started

        if t < 0:
            v = 0.
        elif t < self.t_acc:
            v = t*self.a_lim
        elif t < self.t_acc + self.t_cons:
            v = self.v_lim
        elif t < self.t_acc + self.t_cons + self.t_dec:
            v = self.v_lim - (t-(self.t_acc + self.t_cons))*self.a_lim
        else:
            self.is_t_calculated = False
            return True
            
        msg = Twist()
        msg.linear.x = direction_vec[0]*v
        msg.linear.y = direction_vec[1]*v
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = direction_vec[2]*v

        self.cmd_vel_pub.publish(msg)
        return False


    def talk(self):
        if self.is_done_1 == False:
            x_dir = +0.
            y_dir = +1.
            th_dir = +0.

            if self.is_t_calculated == False:
                self.delta_x = 1.5 # meters # Amount of displacement
                # OARBOTs Velocity/Acc limits (experimentally found)
                self.v_lim = 0.1
                self.a_lim = 0.3 #0.6

                self.t_pause = 4.0
                
                self.t_acc, self.t_cons, self.t_dec = self.trap(self.v_lim, self.a_lim, self.delta_x)
                self.is_t_calculated = True
                print(self.t_acc, self.t_cons, self.t_dec)

                self.t0 = None

            self.is_done_1 = self.command(direction_vec=[x_dir,y_dir,th_dir])
        
        elif self.is_done_2 == False:
            x_dir = +1.
            y_dir = +0.
            th_dir = +0.

            if self.is_t_calculated == False:
                self.delta_x = 3.0 # meters # Amount of displacement
                # OARBOTs Velocity/Acc limits (experimentally found)
                self.v_lim = 0.1
                self.a_lim = 0.3 #0.6

                self.t_pause = 4.0
                
                self.t_acc, self.t_cons, self.t_dec = self.trap(self.v_lim, self.a_lim, self.delta_x)
                self.is_t_calculated = True
                print(self.t_acc, self.t_cons, self.t_dec)

                self.t0 = None

            self.is_done_2 = self.command(direction_vec=[x_dir,y_dir,th_dir])

        elif self.is_done_3 == False:
            x_dir = +0.
            y_dir = -1.
            th_dir = +0.

            if self.is_t_calculated == False:
                self.delta_x = 1.5 # meters # Amount of displacement
                # OARBOTs Velocity/Acc limits (experimentally found)
                self.v_lim = 0.1
                self.a_lim = 0.3 #0.6

                self.t_pause = 4.0
                
                self.t_acc, self.t_cons, self.t_dec = self.trap(self.v_lim, self.a_lim, self.delta_x)
                self.is_t_calculated = True
                print(self.t_acc, self.t_cons, self.t_dec)

                self.t0 = None

            self.is_done_3 = self.command(direction_vec=[x_dir,y_dir,th_dir])

        elif self.is_done_4 == False:
            x_dir = -1.
            y_dir = +0.
            th_dir = +0.

            if self.is_t_calculated == False:
                self.delta_x = 3.0 # meters # Amount of displacement
                # OARBOTs Velocity/Acc limits (experimentally found)
                self.v_lim = 0.1
                self.a_lim = 0.3 #0.6

                self.t_pause = 4.0
                
                self.t_acc, self.t_cons, self.t_dec = self.trap(self.v_lim, self.a_lim, self.delta_x)
                self.is_t_calculated = True
                print(self.t_acc, self.t_cons, self.t_dec)

                self.t0 = None

            self.is_done_4 = self.command(direction_vec=[x_dir,y_dir,th_dir])

        


if __name__ == "__main__":
    CmdVelTalker = CmdVelTalker()
    CmdVelTalker.timer_talk.start()
    # CmdVelTalker.timer_switch.start()
    rospy.spin()