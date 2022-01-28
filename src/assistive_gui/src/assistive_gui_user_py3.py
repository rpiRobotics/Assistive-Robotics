#!/usr/bin/env python3
import os
import rospy
import rospkg
import collections
import time
import sys
import subprocess
import numpy as np
from qt_gui.plugin import Plugin
#from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog
from python_qt_binding.QtCore import QMutex, QMutexLocker,QSemaphore, QThread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtWidgets, uic
from std_msgs.msg import Bool, Int32
import threading
from geometry_msgs.msg import Pose2D, Twist, PoseStamped
from led_indicator import LEDIndicator
import rosnode
import signal
import std_msgs.msg
from assistive_msgs.msg import FrameTwist
from tf import TransformListener
import tf_conversions

callback_lock=threading.Lock()

#class robot_button(QtWidgets.QPushButton):
class swarm_button:
    def __init__(self,button,topic):
        self.button=button
        self.publisher=rospy.Publisher(topic, Twist, queue_size=0)
        self.button.pressed.connect(self.button_pressed)
        self.enabled=False

    def button_pressed(self):
        self.enabled=not(self.enabled)
        
        if(self.enabled):
            
            self.button.setStyleSheet('QPushButton {background-color: orange; color: white;}')
            
        else:
            
            self.button.setStyleSheet('QPushButton {background-color: white; color: black;}')

class robot_button:
    def __init__(self,number,topic,commandmode,sizex,sizey,text):
        if commandmode == "whole_robot":
            self.text=text+"\nOarbot Enable"
        elif commandmode == "arm_only":
            self.text=text+" Only \nArm Enable"
        elif commandmode == "base_only":
            self.text=text+" Only \nBase Enable"
        self.enabled=False
        # self.motion_frame="world"
        self.button=QPushButton()
        self.button.setFixedSize(sizex,sizey)
        self.button.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding)
        self.button.setFont(QFont('Ubuntu',13))
        self.button.setText(self.text)
        self.button.pressed.connect(self.button_pressed)
        
        self.publisher=rospy.Publisher(topic, Twist, queue_size=0)
        #self.button2=QPushButton()
        #self.button2.setFont(QFont('Ubuntu',11))
        #self.button2.setText(self.text2)
        #self.button2.pressed.connect(self.button_pressed2)
    # def publish_out_message(self):
    #     message=FrameTwist()
    #     h=std_msgs.msg.Header()
    #     h.stamp=rospy.Time.now()
    #     message.header=h
    #     message.motion_frame=self.motion_frame
        
    def button_pressed(self):
        self.enabled=not(self.enabled)
        
        if(self.enabled):
            
            self.button.setStyleSheet('QPushButton {background-color: orange; color: white;}')
            
        else:
            
            self.button.setStyleSheet('QPushButton {background-color: white; color: black;}')
    
    # def button_pressed2(self):
    #     pass
    
class LEDManager:
    def __init__(self,nodenames,led_objects):
        self.nodenames=nodenames
        self.led_objects=led_objects
        self.active_bots=[]
        self.publisher=rospy.Publisher("robot_enable_status",Int32,queue_size=10)
        self.send_value=0
        for i in range(len(led_objects)):
            self.active_bots.append(False)
            
    def poll_node_names(self):
        #nodenames is loaded from yaml file and should be a list of lists for each robot of desired nodes
        #print(self.nodenames)
        for i in range(len(self.led_objects)):
            if(self.active_bots[i]!=self.led_objects[i].active):
                if(self.led_objects[i].active==True):
                    out=pow(2,i)
                    self.send_value+=out
                    self.active_bots[i]=True
                else:
                    out=pow(2,i)
                    self.send_value-=out
                    self.active_bots[i]=False

                output=Int32()
                output.data=int(self.send_value)
                self.publisher.publish(output)
        time.sleep(0.01)
            
class SWARMGUI(QtWidgets.QMainWindow):
    resized = pyqtSignal()
    def __init__(self):
        super(SWARMGUI, self).__init__()
        self.buttons=[]
        self.labels=[]
        self.setObjectName('MyPlugin')
        self.synced_control_enabled=False
        self.mainscreenui = os.path.join(rospkg.RosPack().get_path('assistive_gui'), 'resource', 'mainwindow.ui')
        uic.loadUi(self.mainscreenui, self)
        desktop=QtWidgets.QDesktopWidget()
        screengeometry=desktop.screenGeometry()
        height=screengeometry.height()
        width=screengeometry.width()
        self.tf = TransformListener()

        #print(type(width))
        heightnew=height//3
        
        #self.setMinimumSize(width-50,heightnew)
        #self.setMaximumSize(width-50,heightnew)
        self.setGeometry(0,0,width-50,heightnew)
        layout=self.layout()
        #self.setMaximumSize(width-70,heightnew)
        #self.setWindowState(Qt.WindowMinimized)
        #self.setWindowState(Qt.WindowActive)
        #self.showMinimized()
        #layout.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.move(70,height-heightnew)
        self.resized.connect(self.windowresized)
        #self.Moveswarm.pressed.connect(self.sync_robot_motion_pressed)
        #self.Moveswarmframe.pressed.connect(self.move_swarm_frame)
        #self.Robot1enable.pressed.connect(self.rob1enable)
        #self.Robot2enable.pressed.connect(self.rob2enable)
        #self.Robot3enable.pressed.connect(self.rob3enable)
        #self.rob1en=False
        #self.rob2en=False
        #self.rob3en=False
        
        #self.pub1=rospy.Publisher('/spacenav/twist/repub', Twist, queue_size=10)
        #self.pub3=rospy.Publisher('/spacenav/twist/repub3', Twist, queue_size=10)
        #self.pub2=rospy.Publisher('/spacenav/twist/repub2', Twist, queue_size=10)
        
        
        #rospy.Subscriber("/OARBOT1/pose", Pose2D, callback) 
        
        self.Leds=[]

        #self.number_of_bots=3
        #self.number_of_bots=rospy.get_param('number_of_robots')
        try:
            self.number_of_bots=rospy.get_param('number_of_robots')
            self.nodenames=rospy.get_param('robot_node_names')

            self.whole_robot_open_loop_command_topics=rospy.get_param('whole_robot_open_loop_command_topics')
            self.arm_only_open_loop_command_topics=rospy.get_param('arm_only_open_loop_command_topics')
            self.base_only_open_loop_command_topics=rospy.get_param('base_only_open_loop_command_topics')

            self.input_command_topic=rospy.get_param('input_command_topic')
            self.robot_types=rospy.get_param('robot_type_information')
            self.closed_loop_swarm_command_topic=rospy.get_param('closed_loop_swarm_command_topic')
            self.open_loop_swarm_command_topic=rospy.get_param('open_loop_swarm_command_topic')
            #self.sync_topic=rospy.get_param('sync_frames_topic')
            self.swarm_tf=rospy.get_param('swarm_tf_frame')
            self.robot_tfs=rospy.get_param('robot_tf_frames')
            self.real_robot_tfs=rospy.get_param('real_robot_tf_frames')
            self.resize_swarm_scaling_factor=float(rospy.get_param('resize_scaling_factor'))
            self.tf_changer_topic=rospy.get_param('tf_changer_topic')
        except:
            self.number_of_bots=3
            self.nodenames=[["/rosout"],["hello"],["hello"]]
            self.command_topics=["/spacenav/twist/repub","/spacenav/twist/repub2","/spacenav/twist/repub3","hello"]
            self.input_command_topic='deadman_switch_spacenav_twist'

        #self.syncpub=rospy.Publisher(self.sync_topic,Bool,queue_size=10)
        self.syncFrames.pressed.connect(self.sync_frames)
        self.moveswarmbutton = swarm_button(self.Moveswarm,self.closed_loop_swarm_command_topic)
        self.moveswarmframebutton = swarm_button(self.Moveswarmframe,self.open_loop_swarm_command_topic)
        self.buttons.append(self.moveswarmbutton)
        self.buttons.append(self.moveswarmframebutton)
        rospy.Subscriber(self.input_command_topic, Twist, self.offset_callback)
        
        for i in range(self.number_of_bots):
            #led=LEDIndicator()
            led=LEDIndicator(i)
            #led.setDisabled(True)
            
            self.Robotlayout.addWidget(led,4,i)
            self.Leds.append(led)
            led.led_change(True)
    
        self.status_manager=LEDManager(self.nodenames,self.Leds)
        
        
        buttonwidth=width-100
        for x in range(self.number_of_bots):
            
            if(len(self.robot_types)!=self.number_of_bots):
                print("NOT ALL ROBOTS HAVE BEEN GIVEN TYPE INFORMATION")
                break
            
            robot_label=QLabel()
            robot_label.setFixedSize(buttonwidth//self.number_of_bots,100)
            
            robot_label.setAlignment(Qt.AlignCenter)
            robot_label.setText(self.robot_types[x])
            robot_label.setFont(QFont('Ubuntu',13))
            self.Robotlayout.addWidget(robot_label,0,x)
            self.labels.append(robot_label)
            
                
        
        #print(self.robot_types[1] is "Holonomic")
        for i in range(self.number_of_bots):
            if(len(self.whole_robot_open_loop_command_topics) != self.number_of_bots and len(self.whole_robot_open_loop_command_topics)!=len(self.arm_only_open_loop_command_topics)):
                print("NOT ENOUGH COMMAND TOPICS GIVEN FOR NUMBER OF ROBOTS")
                break
            
            
            button_class_object=robot_button(i,self.whole_robot_open_loop_command_topics[i],"whole_robot",buttonwidth//self.number_of_bots,heightnew//8,self.robot_types[i])
            self.Robotlayout.addWidget(button_class_object.button,1,i)
            self.buttons.append(button_class_object)

            button_class_object2=robot_button(i,self.arm_only_open_loop_command_topics[i],"arm_only",buttonwidth//self.number_of_bots,heightnew//8,self.robot_types[i])
            self.Robotlayout.addWidget(button_class_object2.button,2,i)
            self.buttons.append(button_class_object2)

            button_class_object3=robot_button(i,self.base_only_open_loop_command_topics[i],"base_only",buttonwidth//self.number_of_bots,heightnew//8,self.robot_types[i])
            self.Robotlayout.addWidget(button_class_object3.button,3,i)
            self.buttons.append(button_class_object3)

        """
        self.robot1led=LEDIndicator()
        self.robot1led.setDisabled(True)
        self.robot2led=LEDIndicator()
        self.robot2led.setDisabled(True)
        self.robot3led=LEDIndicator()
        self.robot3led.setDisabled(True)
        
        self.Robotlayout.addWidget(self.robot1led,1,0)
        self.Robotlayout.addWidget(self.robot2led,1,1)
        self.Robotlayout.addWidget(self.robot3led,1,2)
        """
        rospy.Timer(rospy.Duration(0.1), self.callback_gui)
        #needed_robot_nodes=[]
        self.plus = os.path.join(rospkg.RosPack().get_path('assistive_gui'), 'resource', '+.PNG')
        self.minus = os.path.join(rospkg.RosPack().get_path('assistive_gui'), 'resource', '-.PNG')
        icon=QIcon()
        icon.addPixmap(QPixmap(self.plus))
        self.plusbutton.setIcon(icon)
        self.plusbutton.setIconSize(QSize(100,100))
        self.plusbutton.pressed.connect(self.expand_structure)
        icon2=QIcon()
        icon2.addPixmap(QPixmap(self.minus))
        self.minusbutton.setIcon(icon2)
        self.minusbutton.setIconSize(QSize(100,100))
        self.minusbutton.pressed.connect(self.shrink_structure)
        rp = rospkg.RosPack()
        self.package_path = rp.get_path('assistive_gui')
        #self.plusbutton.resize(width/3,height/5)
        #self.minusbutton.resize(width/3,height/5)
        self.rotation_disabled=False
        self.Disablerotation.pressed.connect(self.disable_rotation)
        #self.Savestructure.resize(width/3,height/5)
        #self.repubme=rospy.Publisher(self.input_command_topic, Twist, queue_size=0)
        #rospy.Timer(rospy.Duration(0.1), self.move_swarm_frame)
        self.Savestructure.pressed.connect(self.save_structure)
        self.Loadstructure.pressed.connect(self.load_structure)
        #self.Assumestructure.pressed.connect(self.assume_structure)
        self.tf_changer=rospy.Publisher(self.tf_changer_topic,PoseStamped,queue_size=10)
        self.windowresized()
        self.show()


    def disable_rotation(self):
        self.rotation_disabled=not(self.rotation_disabled)
        if(self.rotation_disabled):
            
            self.Disablerotation.setStyleSheet('QPushButton {background-color: orange; color: white;}')
            
        else:
            
            self.Disablerotation.setStyleSheet('QPushButton {background-color: white; color: black;}')    
   
    def callback_gui(self,evt):
        self.status_manager.poll_node_names()
        """
        for node_name in rosnode.get_node_names():
            print(node_name)
        if('/rosout' in rosnode.get_node_names()):
            self.robot1led.led_change(True)
        else:
            self.robot1led.led_change(False)
        """
    def move_swarm_frame(self,evt):
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=0
        self.repubme.publish(message)

    def expand_structure(self):
        for i in range(self.number_of_bots):
            if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(self.robot_tfs[i]):
                t = self.tf.getLatestCommonTime(self.robot_tfs[i], self.swarm_tf)
                trans, quaternions = self.tf.lookupTransform(self.swarm_tf,self.robot_tfs[i], t)
                p1 = PoseStamped()
                p1.header.frame_id = self.robot_tfs[i]
                #rospy.logwarn(str(trans[0]))
                trans[0]+=trans[0]*self.resize_swarm_scaling_factor
                trans[1]+=trans[1]*self.resize_swarm_scaling_factor
                #rospy.logwarn(str(trans[0]))
                p1.pose.position.x = float(trans[0])
                p1.pose.position.y = float(trans[1])
                p1.pose.position.z = float(trans[2])
                p1.pose.orientation.w = float(quaternions[3])
                p1.pose.orientation.x = float(quaternions[0])
                p1.pose.orientation.y = float(quaternions[1])
                p1.pose.orientation.z = float(quaternions[2])
                #p_in_base = self.tf.transformPose("/base_link", p1)
                self.tf_changer.publish(p1)


    def shrink_structure(self):
        for i in range(self.number_of_bots):
            if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(self.robot_tfs[i]):
                t = self.tf.getLatestCommonTime(self.robot_tfs[i], self.swarm_tf)
                trans, quaternions = self.tf.lookupTransform(self.swarm_tf,self.robot_tfs[i], t)
                p1 = PoseStamped()
                p1.header.frame_id = self.robot_tfs[i]
                trans[0]-=trans[0]*self.resize_swarm_scaling_factor
                trans[1]-=trans[1]*self.resize_swarm_scaling_factor
                p1.pose.position.x = float(trans[0])
                p1.pose.position.y = float(trans[1])
                p1.pose.position.z = float(trans[2])
                p1.pose.orientation.w = float(quaternions[3])
                p1.pose.orientation.x = float(quaternions[0])
                p1.pose.orientation.y = float(quaternions[1])
                p1.pose.orientation.z = float(quaternions[2])
                #p_in_base = self.tf.transformPose("/base_link", p1)
                self.tf_changer.publish(p1)

    def offset_callback(self,data):
        with callback_lock:
            if(self.rotation_disabled):
                data.angular.x = data.angular.x*0.
                data.angular.y = data.angular.y*0.
                data.angular.z = data.angular.z*0.
            else:
                data.angular.x = data.angular.x*1.0
                data.angular.y = data.angular.y*1.0
                data.angular.z = data.angular.z*1.0
            for i in range(len(self.buttons)):
                if(self.buttons[i].enabled):
                    self.buttons[i].publisher.publish(data)
           
    
            
    def sync_frames(self):
        for i in range(self.number_of_bots):
            if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(self.real_robot_tfs[i]):
                t = self.tf.getLatestCommonTime(self.real_robot_tfs[i], self.swarm_tf)
                (trans,quaternions) = self.tf.lookupTransform(self.swarm_tf,self.real_robot_tfs[i],t)
                p1 = PoseStamped()
                p1.header.frame_id = self.robot_tfs[i]
                
                p1.pose.position.x = float(trans[0])
                p1.pose.position.y = float(trans[1])
                p1.pose.position.z = float(trans[2])
                p1.pose.orientation.w = float(quaternions[3])
                p1.pose.orientation.x = float(quaternions[0])
                p1.pose.orientation.y = float(quaternions[1])
                p1.pose.orientation.z = float(quaternions[2])
                #p_in_base = self.tf.transformPose("/base_link", p1)
                self.tf_changer.publish(p1)

        

    def sync_robot_motion_pressed(self):
        for i in range(len(self.buttons)):
            self.synced_control_enabled=True
            if(not self.buttons[i].enabled):
                self.synced_control_enabled=False
                break
                
        #if(self.rob1en and self.rob2en and self.rob3en): self.synced_control_enabled=True
        self.synced_control_enabled=not(self.synced_control_enabled)
        
        if(self.synced_control_enabled):
            for i in range(len(self.buttons)):
                self.buttons[i].enabled=True
                self.buttons[i].button.setStyleSheet('QPushButton {background-color: orange; color: white;}')
            
            
        else:
            for i in range(len(self.buttons)):
                self.buttons[i].enabled=False
                self.buttons[i].button.setStyleSheet('QPushButton {background-color: white; color: black;}')
            
            
    #def motion_callback(self):
    

    def resizeEvent(self, event):
        self.resized.emit()
        
    def windowresized(self):
        windowwidth=self.rect().width()
        windowheight=self.rect().height()
        f=QFont('',windowwidth//110)
        for i in range(len(self.buttons)):
            self.buttons[i].button.setFont(f)
        for i in range(len(self.labels)):
            self.labels[i].setFont(f)
        self.label.setFont(f)
        self.Savestructure.setFont(f)
        self.Loadstructure.setFont(f)
        self.Disablerotation.setFont(f)
        #self.Assumestructure.setFont(f)
        self.syncFrames.setFont(f)

    def save_structure(self):
        name, done1 = QtWidgets.QInputDialog.getText(
             self, 'Save Structure', 'Enter desired save name:')
        if(done1):
            name=self.package_path+'/resource/'+name+'.txt'
            f = open(name, "w")
            
            for i in range(len(self.robot_tfs)):
                if(self.tf.frameExists(self.robot_tfs[i])):
                    t = self.tf.getLatestCommonTime(self.robot_tfs[i], self.swarm_tf)
                    position, quaternion = self.tf.lookupTransform(self.robot_tfs[i], self.swarm_tf, t)
                    
                    print(position, quaternion)
                    f.write("robot_name: %s\n"%self.robot_tfs[i])
                    f.write(str(position)+"\n")
                    f.write(str(quaternion)+"\n")
            f.close()

    def load_structure(self):
        name, done1 = QtWidgets.QInputDialog.getText(
             self, 'Load Structure', 'Enter desired file name:')
        if(done1):
            name=self.package_path+'/resource/'+name+'.txt'
            rospy.logwarn("assistive_gui_user.py: line 368: "+ name)
            f = open(name, "r+")
            lines=f.readlines()
            length=len(lines)//3
            for i in range(length):
                index=3*i
                frame_name=lines[index][12:].strip()
                position_line=lines[index+1][1:-2]
                quaternion_line=lines[index+2][1:-2]
                rospy.loginfo(frame_name)
                if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(frame_name):
                    rospy.loginfo(str(i))
                    #t = self.tf.getLatestCommonTime(self.swarm_tf, frame_name)
                    p1 = PoseStamped()
                    p1.header.frame_id = frame_name
                    positions=position_line.split(', ')
                    quaternions=quaternion_line.split(', ')
                    p1.pose.position.x = float(positions[0])
                    p1.pose.position.y = float(positions[1])
                    p1.pose.position.z = float(positions[2])
                    p1.pose.orientation.w = float(quaternions[3])
                    p1.pose.orientation.x = float(quaternions[0])
                    p1.pose.orientation.y = float(quaternions[1])
                    p1.pose.orientation.z = float(quaternions[2])
                    #p_in_base = self.tf.transformPose("/base_link", p1)
                    self.tf_changer.publish(p1)

    
                
    

def main():
    rospy.init_node('assistive_gui',log_level=rospy.DEBUG)
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv) # Create an instance of QtWidgets.QApplication
    window = SWARMGUI() # Create an instance of our class
    window.show()
    sys.exit(app.exec_()) # Start the application

if __name__== '__main__':
    main()

