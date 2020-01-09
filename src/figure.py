#!/usr/bin/env python

import rospy
from consensus.msg  import triggertime
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt



class Figure():
    def __init__(self):
        rospy.init_node('figure', anonymous=True)
        self.x0 = []
        self.y0 = []
        self.x1 = []
        self.y1 = []
        self.x2 = []
        self.y2 = []
        self.x3 = []
        self.y3 = []

        self.a1 = []
        self.a2 = []
        self.a3 = []
        self.a4 = []


        self.tr1 = []
        self.tr2 = []
        self.tr3 = []
        self.tr4 = []
        
        rospy.Subscriber("tb3_0/odom", Odometry, self.callback0)
        rospy.Subscriber("tb3_1/odom", Odometry, self.callback1)
        rospy.Subscriber("tb3_2/odom", Odometry, self.callback2)
        rospy.Subscriber("tb3_3/odom", Odometry, self.callback3)

        rospy.Subscriber("triggertime1", triggertime, self.trcallback1)
        rospy.Subscriber("triggertime2", triggertime, self.trcallback2)
        rospy.Subscriber("triggertime3", triggertime, self.trcallback3)
        rospy.Subscriber("triggertime4", triggertime, self.trcallback4)
        
        rospy.spin() 
        self.plotfigure()
    def callback0(self, data):
        #plt.figure()
        
        self.x0.append(data.pose.pose.position.x)  
        self.y0.append(data.pose.pose.position.y) 
    def callback1(self, data):
        self.x1.append(data.pose.pose.position.x)  
        self.y1.append(data.pose.pose.position.y)   
    
    
    def callback2(self, data):
        self.x2.append(data.pose.pose.position.x)  
        self.y2.append(data.pose.pose.position.y) 
    
    def callback3(self, data):
        self.x3.append(data.pose.pose.position.x)  
        self.y3.append(data.pose.pose.position.y) 

    def trcallback1(self,data):
        rospy.loginfo("get data")
        self.tr1.append(data.trigger)
        self.a1.append(data.sample)
   
    def trcallback2(self,data):
        self.tr2.append(data.trigger)
        self.a2.append(data.sample)
        
    def trcallback3(self,data):
        self.tr3.append(data.trigger)
        self.a3.append(data.sample)
       
    def trcallback4(self,data):
        self.tr4.append(data.trigger)
        self.a4.append(data.sample)
       

    def plotfigure(self):
        rospy.loginfo("start figure")
        plt.figure(1)
        plt.xlabel('x/m')
        plt.ylabel('y/m')
        plt.plot(self.x0,self.y0,label='tb3-1')
        plt.plot(self.x1,self.y1,label='tb3-2')
        plt.plot(self.x2,self.y2,label='tb3-3')
        plt.plot(self.x3,self.y3,label='tb3-4')
        plt.legend(loc = 'upper left')

        plt.figure(2)
        
        plt.plot(self.a1, self.tr1, linewidth=3,label='tb3-1')
        plt.plot(self.a2, self.tr2, linewidth=3,label='tb3-2')
        plt.plot(self.a3, self.tr3, linewidth=3,label='tb3-3')
        plt.plot(self.a4, self.tr4, linewidth=3,label='tb3-4')
        plt.xlabel('x:sample times')
        plt.ylabel('y:trigger times')
        plt.legend(loc = 'upper left')
        plt.show()

if __name__ == '__main__':
    myfigure = Figure()

