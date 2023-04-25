#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class obstacleAvoidance():
    def __init__(self):
        
        #initializing the node
        rospy.init_node('obstacle_avoidance',anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)          #publisher - velocity commands
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_update)    #subscriber - Laser scans, callback function that stores the subscribed data in a variable
        self.rate = rospy.Rate(10)
        
        #self.front = 1
        #self.left = 1
        #self.right = 1
        #self.front1 =1
        #self.left1 =1
        #self.right1 =1
        self.vel = Twist()

        
    def scan_update(self,data):
        ranges = np.array(data.ranges) #msg.ranges is of type tuple so can't modify. Need to convert to array
        #our lidar returns 0 for values outside max range. So convert these indices from 0 to inf
        ranges[np.where(ranges == 0)] = 20
        
        #self.front1 = ((sum(ranges[0:45])/45) + (sum(ranges[315:359])/44))/2
        #self.left1 = sum(ranges[45:80])/45
        #self.right1 = sum(ranges[280:325])/45
        front = min([min(ranges[0:45]),min(ranges[315:359])])
        left = min(ranges[15:80])
        right = min(ranges[280:345])
        #front_value = self.front
        #right_value = self.right
        #left_value = self.left
        
        print('front : {}'.format(front))
        print('right: {}'.format(right))
        print('left: {}'.format(left))


        
        #if self.lookahead_dist == 0:
        #self.lookahead_dist = 3.5
        linear_vel = min(0.22,self.long_pid(front))
        self.vel.linear.x = linear_vel
           
           
        error = left-right
        ang_z = min(2.84,self.lat_pid(error))
        self.vel.angular.z = ang_z
           
           
        self.vel_pub.publish(self.vel)
        self.rate.sleep()

        
    def long_pid(self,d):
        kp_long = 0.2
        return d*kp_long
        
    def lat_pid(self,e):
        kp_lat = 2
        return e*kp_lat     
        
    def obsav(self):
        pass
        
front = [];
left = [];
right = [];   
           
if __name__=='__main__':
    try:
        x = obstacleAvoidance()
        #x.obsav()
        
        while not rospy.is_shutdown():
        	pass
    except rospy.ROSInterruptException: pass       
