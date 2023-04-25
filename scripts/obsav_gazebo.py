#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class obstacleAvoidance():
    def __init__(self):
        
        #initializing the node
        rospy.init_node('obstacle_avoidance',anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)          #publisher - velocity commands
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_update)    #subscriber - Laser scans, callback function that stores the subscribed data in a variable
        self.rate = rospy.Rate(10)
        self.left = 1 #random value
        self.right = 1 #random value
        self.front = 1 #random value
        
    def scan_update(self,data):
        self.front = min(0.5,sum(data.ranges[45:315])/90)
        self.left = min(1.2,sum(data.ranges[15:80])/55)
        self.right = min(1.2,sum(data.ranges[280:345])/55)
        front_value = self.front
        right_value = self.right
        left_vlaue = self.left
        
    def error(self):   
        err = self.left - self.right
        return err
        
    def long_pid(self,d):
        kp_long = 0.1
        return d*kp_long
        
    def lat_pid(self,e):
        kp_lat = 1.5
        return e*kp_lat     
        
    def obsav(self):
        self.vel = Twist()
        
        
        while not rospy.is_shutdown():
           print('front : {}'.format(self.front))
           print('right: {}'.format(self.right))
           print('left: {}'.format(self.left))
           #if self.lookahead_dist == 0:
               #self.lookahead_dist = 3.5
           linear_vel = min(0.22,self.long_pid(self.front))
           self.vel.linear.x = linear_vel
           
           
           error = self.error()
           ang_z = min(2.84,self.lat_pid(error))
           self.vel.angular.z = ang_z
           
           
           self.vel_pub.publish(self.vel)
           self.rate.sleep()
           
if __name__=='__main__':
    try:
        x = obstacleAvoidance()
        x.obsav()
    except rospy.ROSInterruptException: pass       
