#!/usr/bin/env python3

from pynput import keyboard
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #message type used by /scan topic for laser scan data
import math #so I can use pi, cos and sin
import numpy as np

def on_press(key):
	pass

def on_release(key):
	#key is pynput.keyboard.KeyCode for letters and numbers
	#INITIAL STATE IS 0!
	
	#to tell Python we are editing a global var inside a function need to write "global subscriber"
	global subscriber #only need this ONCE
	
	#initial state, don't move
	if key.char == "0":
		state = 0;
		print("State " + str(state) + " activated")
		
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		subscriber.unregister()
		subscriber = rospy.Subscriber("/scan", LaserScan, empty_callback)#use empty_callback so won't do anything. ALWAYS need to set subscibrer to unsuscbribe in other parts works
		
	####################################################################################################
		
	#State 1 = Wall Follow
	elif key.char == "1":
		state = 1
		print("State " + str(state) + " activated")
		
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		subscriber.unregister()
		subscriber = rospy.Subscriber("/scan", LaserScan, laser_wallFollow_Callback)
		
		
	######################################################################################################
		
	#State 2 = Obstacle Avoidance
	elif key.char == "2":
		state = 2;
		print("State " + str(state) + " activated")
		
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		
		
		subscriber.unregister()
		x = obstacleAvoidance()
		x.obsav()

		
	######################################################################################################
		
	#State 3 = Line Follow and Stop Sign
	elif key.char == "3":
		state = 3;
		print("State " + str(state) + " activated")
		
		#subscriber.unregister()
		#subscriber = rospy.Subscriber("/scan", LaserScan, DIFFERENT_CALLBACK_FUNC)
		
	######################################################################################################
		
	#State 4 = April Tag
	elif key.char == "4":
		state = 4;
		print("State " + str(state) + " activated")
		
		#subscriber.unregister()
		#subscriber = rospy.Subscriber("/scan", LaserScan, DIFFERENT_CALLBACK_FUNC)
		
	######################################################################################################
	
def empty_callback(msg):
	vel.linear.x = 0
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = 0
		
	publisher.publish(vel)
	rate.sleep()
		
def laser_wallFollow_Callback(msg):
	ranges = np.array(msg.ranges) #msg.ranges is of type tuple so can't modify. Need to convert to array
	
	#our lidar returns 0 for values outside max range. So convert these indices from 0 to inf
	ranges[np.where(ranges == 0)] = np.inf

	#ignore 1/2 of ranges behind turtlebot so only 1 solution instead of 2
	leftRanges = ranges[0:90] #doesn't include last index
	rightRanges = ranges[270:360]
	
	#final array needs to read -90 to +90
	leftRanges = np.flip(leftRanges) 
	rightRanges = np.flip(rightRanges)
	
	ranges = np.hstack((leftRanges,rightRanges)) #creates frontal hemisphere of ranges where mid is now index (180/2)-1=89
	
	boolean = np.isinf(ranges) #returns array where true if inf and false otherwise

	#range_max does not return INF when max range is actually INF, so don't use it	
	
	print("closest distance to wall: %f"%np.amin(ranges))
	
	forward = 0.2;
	turn = 0.25; #0.3 works
	
	#safety net to avoid hitting wall at all costs
	
	#if hit left wall turn right
	if np.amin(ranges[44:90]) < 0.20: #if we're driving at the left side wall (okay to be driving parallel to it so dont use 0:45 deg ranges)
		print("Too close to left wall! d = %f"%np.amin(ranges))
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = turn*-3
		
		publisher.publish(vel)
		rate.sleep()
		
		return #if heading towards wall, turn right and *exit* callback function
		
	#if hit right wall turn left
	if np.amin(ranges[90:135]) < 0.20: #too close to wall, turn right, usually hitting left wall
		print("Too close to right wall! d = %f"%np.amin(ranges))
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = turn*3
		
		publisher.publish(vel)
		rate.sleep()
		
		return
		
	#2 cases:
	#case 1: max range is a number (not INF) and we need to locate it at our 0 deg (don't need to cluster)
	#case 2: max range is INF so we need to find cluster of INF and make the mid point our 0 deg
	
	#Case 2:
	if boolean.any() == True: #if any number in our ranges is inf (a far distance is present in our hemisphere GOOD), will enter if
		#find cluster of consecutive ints and take mid point
		ind = np.average(np.where(boolean)[0]) #find indices of INF entries then take average to get most important direction (should outweight small clusters of INF vs large ones)
		print("INF: ind = %d" % ind)
	
	#don't use while loop because it will keep sending turn command and ind won't update till next callback so gets stuck in infinite turn	
		if ind == 89:
			print("	INF: going straight")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = 0
		elif ind > 89: #if opening is to our right make a negative turn (CW in z is negative)
			print("	INF: turning right")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = turn*-2
		else: #if opening is to our left, make a left turn (+z)
			print("	INF: turning left")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = turn*2
				
		publisher.publish(vel)
		rate.sleep()
	#Case 1
	else: #furthest away point is within max range of lidar so it retunrs a number and not INF
		ind = np.argmax(ranges) #returns index of max value in ranges
		print("REAL #: ind = %d" % ind)
		
		if ind == 89:
			print("	REAL #: going straight")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = 0
		elif ind > 89: #if opening is to our right make a negative turn (CW in z is negative)
			print("	REAL #: turning right")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = turn*-2
		else: #if opening is to our left, make a left turn (+z)
			print("	REAL #: turning left")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = turn*2
				
		publisher.publish(vel)
		rate.sleep()
		

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



		
rospy.init_node('turtlesim_controller',anonymous=True)

#global variables (need to be used in multiple functions)
state = 0

publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)#all states will use this so put it here for all to use
vel = Twist() #intialize vel as a Twist message
rate = rospy.Rate(10)

subscriber = rospy.Subscriber("/scan", LaserScan, empty_callback)


if __name__ == '__main__':
	try:
		#ensure robot is not moving at start
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		print("WAITING FOR STATE")
		
		listener = keyboard.Listener(on_press=on_press,on_release=on_release)
		listener.start()
		
		#code won't run unless this while loop is present so have it with a pass
		while not rospy.is_shutdown():
			pass
		
	except rospy.ROSInterruptException:
		pass
