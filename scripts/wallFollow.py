#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #message type used by /scan topic for laser scan data
import math #so I can use pi, cos and sin
import numpy as np

#assumes robot starts facing near direction it wants to go. If desired direction is behind robot this won't work!

#function called everytime we get a new lidar scan
def callback(msg):	
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
	if np.amin(ranges[0:90-15]) < 0.25: #too close to wall, turn right, usually hitting left wall
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
	if np.amin(ranges[90+15:]) < 0.25: #too close to wall, turn right, usually hitting left wall
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
			vel.angular.z = turn*-4
		else: #if opening is to our left, make a left turn (+z)
			print("	INF: turning left")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = turn*4
				
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
			vel.angular.z = turn*-1
		else: #if opening is to our left, make a left turn (+z)
			print("	REAL #: turning left")
			vel.linear.x = forward
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = turn
				
		publisher.publish(vel)
		rate.sleep()
	
if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim_controller',anonymous=True)
		
		#publisher:
		publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		vel = Twist() #intialize vel as a Twist message
		rate = rospy.Rate(10)
		
		#subscriber:
		subscriber = rospy.Subscriber("/scan", LaserScan, callback)

		#code won't run unless this while loop is present so have it with a pass
		while not rospy.is_shutdown():
			pass
		
	except rospy.ROSInterruptException:
		pass
