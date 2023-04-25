#!/usr/bin/env python3

from pynput import keyboard
from apriltag_ros.msg import AprilTagDetectionArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #message type used by /scan topic for laser scan data
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from darknet_ros_msgs.msg import BoundingBoxes
import math #so I can use pi, cos and sin
import numpy as np

def on_press(key):
	pass

def on_release(key):
	#key is pynput.keyboard.KeyCode for letters and numbers
	#INITIAL STATE IS 0!
	
	#to tell Python we are editing a global var inside a function need to write "global subscriber"
	global subscriber #only need this ONCE
	global subscriber2 #need two because line following and stop sign task needs to subscribe to two topics
	
	
	####################################################################################################
	
	#initial state, don't move
	if key.char == "0":
		state = 0;
		print("State " + str(state) + " activated")
		
		subscriber.unregister() #assume node was previosuly registered to some topic and callback so need to disconnect
		subscriber2.unregister()
		cv2.destroyAllWindows() #assume other nodes brought up mask and camera windows so close them

		#stop robot moving from previous callback
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)
		subscriber = rospy.Subscriber("/scan", LaserScan, empty_callback)#use empty_callback so won't do anything. ALWAYS need to set subscibrer to unsuscbribe in other parts works
		
		
	####################################################################################################
		
	#State 1 = Wall Follow
	elif key.char == "1":
		state = 1;
		print("State " + str(state) + " activated")
		
		subscriber.unregister()
		#print("passed subscriber1 unsubscribe")
		subscriber2.unregister()
		#print("passed subscriber2 unsubscribe")
		cv2.destroyAllWindows()
		
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		#print("sent 0 velocity command")
		
		subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)
		#print("inited 2nd subscriber")
		subscriber = rospy.Subscriber("/scan", LaserScan, wallFollow_Callback)
		#print("inited 1st subscriber")
		
		
		
	######################################################################################################
		
	#State 2 = Obstacle Avoidance
	elif key.char == "2":
		state = 2;
		print("State " + str(state) + " activated")
		
		subscriber.unregister()
		subscriber2.unregister()
		cv2.destroyAllWindows()
		
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)
		subscriber = rospy.Subscriber("/scan", LaserScan, obstacle_avoid_callback)
		
		
	######################################################################################################
		
	#State 3 = Line Follow and Stop Sign
	elif key.char == "3":
		state = 3;
		print("State " + str(state) + " activated")
		
		subscriber.unregister()
		subscriber2.unregister()
		cv2.destroyAllWindows()
		
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		#global line_follower_object
		
		#line_follower_object = LineFollower() #creates a cv bridge
		subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, yolo_callback)
		subscriber = rospy.Subscriber("/camera/image/compressed",CompressedImage,camera_callback,queue_size = 1) #queue_size 1 is very important! or else huge lag
		
		
	######################################################################################################
		
	#State 4 = April Tag
	elif key.char == "4":
		state = 4;
		print("State " + str(state) + " activated")
		
		subscriber.unregister()
		subscriber2.unregister()
		cv2.destroyAllWindows()
		
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		
		publisher.publish(vel)
		rate.sleep()
		
		subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)
		subscriber = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,tag_update)
		
		
	######################################################################################################
		
def empty_callback(msg):
	pass
	
############################################################################################################
#Wall Follow Code

def wallFollow_Callback(msg):
	print("entered wallFollow callback")
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
	
	#SAFETY NET:
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
		
##################################################################################################################

#OBSTACLE AVOIDANCE CODE
def obstacle_avoid_callback(data):
	ranges = np.array(data.ranges) #msg.ranges is of type tuple so can't modify. Need to convert to array
	#our lidar returns 0 for values outside max range. So convert these indices from 0 to inf
	
	ranges[np.where(ranges == 0)] = 20
	front = min([min(ranges[0:45]),min(ranges[315:359])])
	left = min(ranges[15:80])
	right = min(ranges[280:345])
	
	print('front : {}'.format(front))
	print('right: {}'.format(right))
	print('left: {}'.format(left))
	
	#if self.lookahead_dist == 0:
	#self.lookahead_dist = 3.5
	linear_vel = min(0.22,long_pid(front))
	vel.linear.x = linear_vel
	
	error = left-right
	ang_z = min(2.84,lat_pid(error))
	vel.angular.z = ang_z
	
	publisher.publish(vel)
	rate.sleep()

def long_pid(d):
	kp_long = 0.2
	return d*kp_long
	
def lat_pid(e):
	kp_lat = 2
	return e*kp_lat

#####################################################################################################################
#LINE FOLLOW AND STOP SIGN CODE
def camera_callback(ros_data):
        # We select bgr8 because its the OpneCV encoding by default
        #cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") #if using non-CompressedImage version
        np_arr = np.frombuffer(ros_data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        #crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)] #crop height keep same width
        #crop_img = cv_image[340:360][1:640]
        crop_img = cv_image[int(height-20):int(height)][1:int(width)] #keep all of width, take only last 20 pixels of image in height (closer lookahead distance follows line better)

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        #cv2.inRange RANGE IS 0 TO 255
        #Online tool ranges:
        #H: 0 to 360
        #S: 0 to 100
        #V: 0 to  100
        lower_yellow = np.array([35*(255/360),2*2.55,28*2.55])
        upper_yellow = np.array([140*(255/360),60*2.55,100*2.55]) #don't go past 140 on H or will start to detect blue reflection of sky
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        
        foundLine = True #assume true at start and if division by 0 occurs make it false

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = width/2, height/2
            foundLine = False
            
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        #cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        #cv2.imshow("Original", cv_image)
        #cv2.imshow("MASK", mask)
        #cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        
        if foundLine == True:
            Kp = 1;
            vel.linear.x = 0.05 #gotta go very slow to account for network lag
            #if blob is on centerline, division will be 0, if blob is all the way right on screen, division will be 1.
            #Kp convert [0,1] range to [0,0.4] rad/s range
            #cx-(width/2) is positive when blob is right of centerline so need to multiply by negative Kp to get right turn
            vel.angular.z = -Kp * ((cx-width/2)/(width/2))
        else:
            vel.linear.x = 0
            vel.angular.z = 0.2
            
        print("cx = %f. mid = %f" % (cx,width/2))
        #################################

        rospy.loginfo("ANGULAR VALUE SENT===>"+str(vel.angular.z))
        # Make it start turning
        publisher.publish(vel)
        rate.sleep()
        
def yolo_callback(msg):
	global detectedStopSign
	global subscriber

	#msg.bounding_boxes is a python list where each index is a object detected
	#print("Raw Message:")
	#print(msg.bounding_boxes)
	
	#print("\n for loop mssg:")
	
	if detectedStopSign == False:
		for i in msg.bounding_boxes:
			if i.Class == "stop sign" or i.Class == "parking meter" or i.Class == "clock":
				print("DETECTED STOP SIGN!")
				detectedStopSign = True
				print("Stopping for 3 seconds")
				subscriber.unregister() #unsubscribe lidar subscriber (stops callback)
				rospy.sleep(3.)
				subscriber = rospy.Subscriber("/camera/image/compressed",CompressedImage,camera_callback,queue_size = 1) #re-subscribe
			

#####################################################################################################################

#APRIL TAG CODE
    
def tag_update(data):
        global x
        global z
        
        x = data.detections[0].pose.pose.pose.position.x
        z = data.detections[0].pose.pose.pose.position.z
        print('x: {}'.format(x))
        print('z: {}'.format(z))
        
        gain_x = 0.1
        gain_z = -1.5
        
        if z >= 0.2:
        	vel.linear.x = z*gain_x
        	vel.angular.z = x*gain_z
        else:
        	vel.linear.x = 0
        	vel.angular.z = 0
        	
        #publishing these values
        publisher.publish(vel)
        rate.sleep()
        

#####################################################################################################################

		
rospy.init_node('turtlesim_controller',anonymous=True)

#global variables (need to be used in multiple functions)
state = 0

publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)#all states will use this so put it here for all to use
vel = Twist() #intialize vel as a Twist message
rate = rospy.Rate(40)

subscriber = rospy.Subscriber("/scan", LaserScan, empty_callback)
subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)

bridge_object = CvBridge()
detectedStopSign = False

x = 0
z = 0

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
		
		print("Initial state = " + str(state))
		
		listener = keyboard.Listener(on_press=on_press,on_release=on_release)
		listener.start()

		#code won't run unless this while loop is present so have it with a pass
		while not rospy.is_shutdown():
			pass
		
	except rospy.ROSInterruptException:
		pass
