#!/usr/bin/env python3

from pynput import keyboard
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #message type used by /scan topic for laser scan data
import math #so I can use pi, cos and sin
import numpy as np

def on_press(key):
	pass

def on_release(key):
	#key is pynput.keyboard.KeyCode for letters and numbers
	#INITIAL STATE OF PARAAM IS 0!
	
	if key.char == "0":
		rospy.set_param("state", 0)
		print("State " + str(rospy.get_param("/state")) + " activated")
	if key.char == "1":
		rospy.set_param("state", 1)
		print("State " + str(rospy.get_param("/state")) + " activated")
	elif key.char == "2":
		rospy.set_param("state", 2)
		print("State " + str(rospy.get_param("/state")) + " activated")
	elif key.char == "3":
		rospy.set_param("state", 3)
		print("State " + str(rospy.get_param("/state")) + " activated")
	elif key.char == "4":
		rospy.set_param("state", 4)
		print("State " + str(rospy.get_param("/state")) + " activated")
	
	if key == keyboard.Key.esc:
		# Stop listener
		return False

if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim_controller',anonymous=True)
		
		print("Initial state = " + str(rospy.get_param("/state")))
		
		listener = keyboard.Listener(on_press=on_press,on_release=on_release)
		listener.start()

		#code won't run unless this while loop is present so have it with a pass
		while not rospy.is_shutdown():
			pass
		
	except rospy.ROSInterruptException:
		pass
