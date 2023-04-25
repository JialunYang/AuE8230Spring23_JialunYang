#!/usr/bin/env python3
import time
import threading

import cv2
import torch
import rospy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class CustomThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.value = 'a'

    def run(self):
        while True:
            self.value = input()
            print(f'you entered {self.value}')


class KeyBoard():
    def __init__(self):

        # start the keyboard detection
        self.a = CustomThread()
        self.a.start()

        # initializing the node
        rospy.init_node('status', anonymous=True)
        # pulisher - staus of the robot
        self.status_pub = rospy.Publisher('/status', String, queue_size=10)
        # publish frequency
        self.rate = rospy.Rate(20)

        # init status
        self.keyboard_input = 'a'
        self.once_time_flag = False
        self.traj_flag = False

    def publish_status(self):
        ''' publish the status from the keyboard'''

        while not rospy.is_shutdown():

            self.keyboard_input = self.a.value

            # rospy.loginfo(f"the status is {self.keyboard_input}")
            # rospy.loginfo(f"keyboard input is {self.keyboard_input}")
            self.status_pub.publish(self.keyboard_input)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = KeyBoard()
        rospy.sleep(2)
        node.publish_status()
    except rospy.ROSInterruptException:
        pass
