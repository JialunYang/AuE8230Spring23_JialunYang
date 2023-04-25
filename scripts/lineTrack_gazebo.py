#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
#could not get import move_robot to work so just pasted in this file

rospy.init_node('line_following_node', anonymous=True)

#need this to be global so scope lets me access from all funcs
twist_object = Twist()

class MoveTurtlebot3(object):

    def __init__(self):
        #creates an object which publishes (also subscribes for error check) to cmd_vel and initializes a Twist message
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_subs = rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(40) #publishing to cmd_vel 50 times per second. 10 Hz was creating too much weaving

    def cmdvel_callback(self,msg):
        self.last_cmdvel_command = msg
    
    def compare_twist_commands(self,twist1,twist2):
        LX = twist1.linear.x == twist2.linear.x
        LY = twist1.linear.y == twist2.linear.y
        LZ = twist1.linear.z == twist2.linear.z
        AX = twist1.angular.x == twist2.angular.x
        AY = twist1.angular.y == twist2.angular.y
        AZ = twist1.angular.z == twist2.angular.z
        equal = LX and LY and LZ and AX and AY and AZ
        if not equal:
            rospy.logwarn("The Current Twist is not the same as the one sent, Resending")
        return equal

    def move_robot(self, twist_object):
        # We make this to avoid Topic loss, specially at the start
        current_equal_to_new = False
        while (not (current_equal_to_new) ):
            self.cmd_vel_pub.publish(twist_object)
            self._cmdvel_pub_rate.sleep()
            current_equal_to_new = self.compare_twist_commands(twist1=self.last_cmdvel_command,
                                    twist2=twist_object)
                                    
    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)

#need this to be global so scope lets me access from all funcs
moveTurtlebot3_object = MoveTurtlebot3()

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

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

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        
        foundLine = True #assume true at start and if division by 0 occurs make it false

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = width/2, height/2
            foundLine = False
            
        if foundLine == True:
            print("Line Found!")
        else:
            print("Looking for line")
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        
        if foundLine == True:
            #this code did not turn more or else depending on how far from centerline the blob is
            #tolerance = 5
            #if cx > width/2+tolerance: #blob to right of centerline
            #    twist_object.linear.x = 0
            #    twist_object.angular.z = -0.2 #turn right
            #elif cx < width/2-tolerance:
            #    twist_object.linear.x = 0
            #    twist_object.angular.z = 0.2
            #else:
            #    twist_object.linear.x = 0.15
            #    twist_object.angular.z = 0
            
            #this code turns more if the blob is further from centerline, should allow me to use constant longitudinal velocity
            Kp = 0.4;
            twist_object.linear.x = 0.15 #can't go full 0.2
            #if blob is on centerline, division will be 0, if blob is all the way right on screen, division will be 1.
            #Kp convert [0,1] range to [0,0.4] rad/s range
            #cx-(width/2) is positive when blob is right of centerline so need to multiply by negative Kp to get right turn
            twist_object.angular.z = -Kp * ((cx-width/2)/(width/2))
        else:
            twist_object.linear.x = 0
            twist_object.angular.z = 0.2
            
        print("cx = %f. mid = %f" % (cx,width/2))
        #################################

        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        # Make it start turning
        moveTurtlebot3_object.move_robot(twist_object)

    def clean_up(self):
        moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()



def main():
    
    #initial Twist message is all zeros to make it turn to start off with
    twist_object.angular.z = 0.2
    
    #creates a cv bridge, subscribes to camera and creates a MoveTurtlebot3 object
    line_follower_object = LineFollower()
    rate = rospy.Rate(40) #read from lidar 10 times per second
    
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        moveTurtlebot3_object.clean_class()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        moveTurtlebot3_object.move_robot(twist_object)
        rate.sleep()

if __name__ == '__main__':
        main()
