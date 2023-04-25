#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PID import PID

traffic_sign_c = 0


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()

        self.camera_flag_sub = rospy.Subscriber(
            '/camera_flag', String, self.update_camera_status)
        self.status_sub = rospy.Subscriber(
            '/status', String, self.update_status)
        self.april_tag_sub = rospy.Subscriber(
            '/april_tag_flag', String, self.april_tag_update)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.twist_object = Twist()
        self.height = 0
        self.width = 0
        self.traj_flag = False
        self.once_stop_flag = False

        # self.pid_controller = PID(P=0.0008, I=0.000, D=0.0005)
        self.pid_controller = PID(P=0.001, I=0.000, D=0.0005)
        self.pid_controller.clear()
        self.rate = rospy.Rate(10)
        self.camera_flag = 'F'
        self.april_tag_flag = 'F'
        self.last_err = 0
        self.status = 'a'
        self.turn_flag = False
        self.start_time = None

    def april_tag_update(self, data):
        self.april_tag_flag = data.data

    def start_camera(self):
        self.image_sub = rospy.Subscriber("/camera/image",
                                          Image, self.camera_callback)

    def update_camera_status(self, data):
        self.camera_flag = data.data
        rospy.loginfo(f"camera flag is {self.camera_flag}")

    def update_status(self, data):
        self.status = data.data
        self.status = 'c'

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(
            data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        self.height, self.width, channels = cv_image.shape
        # crop_img = cv_image[int((self.height/2)+100):int((self.height/2)+120)][1:int(self.width)]
        # rospy.loginfo(
        # f'image size is height: {self.height} width: {self.width}')
        crop_img = cv_image[int((self.height / 2) + 100):int((self.height / 2) + 120)][1:int(self.width)]

        # crop_img = cv_image[220:240][1:320]
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        # hsv2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        # without flashlight
        # lower_yellow = np.array([45, 30, 30])
        # upper_yellow = np.array([97, 255, 209])
        # with flashlight
        lower_yellow = np.array([15, 32, 70])
        upper_yellow = np.array([75, 255, 210])
        # stop sign hsv value
        # lower_red = np.array([20, 20, 80])
        # upper_red = np.array([60, 60, 255])

        # stop sign hsv value with flashlight
        lower_red = np.array([70, 70, 70])
        upper_red = np.array([150, 190, 190])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(cv_image, lower_red, upper_red)

        # find the contours
        contours, _ = cv2.findContours(
            mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # take the first contour
        try:
            cnt = contours[0]
            x, y, w, h = cv2.boundingRect(cnt)
        except:
            w = 0

        # rospy.loginfo(f"the width of the stop sign {w}")

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            self.camera_flag = 'T'
            self.traj_flag = True
            rospy.loginfo("Find the yellow line!!!!!!!!!!!")
        except ZeroDivisionError:
            cx, cy = self.height / 2, self.width / 2
            self.traj_flag = False

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
        cv2.circle(mask, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        cv2.imshow("orignal", cv_image)
        # cv2.imshow("hsv", hsv)
        cv2.imshow("MASK", mask2)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        # if self.camera_flag == 'T':
        #     self.find_trajectory(cx, cy, w)
        self.find_trajectory(cx, cy, w)

        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(self.twist_object.angular.z))

    def find_trajectory(self, cx, cy, w):
        # if not see the trajectory
        # rospy.loginfo("control the turtlebot3!")
        if self.traj_flag == False:
            self.turn_flag = True
            self.pid_controller.update(3 * self.last_err)
            self.twist_object.linear.x = 0.12  # 0.07
            self.twist_object.angular.z = self.pid_controller.output
            # rospy.loginfo("lose the trajectory")
        else:
            err = -1 * (self.width / 2 - cx)
            self.pid_controller.update(err)
            self.twist_object.linear.x = 0.12  # 0.07
            self.twist_object.angular.z = self.pid_controller.output
            self.last_err = err
            # rospy.loginfo("the error is {}".format(-1 * err))

        if self.once_stop_flag == False and self.traj_flag == True:
            if self.start_time == None and w > 20:
                self.start_time = rospy.get_time()
            now = rospy.get_time()
            if self.start_time != None:
                time_elapse = now - self.start_time
                # rospy.loginfo(f"elapse time is {time_elapse}")
                if time_elapse > 3:
                    self.once_stop_flag = True
                    self.twist_object.linear.x = 0
                    self.twist_object.angular.z = 0
                    rospy.loginfo("stop the turtlebot")
                    rospy.sleep(3)
        # rospy.loginfo("cx value is {}, cy value is {}".format(cx, cy))

    def line_follow(self):
        start_time = None
        while not rospy.is_shutdown():

            # Make it start turning
            if self.status == 'c':
                self.vel_pub.publish(self.twist_object)

            else:
                self.twist_object.linear.x = 0
                self.twist_object.angular.z = 0
                self.vel_pub.publish(self.twist_object)
                break
            if self.april_tag_flag == 'T':
                self.twist_object.linear.x = 0
                self.twist_object.angular.z = 0
                self.vel_pub.publish(self.twist_object)
                break

            self.rate.sleep()

    def clean_up(self):
        self.twist_object.linear.x = 0
        self.twist_object.angular.z = 0
        self.vel_pub.publish(self.twist_object)
        cv2.destroyAllWindows()


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    while line_follower_object.status != 'c':
        continue

    # while line_follower_object.camera_flag != 'T':
    #     continue

    rospy.sleep(2)

    rospy.loginfo("start the camera")
    line_follower_object.start_camera()

    rospy.sleep(2)

    # control the robot
    line_follower_object.line_follow()

    rate = rospy.Rate(5)
    ctrl_c = False

    line_follower_object.clean_up()


if __name__ == '__main__':

    main()
