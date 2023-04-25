#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PID import PID


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()

        self.status_sub = rospy.Subscriber(
            '/status', String, self.update_status)

        self.vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)

        self.twist_object = Twist()
        self.height = 0
        self.width = 0
        self.traj_flag = False
        # self.pid_controller = PID(P=0.0015, I=0.000, D=0.0007)
        self.pid_controller = PID(P=0.0012, I=0.000, D=0.0006)
        self.pid_controller.clear()
        self.rate = rospy.Rate(10)
        self.status = 'a'

    def start_camera(self):
        self.image_sub = rospy.Subscriber("/robot1/camera/rgb/image_raw",
                                          Image, self.camera_callback)

    def update_status(self, data):
        self.status = data.data

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(
            data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        self.height, self.width, channels = cv_image.shape
        # crop_img = cv_image[int((self.height/2)+100):int((self.height/2)+120)][1:int(self.width)]
        # rospy.loginfo(
        # f'image size is height: {self.height} width: {self.width}')
        crop_img = cv_image[440:460][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            self.traj_flag = True
        except ZeroDivisionError:
            cx, cy = self.height / 2, self.width / 2
            self.traj_flag = False

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
        # cv2.circle(mask, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        # # cv2.imshow("Original", cv_image)
        # cv2.imshow("MASK", mask)
        # cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        self.find_trajectory(cx, cy)

        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(self.twist_object.angular.z))

    def find_trajectory(self, cx, cy):
        # if not see the trajectory
        if self.traj_flag == False:
            self.twist_object.linear.x = 0.1
            # rospy.loginfo("lose the trajectory")
        else:
            err = -1 * (self.width / 2 - cx)
            self.pid_controller.update(err)
            self.twist_object.linear.x = 0.1
            self.twist_object.angular.z = self.pid_controller.output
            # rospy.loginfo("the error is {}".format(-1 * err))

        # rospy.loginfo("cx value is {}, cy value is {}".format(cx, cy))

    def line_follow(self):
        start_time = None
        while not rospy.is_shutdown():

            # Make it start turning
            if self.status == 'c':
                self.vel_pub.publish(self.twist_object)
                if start_time != None:
                    now = rospy.get_time()
                    rospy.loginfo(f"passed time {now - start_time}")
                    if now - start_time > 6:
                        rospy.loginfo("stop the robot!!!!!")
                        self.twist_object.linear.x = 0
                        self.twist_object.angular.z = 0
                        self.vel_pub.publish(self.twist_object)
                        rospy.sleep(3)
                        start_time = None
            elif self.status == 'd':
                rospy.loginfo("stop the robot after 6 seconds")
                if start_time == None:
                    start_time = rospy.get_time()
            else:
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
    # while line_follower_object.status != 'b':
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
