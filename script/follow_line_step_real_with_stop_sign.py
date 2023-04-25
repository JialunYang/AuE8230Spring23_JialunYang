#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PID import PID
from move_robot import MoveTurtlebot3
from darknet_ros_msgs.msg import BoundingBoxes
traffic_sign_c = 0


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.pub = pub
        self.status_sub = rospy.Subscriber(
            '/status', String, self.update_status)
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.camera_pub = rospy.Publisher(
            '/camera_flag', String, queue_size=10)
        self.stop_sign_subscriber = rospy.Subscriber(
            '/darknet_ros/bounding_boxes', BoundingBoxes, self.stop_sign_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

        self.twist_object = Twist()
        self.height = 0
        self.width = 0
        self.traj_flag = False

        # self.pid_controller = PID(P=0.0008, I=0.000, D=0.0005)
        self.pid_controller = PID(P=0.001, I=0.000, D=0.0005)
        self.pid_controller.clear()
        self.rate = rospy.Rate(10)
        self.camera_flag = 'F'
        self.last_err = 0
        self.status = 'a'

    def start_camera(self):
        self.image_sub = rospy.Subscriber("/camera/image",
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
        crop_img = cv_image[int((self.height / 2) + 100):int((self.height / 2) + 120)][1:int(self.width)]

        # crop_img = cv_image[220:240][1:320]
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([45, 30, 30])
        upper_yellow = np.array([97, 255, 209])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # lower_green_yellow = np.array([60, 80, 50])
        # upper_green_yello = np.array([100, 180, 120])
        # mask = cv2.inRange(crop_img, lower_green_yellow, upper_green_yello)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            self.camera_flag = 'T'
            self.traj_flag = True
        except ZeroDivisionError:
            cx, cy = self.height / 2, self.width / 2
            self.traj_flag = False

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
        cv2.circle(mask, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        cv2.imshow("orignal", cv_image)
        # cv2.imshow("hsv", hsv)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        if self.camera_flag == 'T':
            self.find_trajectory(cx, cy)

        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(self.twist_object.angular.z))

    def find_trajectory(self, cx, cy):
        # if not see the trajectory
        if self.traj_flag == False:
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

        # rospy.loginfo("cx value is {}, cy value is {}".format(cx, cy))

    def line_follow(self):
        start_time = None
        while not rospy.is_shutdown():

            self.camera_pub.publish(self.camera_flag)
            # Make it start turning
            if self.status == 'c':

                self.vel_pub.publish(self.twist_object)
                #     if start_time != None:
                #         now = rospy.get_time()
                #         rospy.loginfo(f"passed time {now - start_time}")
                #         if now - start_time > 6:
                #             rospy.loginfo("stop the robot!!!!!")
                #             self.twist_object.linear.x = 0
                #             self.twist_object.angular.z = 0
                #             self.vel_pub.publish(self.twist_object)
                #             rospy.sleep(3)
                #             start_time = None
                # elif self.status == 'd':
                #     rospy.loginfo("stop the robot after 6 seconds")
                #     if start_time == None:
                #         start_time = rospy.get_time()
            else:
                self.twist_object.linear.x = 0
                self.twist_object.angular.z = 0
                self.vel_pub.publish(self.twist_object)
                break

            self.rate.sleep()

    def stop_sign_callback(self, msg):
        global num
        num = 0
        if msg.bounding_boxes[len(msg.bounding_boxes) - 1].id == 11:
            prediction = msg.bounding_boxes
            for box in prediction:
                identified_class = box.Class
                probability = float(box.probability)
                area = abs(box.xmax - box.xmin) * abs(box.ymax - box.ymin)
                print(area, 'area')
                print('probability', probability)
                if ((probability >= 0.85) and (area >= 6000)):
                    print('stop_sign_detected')
                    num = 1

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
