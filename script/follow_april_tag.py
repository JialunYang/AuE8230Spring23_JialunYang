#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String


class tagFollow():
    def __init__(self):

        # initializing the node
        rospy.init_node('follow_april_tag', anonymous=True)
        # publisher - velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # subscriber - Laser scans, callback function that stores the subscribed data in a variable
        self.scan_sub = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.tag_update)
        self.status_sub = rospy.Subscriber(
            '/status', String, self.update_status)
        self.april_tag_pub = rospy.Publisher(
            '/april_tag_flag', String, queue_size=10)
        self.rate = rospy.Rate(10)
        self.status = 'a'
        self.april_tag_flag = 'F'
        self.x = 0
        self.z = 0

    def update_status(self, data):
        self.status = data.data
        self.status = 'd'

    def tag_update(self, data):
        if len(data.detections) > 0:
            self.x = data.detections[0].pose.pose.pose.position.x
            self.z = data.detections[0].pose.pose.pose.position.z
            self.get_april_tag = True
            self.april_tag_flag = 'T'
            self.april_tag_pub.publish(self.april_tag_flag)
            rospy.loginfo(
                "x position is {}, z position is {}".format(self.x, self.z))
        else:
            self.get_april_tag = False

    def follow(self):
        self.vel_msg = Twist()

        while not rospy.is_shutdown():
            gain_x = 0.2
            gain_z = -5

            if self.z >= 0.4:
                self.vel_msg.linear.x = self.z * gain_x
                self.vel_msg.angular.z = self.x * gain_z

            else:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0

            # publishing these values

            self.vel_pub.publish(self.vel_msg)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        bot = tagFollow()
        while bot.status != 'd':
            continue
        while bot.april_tag_flag != 'T':
            continue
        bot.follow()
    except rospy.ROSInterruptException:
        pass
