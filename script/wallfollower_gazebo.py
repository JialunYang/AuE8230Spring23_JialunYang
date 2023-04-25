#!/usr/bin/env python3
import time
import threading

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class WallFollowing():
    def __init__(self):

        # initializing the node
        rospy.init_node('wall_follower', anonymous=True)
        # publisher - velocity commands
        self.vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        # subscriber - Laser scans, callback function that stores the subscribed data in a variable
        self.scan_sub = rospy.Subscriber(
            '/robot1/scan', LaserScan, self.scan_update)
        # subcriber - status
        self.status_sub = rospy.Subscriber(
            '/status', String, self.update_status)

        # init the parameters
        self.rate = rospy.Rate(10)
        self.laser_scan90 = 1  # random value
        self.laser_scan270 = 1  # random value
        self.lookahead_dist = 1  # random value
        self.last_error_lat = 0
        self._last_time = None
        self.status = 'a'

    def update_status(self, data):
        if data.data != 'a':
            self.status = data.data
        # rospy.loginfo(f"the self.status is {self.status}")

    def scan_update(self, data):

        left = data.ranges[90]
        right = data.ranges[270]
        front = data.ranges[0]
        right_cone = []
        left_cone = []
        front_cone = []

        if right == float("inf"):
            right_min = 0.25
        else:
            for i in range(300, 341):
                right_cone.append(data.ranges[i])

        if left == float("inf"):
            left_min = 0.25
        else:
            for o in range(20, 61):
                left_cone.append(data.ranges[o])

        for p in range(len(data.ranges)):
            if p < 20 or p > 340:
                front_cone.append(data.ranges[p])
            if len(right_cone) != 0:
                right_min = min(right_cone)
            if len(left_cone) != 0:
                left_min = min(left_cone)

            front_min = min(front_cone)
        self.laser_scan90 = left_min
        self.laser_scan270 = right_min
        self.lookahead_dist = front_min

        # rospy.loginfo(f"lookahead dist is {self.lookahead_dist}")
        # rospy.loginfo(f"laser_scan90 dist is {self.laser_scan90}")
        # rospy.loginfo(f"laser_scan270 dist is {self.laser_scan270}")

    def currentError(self):
        d_90 = self.laser_scan90
        d_270 = self.laser_scan270
        # rospy.loginfo(f"Lidar error is {d_90 - d_270}")
        return d_90 - d_270

    def pdController_lat(self, error_lat):
        pGain_lat = 0.35  # 0.2 0.3
        dGain_lat = 0.1  # 0.1 0.15
        now = self._current_time()
        dt = now - self._last_time if self._last_time is not None else 1e-10
        pValue = pGain_lat * error_lat
        pValue = np.clip(pValue, -0.2, 0.2)
        dValue = dGain_lat * (error_lat - self.last_error_lat) / dt
        dValue = np.clip(dValue, -0.1, 0.1)
        self.last_error_lat = error_lat
        self._last_time = now

        return pValue + dValue

    def pController_lat(self, error_lat):
        pGain_lat_x = 1.0
        pGain_lat_y = 5  # 1.5
        if self.lookahead_dist < 0.3:
            return pGain_lat_y * error_lat
        return pGain_lat_x * error_lat

    def pController_long(self, dist):
        pGain_long = 0.1
        return pGain_long * dist

    def wallFollowing(self):
        self.vel_msg = Twist()
        while not rospy.is_shutdown():
            lin_x = self.pController_long(min(1.0, self.lookahead_dist))
            # changing the angular about z based on the error value
            self.vel_msg.linear.x = lin_x
        #    rospy.loginfo(f"linear vel is {self.vel_msg.linear.x}")
            error_lat = self.currentError()
            ang_z = min(self.pdController_lat(error_lat), 0.4)  # 0.2
            self.vel_msg.angular.z = ang_z
            # publishing these values
            self.vel_pub.publish(self.vel_msg)

            if self.lookahead_dist < 0.2:
                self.status = 'b'

            if self.status != 'a':
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.vel_pub.publish(self.vel_msg)
                break

            self.rate.sleep()

    def avoid(self):
        self.vel_msg = Twist()
        while not rospy.is_shutdown():
            lin_x = self.pController_long(min(1.0, self.lookahead_dist))
            self.vel_msg.linear.x = lin_x
            # changing the angular about z based on the error value
            error_lat = self.currentError()
            ang_z = min(0.6, self.pController_lat(error_lat))
            self.vel_msg.angular.z = ang_z
            self.vel_pub.publish(self.vel_msg)

            if self.status != 'b':
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.vel_pub.publish(self.vel_msg)
                break

            self.rate.sleep()

    @staticmethod
    def _current_time():
        """Return the current time obtained from time.monotonic()
           if python >= 3.3.
        """
        try:
            # Get monotonic time to ensure that time deltas are positive
            return time.monotonic()
        except AttributeError:
            # Fall back to time.time() if time.monotonic() is not available
            return time.time()


if __name__ == '__main__':
    try:
        x = WallFollowing()
        if x.status == 'a':
            x.wallFollowing()
        rospy.sleep(3)
        if x.status == 'b':
            x.avoid()
    except rospy.ROSInterruptException:
        pass
