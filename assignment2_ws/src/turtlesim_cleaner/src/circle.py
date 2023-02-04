#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def circle():
    rospy.init_node('circle', anonymous = True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate =rospy.Rate(1)

    while not rospy.is_shutdown():
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.2

        velocity_publisher.publish(vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        # Testing our function
        circle()
    except rospy.ROSInterruptException:
        pass
