#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def angular_spd(self, goal_pose, constant=6):
        angle1 = goal_pose.theta - self.pose.theta
        angle2 = goal_pose.theta - pi * 2 - self.pose.theta
        angle3 = goal_pose.theta + pi * 2 - self.pose.theta
        angle = min([angle1, angle2, angle3], key = abs)
        return constant * angle

    def move2goal(self, coordinate):
        """Moves the turtle to the goal."""
        goal_pose = Pose()
        vel_msg = Twist()

        # Set the tolerance -- a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.01
        angle_tolerance = 0.001

        # Set the goal pose
        goal_pose.x = coordinate[0]
        goal_pose.y = coordinate[1]
        goal_pose.theta = coordinate[2]

        if (abs(goal_pose.x - self.pose.x) > 0.1) & (abs(goal_pose.y - self.pose.y) > 0.1):
            while self.euclidean_distance(goal_pose) >= distance_tolerance:

                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(goal_pose)

                # Angular velocity in the z-axis.
                vel_msg.angular.z = self.angular_vel(goal_pose)

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                self.rate.sleep()

        else:
            while self.euclidean_distance(goal_pose) > distance_tolerance:
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(goal_pose)
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

            while abs(self.pose.theta - goal_pose.theta) > angle_tolerance:
                #Angular velocity in the z-axis
                vel_msg.angular.z = self.angular_spd(goal_pose)
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

    def square_closedloop(self):
        coordinates = [[5, 5, 0], [5, 5, 0], [8, 5, pi/2], [8, 8, pi], [5, 8, -pi/2], [5, 5, 0]]
        for cdn in coordinates:
            self.move2goal(cdn)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.square_closedloop()
    except rospy.ROSInterruptException:
        pass
    
