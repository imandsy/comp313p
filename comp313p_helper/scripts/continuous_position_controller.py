#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt
from PyKDL import Rotation

class stdr_controller():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('stdr_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.command_pose_subscriber = rospy.Subscriber('/robot0/cmd_pose', Pose, self.command_callback)
        self.current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_callback)
        self.command_pose = Pose()
        self.current_pose = Odometry()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def command_callback(self, data):
        self.command_pose = data
        self.command_pose.x = round(self.command_pose.x, 4)
        self.command_pose.y = round(self.command_pose.y, 4)
        self.command_pose.theta = round(self.command_pose.theta, 4)

    def current_callback(self, data):
        self.current_pose = data

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.current_pose.pose.pose.position.x), 2) + pow((goal_y - self.current_pose.pose.pose.position.y), 2))
        return distance

    def run(self):
        self.command_pose.x = self.current_pose.pose.pose.position.x
        self.command_pose.y = self.current_pose.pose.pose.position.y

        while not rospy.is_shutdown():
            distance_tolerance = 0.01
            vel_msg = Twist()

            theta = 0

            while self.get_distance(self.command_pose.x, self.command_pose.y) >= distance_tolerance:

                theta = Rotation().Quaternion(self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w).GetEulerZYX()[0]
                #Porportional Controller
                #linear velocity in the x-axis:
                vel_msg.linear.x = 0.5 * sqrt(pow((self.command_pose.x - self.current_pose.pose.pose.position.x), 2) + pow((self.command_pose.y - self.current_pose.pose.pose.position.y), 2))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                #angular velocity in the z-axis:
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 1 * (atan2(self.command_pose.y - self.current_pose.pose.pose.position.y, self.command_pose.x - self.current_pose.pose.pose.position.x) - theta)

                #Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            #Stopping our robot after the movement is over
            vel_msg.linear.x = 0
            vel_msg.angular.z =0
            self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        #Testing our function
        x = stdr_controller()
        x.run()

    except rospy.ROSInterruptException: pass