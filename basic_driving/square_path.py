#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class SquarePath:
    def __init__(self):
        rospy.init_node('square_path')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)

        self.pose = Odometry()
        self.yaw = 0

        self.side_length = 1.0  
        self.linear_speed = 0.2  
        self.angular_speed = 0.2 
        self.turn_angle = math.pi/2
        self.tolerance = 0.98
        self.goal = self.turn_angle * self.tolerance

        rospy.wait_for_message('/odom', Odometry)
        self.run_square_path()

    def odom_callback(self, data):
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        roll, pitch, self.yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def run_square_path(self):
        for i in range(4):
            self.move_straight(self.side_length)
            rospy.sleep(0.1)  
            self.turn(self.goal)
            rospy.sleep(0.1)  

    def move_straight(self, distance):
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        distance_moved = 0

        start_x = self.pose.position.x
        start_y = self.pose.position.y

        while distance_moved < distance:
            self.velocity_publisher.publish(vel_msg)
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            distance_moved = math.sqrt((current_x - start_x) ** 2 + (current_y - start_y) ** 2)
            self.rate.sleep()

        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def turn(self, angle):
        vel_msg = Twist()
        vel_msg.angular.z = self.angular_speed if angle > 0 else -self.angular_speed
        start_yaw = self.yaw

        angle_turned = 0

        while angle_turned<angle:
            self.velocity_publisher.publish(vel_msg)
            angle_turned = self.yaw - start_yaw
            angle_turned = (angle_turned + 2 * 3.14159) % (2 * 3.14159)
            # rospy.loginfo(angle)
            # rospy.loginfo(angle_turned)
            self.rate.sleep() 

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        SquarePath()
    except rospy.ROSInterruptException:
        pass
