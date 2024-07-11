#!/usr/bin/env python3

import rospy

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
 
class Driver:

    def __init__(self):
        rospy.init_node('test_drive')
        self.pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.odomCB)
        self.rate = rospy.Rate(10)
        self.twist = Twist()
        self.rotation_speed = 0.5
        self.current_yaw = 0.0
        self.tolerance = 0.98
        self.goal = math.pi*2*self.tolerance
        self.initial_yaw = None
        rospy.loginfo(self.goal)


    def run(self):
        while not rospy.is_shutdown():
            rospy.wait_for_message('/odom', Odometry)
            self.initial_yaw = self.current_yaw
            self.twist.angular.z = self.rotation_speed
            angle_turned = 0
            
            while angle_turned<self.goal:
                self.pub.publish(self.twist)
                angle_turned = self.current_yaw - self.initial_yaw
                angle_turned = (angle_turned + 2 * 3.14159) % (2 * 3.14159)

                rospy.loginfo(angle_turned)
                self.rate.sleep()   

            self.twist.angular.z = 0
            self.pub.publish(self.twist)  
            break   

    def odomCB(self,msg):        
        self.pose = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        
        

if __name__ == "__main__":
    d = Driver()
    d.run()