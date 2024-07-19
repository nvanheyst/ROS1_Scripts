#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

 
class Driver:

    def __init__(self):
        rospy.init_node('test_drive')
        self.pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.odomCB)
        self.rate = rospy.Rate(10)
        self.twist = Twist()
        self.driving_speed = 0.19
        self.current_x = 0.0
        #self.tolerance = 0.98
        self.goal = -1
        rospy.loginfo(self.goal)


    def run(self):
        while not rospy.is_shutdown():
            rospy.wait_for_message('/odom', Odometry)
            self.initial_x = self.current_x
            self.twist.linear.x = -self.driving_speed
            distance_driven = 0
            
            while distance_driven>self.goal:
                self.pub.publish(self.twist)
                distance_driven = self.current_x - self.initial_x
                
                rospy.loginfo(distance_driven)
                self.rate.sleep()   

            self.twist.linear.x = 0
            self.pub.publish(self.twist)  
            break   

    def odomCB(self,msg):        
        self.current_x = msg.pose.pose.position.x
        
        
        

if __name__ == "__main__":
    d = Driver()
    d.run()
