#!/usr/bin/env python3

# a simple program to drive a robot in an increasing spiral and then back
# developed and tested on a Clearpath Ridgeback in Simulation, not for use on a real platform without additional review and testing
# considerations - no obstacle detection, only using wheel encoder data

import rospy
from geometry_msgs.msg import Twist

class SpiralDriver:
    def __init__(self):
        rospy.init_node('spiral_driver')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 1
        self.speed_increment = 0.025
        self.state = True

    def drive_in_spiral(self):
        twist = Twist()

        while not rospy.is_shutdown():
            twist.linear.x = self.current_linear_velocity
            twist.angular.z = self.current_angular_velocity

            self.pub.publish(twist)
            
            if self.current_linear_velocity>=1:
                self.state=False

            if not self.state and 0<=self.current_linear_velocity<0.025:
                self.current_linear_velocity=0
                self.pub.publish(twist)
                rospy.loginfo("Spiral complete")
                return

            if self.state:
                self.current_linear_velocity+=self.speed_increment
            else:
                self.current_linear_velocity-=self.speed_increment


            #rospy.loginfo(self.current_linear_velocity)

            rospy.sleep(0.25)

if __name__ == '__main__':
    try:
        driver = SpiralDriver()
        driver.drive_in_spiral()
    except rospy.ROSInterruptException:
        pass
