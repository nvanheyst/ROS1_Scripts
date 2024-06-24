#!/usr/bin/env python3

from clearpath_localization_msgs.srv import *
from clearpath_navigation_msgs.msg import *
import rospy
from tf.transformations import euler_from_quaternion


def getRobotPose():
    print("Waiting for robot pose")
    odom_msg = rospy.wait_for_message("/localization/odom", nav_msgs.msg.Odometry, timeout=5)
 
    return odom_msg



if __name__ == '__main__':
    
    rospy.init_node('nvh_pose_position')

    robot_pose = getRobotPose()
    (robot_roll, robot_pitch, robot_yaw) = euler_from_quaternion(
                [robot_pose.pose.pose.orientation.x, robot_pose.pose.pose.orientation.y, robot_pose.pose.pose.orientation.z,
                robot_pose.pose.pose.orientation.w])

    enu_heading = robot_yaw*180/3.141592653
    

    print("Raw Heading (ENU): ")
    print(enu_heading)

    
    exit()