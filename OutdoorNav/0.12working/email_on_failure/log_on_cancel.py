#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalID
from clearpath_navigation_msgs.msg import *
from clearpath_localization_msgs.srv import *

import time


def callback(msg):
    current_time = time.ctime()
    robot_lat_lon = getRobotLatLon()
    rospy.loginfo("Mission cancelled at " + current_time)
    rospy.loginfo("Robot currently at " + str(robot_lat_lon.latitude) + ", " + str(robot_lat_lon.longitude))

def getRobotPose():
    #print("Waiting for robot pose")
    odom_msg = rospy.wait_for_message("/localization/odom", nav_msgs.msg.Odometry, timeout=5)
    return odom_msg

def getRobotLatLon():
    robot_pose = getRobotPose()
    pose_msg = geometry_msgs.msg.PoseStamped()
    pose_msg.pose = robot_pose.pose.pose
    xy_to_lat_lon_service_ = rospy.ServiceProxy('/localization/xy_to_lat_lon', ConvertCartesianToLatLon)
    robot_lat_lon = xy_to_lat_lon_service_(pose_msg)
    return robot_lat_lon.msg  
   
    
if __name__ == '__main__':
    rospy.init_node('log_on_cancel')
    
    sub = rospy.Subscriber("/mission/cancel", GoalID, callback)
    
    
    rospy.spin()