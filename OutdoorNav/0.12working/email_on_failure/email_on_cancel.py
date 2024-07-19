#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalID
from clearpath_navigation_msgs.msg import *
from clearpath_localization_msgs.srv import *

import time

import smtplib, ssl

port = 465  
smtp_server = "smtp.gmail.com"
password = #password hidden
sender_email = "huskyobserver@gmail.com"  
receiver_email = "huskyobserver+python@gmail.com"  
#receiver_email2 = "huskyobserver+python2@gmail.com"  
robot = "cpr-hicks"
context = ssl.create_default_context()


def callback(msg):
    current_time = time.ctime()
    robot_lat_lon = getRobotLatLon()
    rospy.loginfo("Mission cancelled at " + current_time)
    rospy.loginfo("Reason: Cancelled by user")
    rospy.loginfo("Robot currently at " + str(robot_lat_lon.latitude) + ", " + str(robot_lat_lon.longitude))
    
    message = """\
    Subject: Mission cancelled for{robot} 

    Mission cancelled at {current_time}
    Reason: Cancelled by user
    Robot currently at {robot_lat_lon.latitude}, {robot_lat_lon.longitude}."""

    with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
        server.login("huskyobserver@gmail.com", password)
        print("Successful login")
        server.sendmail(sender_email, receiver_email, message.format(robot=robot,current_time=current_time,robot_lat_lon=robot_lat_lon))
        #server.sendmail(sender_email, receiver_email2, message)
        print("Email sent")

def getRobotPose():
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
    rospy.init_node('email_on_cancel')
    
    sub = rospy.Subscriber("/mission/cancel", GoalID, callback)
    
    
    rospy.spin()
