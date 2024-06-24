#!/usr/bin/env python3

import rospy
from clearpath_navigation_msgs.msg import *
from clearpath_localization_msgs.srv import *

import time

import smtplib
from email.message import EmailMessage

password = "quya pzra arwe uogo"
robot = "Marketing Observer (CPR-Hicks)"



def callback(msg):
    result = msg
    if result.result.success : 
        print("Sucessful mission")
    else:
        
        current_time = time.ctime()
        robot_lat_lon = getRobotLatLon()
        reason = result.result.message
        # rospy.loginfo("Mission cancelled at " + current_time)
        # rospy.loginfo("Reason: " + reason)
        # rospy.loginfo("Robot currently at " + str(robot_lat_lon.latitude) + ", " + str(robot_lat_lon.longitude))
        
        msg = EmailMessage()
        msg.set_content('Mission cancelled at ' + str(current_time) + 
                        '\nReason: ' + str(reason) + '\nRobot currently at: ' + str(robot_lat_lon.latitude)+ ', ' + str(robot_lat_lon.longitude) )

        msg['Subject'] = 'Mission cancelled for ' + robot
    
        msg['From'] = "huskyobserver@gmail.com" 
        msg['To'] = "huskyobserver+python@gmail.com"

        server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
        server.login("huskyobserver@gmail.com", password)
        server.send_message(msg)
        print("Sent email")
        server.quit()


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
    rospy.init_node('email_on_failure')
    
    sub = rospy.Subscriber("/autonomy/network_mission/result", 
                           clearpath_navigation_msgs.msg.ExecuteNetworkMissionByUuidActionResult, callback)
    
    
    rospy.spin()