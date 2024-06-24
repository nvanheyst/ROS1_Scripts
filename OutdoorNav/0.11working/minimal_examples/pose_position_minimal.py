#!/usr/bin/env python3

from clearpath_localization_msgs.srv import *
from clearpath_navigation_msgs.msg import *
import rospy
from tf.transformations import euler_from_quaternion


def getRobotPose():
    print("Waiting for robot pose")
    odom_msg = rospy.wait_for_message("/localization/odom", nav_msgs.msg.Odometry, timeout=5)
    #print(odom_msg)
    return odom_msg

def getRobotLatLon():
    robot_pose = getRobotPose()
    pose_msg = geometry_msgs.msg.PoseStamped()
    pose_msg.pose = robot_pose.pose.pose
    xy_to_lat_lon_service_ = rospy.ServiceProxy('/localization/xy_to_lat_lon', ConvertCartesianToLatLon)
    robot_lat_lon = xy_to_lat_lon_service_(pose_msg)
    return robot_lat_lon.msg

if __name__ == '__main__':
    
    rospy.init_node('nvh_pose_position')

    robot_pose = getRobotPose()
    (robot_roll, robot_pitch, robot_yaw) = euler_from_quaternion(
                [robot_pose.pose.pose.orientation.x, robot_pose.pose.pose.orientation.y, robot_pose.pose.pose.orientation.z,
                robot_pose.pose.pose.orientation.w])

    enu_heading = robot_yaw*180/3.141592653
    
    if enu_heading >= 0:
        ned_heading = 90 - enu_heading
    else:
        ned_heading = 360 + enu_heading +90
    
    print("Heading: ")
    print(ned_heading)

    robot_lat_lon = getRobotLatLon()

    print("Latitude: ")
    print(robot_lat_lon.latitude)
    print("Longitude: ") 
    print(robot_lat_lon.longitude)

    exit()