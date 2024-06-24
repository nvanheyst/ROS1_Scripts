#!/usr/bin/env python3

import rospy
from clearpath_navigation_msgs.msg import *

def callback(msg):
    rospy.loginfo(msg.result.success)

if __name__ == '__main__':
    rospy.init_node('email_on_failure')
    
    sub = rospy.Subscriber("/autonomy/network_mission/result", 
                           clearpath_navigation_msgs.msg.ExecuteNetworkMissionByUuidActionResult, callback)
    
    
    rospy.spin()