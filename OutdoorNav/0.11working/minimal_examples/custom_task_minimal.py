#!/usr/bin/env python

import rospy
import actionlib
import subprocess
# from your_action_package.msg import YourAction, YourActionGoal, YourActionResult, YourActionFeedback
from clearpath_navigation_msgs.msg import UITaskAction, UITaskActionGoal, UITaskActionResult, UITaskActionFeedback
#from datetime import datetime
#import rosnode



class CustomTaskMinimal:
    _feedback = UITaskActionFeedback()
    _result = UITaskActionResult()
    def __init__(self):
        self._as = actionlib.SimpleActionServer("/action_server", UITaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        
        #put any code here
        rospy.loginfo("Testing custom tasks")
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('custom_action_server')
    server = CustomTaskMinimal()
    rospy.spin()
