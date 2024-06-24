#!/usr/bin/env python3

import random
from clearpath_mission_manager_msgs.srv import *
import rospy
from sensor_msgs.msg import *
from clearpath_navigation_msgs.msg import *
import actionlib




def create_and_run_mission(points):

    rospy.wait_for_service('/mission_manager/create_mission')
    mission_req = CreateMissionRequest()
    mission_req.name = "API Mission " + str(random.randint(0,1000)) 
    create_mission = rospy.ServiceProxy('/mission_manager/create_mission', CreateMission)
    result = create_mission.call(mission_req)
    rospy.loginfo("Created mission '{}', UUID: {}".format(mission_req.name, result.result.uuid))
    mission_uuid = result.result.uuid

    create_waypoint = rospy.ServiceProxy('/mission_manager/create_waypoint', CreateWaypoint)

    global wp_result 
    wp_result = CreateWaypointResponse()

    
    for point in points:
        req = CreateWaypointRequest()
        req.latitude = point[0] 
        req.longitude = point[1] 
        req.heading = -1
        req.assign_to.append(mission_uuid)
        req.position_tolerance = -1.0
        req.yaw_tolerance = -1.0
        wp_result = create_waypoint.call(req)

    start_time = rospy.get_time()
    rospy.sleep(0.5)
    mission_client = actionlib.SimpleActionClient('/mission/by_id', clearpath_navigation_msgs.msg.ExecuteMissionByUuidAction)
    mission_client.wait_for_server()
    mission_goal = clearpath_navigation_msgs.msg.ExecuteMissionByUuidGoal()
    mission_goal.uuid = mission_uuid
    mission_goal.from_start = True
    rospy.loginfo("    Starting mission...")
    mission_client.send_goal(mission_goal)
    mission_client.wait_for_result()
    result = mission_client.get_result()
    #state = mission_client.get_state()
    end_time = rospy.get_time()
    duration = round(end_time - start_time, 2)
    
    rospy.loginfo("    Mission: {}, Success: {}, Duration: {}".format(mission_uuid, result, duration))

    

if __name__ == '__main__':
    
    points = [(43.5005924, -80.5471741),
              (43.5005002, -80.5471107)
    ]
    rospy.init_node('create_run_mission')
    create_and_run_mission(points)

    exit()
