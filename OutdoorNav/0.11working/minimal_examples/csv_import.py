#!/usr/bin/env python3

import random
from clearpath_mission_manager_msgs.srv import *
import rospy
from sensor_msgs.msg import *
from clearpath_navigation_msgs.msg import *
#import actionlib
import csv

def readCSV():
    coordinates = []

    with open('test.csv') as file:
        reader = csv.DictReader(file)
        for row in reader:
            coordinates.append((row['Latitude'], row['Longitude']))

    return coordinates


def createMission(points):

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
        req.latitude = float(point[0]) 
        req.longitude = float(point[1]) 
        req.heading = -1
        req.assign_to.append(mission_uuid)
        req.position_tolerance = -1.0
        req.yaw_tolerance = -1.0
        wp_result = create_waypoint.call(req)


if __name__ == '__main__':
    
    points = readCSV()
    rospy.init_node('create_run_mission')
    createMission(points)

    exit()
