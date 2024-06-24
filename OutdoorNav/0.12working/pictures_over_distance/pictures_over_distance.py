#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry

from datetime import datetime
import cv2 as cv
import requests
import numpy as np

AXIS_HOST = "192.168.131.10"
AXIS_SNAPSHOT_API_URL = "http://" + AXIS_HOST + "/axis-cgi/jpg/image.cgi"

x_pos = 0
y_pos = 0
initial_distance = 0
last_pose = None
distance_travelled = 0

def captureSnapshot():
    resp = requests.get(AXIS_SNAPSHOT_API_URL, stream=True).raw
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv.imdecode(image, cv.IMREAD_COLOR)
    return image


def callback(msg):
    global distance_travelled
    global last_pose
    if last_pose is None:
        last_pose = msg
    dx = msg.pose.pose.position.x - last_pose.pose.pose.position.x
    dy = msg.pose.pose.position.y - last_pose.pose.pose.position.y
    distance_travelled  += math.sqrt(dx**2+dy**2)
    last_pose = msg
    
    

if __name__ == '__main__':
    rospy.init_node('pictures_over_distance')

    sub = rospy.Subscriber("localization/odom", Odometry, callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        distance = 2

        # if initial_distance ==0:
        #     initial_distance = math.sqrt(x_pos**2+y_pos**2)

        # actual_distance = math.sqrt(x_pos**2+y_pos**2) - initial_distance
        
        if (distance_travelled>=distance):
            file_name = str(datetime.now()) + ".jpg"
            img = captureSnapshot()
            dir = "/home/administrator/nathan_ws/pictures/"
            img_path = dir + file_name

            print("Saving file {}".format(img_path))
            cv.imwrite(img_path, img)
            
            distance_travelled = 0

        # if (actual_distance>=distance):
        #     file_name = str(datetime.now()) + ".jpg"
        #     img = captureSnapshot()
        #     dir = "/home/administrator/nathan_ws/pictures/"
        #     img_path = dir + file_name

        #     print("Saving file {}".format(img_path))
        #     cv.imwrite(img_path, img)
            
        #     initial_distance = 0

        rate.sleep()


   