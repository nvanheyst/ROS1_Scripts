from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points

from geopy.distance import great_circle
import simplekml
import random
from clearpath_mission_manager_msgs.srv import *
from clearpath_mission_manager_msgs.srv import CreateWaypointRequest
import rospy
import utm
from functools import partial
import geopy.distance
from sensor_msgs.msg import *
from random import choice
from string import ascii_uppercase
import actionlib
from clearpath_navigation_msgs.msg import *
import time
import math


def latlon_to_utm(lat, lon):
    return utm.from_latlon(lat, lon)[:2]

def utm_to_latlon(utm_x, utm_y, zone_number, zone_letter):
    return utm.to_latlon(utm_x, utm_y, zone_number, zone_letter)


def generate_random_point_in_polygon(poly):
    minx, miny, maxx, maxy = poly.bounds
    while True:
        p = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
        if poly.contains(p):
            return p

def is_within_distance(point1, point2, min_distance, max_distance):
    distance = point1.distance(point2)
    return min_distance <= distance <= max_distance

def is_valid_next_point(new_point, last_points, min_distance, max_distance):
    if len(last_points)==1:
        return True
    for point in last_points:
        distance = new_point.distance(point)
        # if (distance > max_distance or distance < min_distance):
        if (distance < min_distance):
            return False
    return True

def generate_point_nearby_bearing(base_point, min_distance, max_distance):
    bearing = random.uniform(0, 360)
    distance = random.uniform(min_distance, max_distance)
    destination = great_circle(distance=distance).destination((base_point.y, base_point.x), bearing)
    return Point(destination.longitude, destination.latitude)


def generate_point_nearby(base_point, last_points, min_distance, max_distance):
    has_point = False
    next_point = Point()
    while not has_point:
        dx = 0
        dy = 0
        searching = True
        while searching:
            dx = random.uniform(-max_distance, max_distance)
            dy = random.uniform(-max_distance, max_distance)
            if (abs(dx) > 2 and abs(dy) > 2):
                searching = False
        new_point = Point(base_point.x + dx, base_point.y + dy)
        if is_valid_next_point(new_point, last_points, min_distance, max_distance):
            has_point = True
            next_point = new_point
    return next_point



def is_within_distance_from_last_points(new_point, last_points, min_distance, max_distance):
    for point in last_points:
        distance = geopy.distance.great_circle((new_point.y, new_point.x), (point.y, point.x)).meters
        if not (min_distance <= distance <= max_distance):
            return False
    return True


def does_not_intersect_previous_lines(new_line, previous_lines, last_n_points):
    for line in previous_lines[-last_n_points:]:
        if new_line.intersects(line):
            return False
    return True

def calculate_angle(line):
    dx = line.coords[1][0] - line.coords[0][0]
    dy = line.coords[1][1] - line.coords[0][1]
    return math.degrees(math.atan2(dy, dx))

# Define your polygon here with latitude-longitude vertices
polygon_vertices = [(43.50071701522529, -80.54734736506819),
(43.50068977706367, -80.5472568405128),
(43.500663998077975, -80.54722063069066),
(43.50067275320642, -80.54717704479361),
(43.50048354486847, -80.54703555826661),
(43.50039356126057, -80.54705366317772),
(43.50037167333583, -80.54711334232906)]

# polygon_vertices = [
# (50.1096132, -97.3191528),
# (50.1095988, -97.3187261),
# (50.1093205, -97.3187230),
# (50.1093463, -97.3191517)]

polygon = Polygon(polygon_vertices)

# Convert polygon vertices to UTM
utm_polygon_vertices = [latlon_to_utm(lat, lon) for lat, lon in polygon_vertices]

# Create a UTM polygon
utm_polygon = Polygon(utm_polygon_vertices)

timeout_seconds = 0.15

rospy.init_node("my_node")

# Attempt to generate points
def attempt_to_generate_points(starting_point, num_points, num_points_check, num_line_check, min_distance, max_distance):
    points = [starting_point]
    lines = []
    start_time = time.time()

    while len(points) < num_points:
        current_point = points[-1]

        ## Generate new point
        has_point = False
        while not has_point:
            if time.time() - start_time > timeout_seconds:
                return None
            next_point = Point()
            dx = random.uniform(-max_distance, max_distance)
            dy = random.uniform(-max_distance, max_distance)
            next_point = Point(current_point.x + dx, current_point.y + dy)
            next_line = LineString([points[-1], next_point])


            ## Check distance to 'num_prev_points' prior points
            invalid_point = False
            for point in points[-num_points_check:]:
                distance = next_point.distance(point)
                if distance < min_distance:
                    invalid_point = True
                    break
            if invalid_point:
                continue
            
            invalid_line = False
            for line in lines[-num_line_check:]:
                if next_line.intersects(line) and not next_line.touches(line):
                    invalid_line = True
                    break
                else:
                    if next_line.distance(line) < 0.8*min_distance and not next_line.touches(line):
                        invalid_line = True
                        break
            if invalid_line:
                continue
            
            ## Check angle with previous line
            if len(lines) > 0:
                angle_new_line = calculate_angle(next_line)
                angle_last_line = calculate_angle(lines[-1])
                angle_difference = abs(angle_new_line - angle_last_line)
                if (angle_difference > 150 and angle_difference < 210):
                    # rospy.loginfo("Angle difference: {}".format(angle_difference))
                    continue

            ## Check if it is in polygon
            if not utm_polygon.contains(next_point):
                continue

            if next_line.intersects(utm_polygon.boundary):
                continue
            
            # def is_far_enough_from_polygon_border(new_point, polygon, min_border_distance):
            if len(points) > (num_points - 2) and next_point.distance(utm_polygon.boundary) <= 1.0:
                continue
                
            has_point = True
            points.append(next_point)
            lines.append(next_line)
    
    end_time = time.time()
    rospy.loginfo("Generated points in {} seconds".format(end_time-start_time))
    return points

def create_and_run_mission(num_points, num_points_check, num_line_check, min_distance, max_distance):
    robot_gps_pose = rospy.wait_for_message('/sensors/gps_0/fix', NavSatFix, timeout=5)

    current_point = Point(latlon_to_utm(robot_gps_pose.latitude, robot_gps_pose.longitude))

    for i in range(10000):
        rospy.loginfo("Attempt {} to generate points...".format(i))
        points = attempt_to_generate_points(current_point, num_points=num_points, num_points_check=num_points_check, num_line_check=num_line_check, min_distance=min_distance, max_distance=max_distance)
        if points is not None:
            break

    rospy.wait_for_service('/mission_manager/create_mission')
    mission_req = CreateMissionRequest()
    rnd = ''.join(choice(ascii_uppercase) for i in range(5))
    mission_req.name = "Random mission " + rnd
    create_mission = rospy.ServiceProxy('/mission_manager/create_mission', CreateMission)
    result = create_mission.call(mission_req)
    rospy.loginfo("Created mission '{}', UUID: {}".format(mission_req.name, result.result.uuid))
    mission_uuid = result.result.uuid

    create_waypoint = rospy.ServiceProxy('/mission_manager/create_waypoint', CreateWaypoint)
    create_task = rospy.ServiceProxy('/mission_manager/create_task', CreateTask)

    wp_result = CreateWaypointResponse()
        
    utm_zone_number, utm_zone_letter = utm.from_latlon(polygon_vertices[0][0], polygon_vertices[0][1])[2:]

    req = CreateWaypointRequest()
    latlon = utm_to_latlon(current_point.x, current_point.y, utm_zone_number, utm_zone_letter)
    req.latitude = latlon[0] #point.x
    req.longitude = latlon[1] #point.y
    req.heading = -1
    req.assign_to.append(mission_uuid)
    req.position_tolerance = -1.0
    req.yaw_tolerance = -1.0
    wp_result = create_waypoint.call(req)
    for point in points:
        req = CreateWaypointRequest()
        latlon = utm_to_latlon(point.x, point.y, utm_zone_number, utm_zone_letter)
        req.latitude = latlon[0] #point.x
        req.longitude = latlon[1] #point.y
        req.heading = -1
        req.assign_to.append(mission_uuid)
        req.position_tolerance = -1.0
        req.yaw_tolerance = -1.0
        wp_result = create_waypoint.call(req)
        
    wait_task = CreateTaskRequest()
    wait_task.action_server_name = "/wait"
    wait_task.name = "Wait 5 seconds"
    wait_task.floats.append(5.0)
    wait_task.assign_to_wp.append(wp_result.result.uuid)
    create_task.call(wait_task)

    # record_start = CreateTaskRequest()
    # record_start.action_server_name = "/rosbag_record"
    # record_start.name = "Start Rosbag"
    # record_start.strings.append("start")
    # record_start.strings.append(rnd)
    # record_start.assign_on_start.append(mission_uuid)
    # record_start.allow_failure = True
    # create_task.call(record_start)

    # record_stop = CreateTaskRequest()
    # record_stop.action_server_name = "/rosbag_record"
    # record_stop.name = "Stop Rosbag"
    # record_stop.strings.append("stop")
    # record_stop.strings.append(rnd)
    # record_stop.assign_on_stop.append(mission_uuid)
    # record_stop.allow_failure = True
    # create_task.call(record_stop)

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
    state = mission_client.get_state()
    end_time = rospy.get_time()
    res = {'result' : result, 'state' : state, 'mission_uuid' : mission_uuid, 'duration' : round(end_time - start_time, 2)}
    return res

delete_mission_service = rospy.ServiceProxy('/mission_manager/delete_mission', DeleteById)
delete_orphans_service = rospy.ServiceProxy('/mission_manager/delete_orphan_objects', DeleteEverything)

for x in range(10):
    result = create_and_run_mission(num_points=10, num_points_check=9, num_line_check=9, min_distance=5.0, max_distance=10.0)
    mission_uuid = result['mission_uuid']
    success = result['result'].success
    duration = result['duration']
    rospy.loginfo("    Mission: {}, Success: {}, Duration: {}".format(mission_uuid, success, duration))
    if success:
        if (delete_mission_service.call(mission_uuid)):
            rospy.loginfo("    Successfully deleted mission.")
        else:
            rospy.logwarn("    Deleting mission failed.")
        if delete_orphans_service.call(True):
            rospy.loginfo("    Successfully cleaned up old missions.")
        else:
            rospy.logwarn("    Failed to clean up old missions.")
    else:
        rospy.logerr("    Mission {} failed.".format(mission_uuid))

exit()

for i in range(10):
    result = create_and_run_mission()
    mission_uuid = result['mission_uuid']
    success = result['result'].success
    duration = result['duration']
    rospy.loginfo("    Mission: {}, Success: {}, Duration: {}".format(mission_uuid, success, duration))
    if success:
        if (delete_mission_service.call(mission_uuid)):
            rospy.loginfo("    Successfully deleted mission.")
        else:
            rospy.logwarn("    Deleting mission failed.")
        if delete_orphans_service.call(True):
            rospy.loginfo("    Successfully cleaned up old missions.")
        else:
            rospy.logwarn("    Failed to clean up old missions.")
    else:
        rospy.logerr("    Mission {} failed.".format(mission_uuid))
