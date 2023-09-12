#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from erp42_serial.msg import ESerial
import math

erp_msg = ESerial()
cluster_center = None

pub = rospy.Publisher("erp42_serial", ESerial, queue_size = 1)

def drive_go():
    global erp_msg
    erp_msg.speed = 0  # Set an appropriate speed for going
    erp_msg.brake = 0
    erp_msg.steer = 0
    pub.publish(erp_msg)
    print('go')

def drive_stop():
    global erp_msg
    erp_msg.speed = 0
    erp_msg.brake = 150
    erp_msg.steer = 0
    pub.publish(erp_msg)
    print('stop')

def drive_right():
    global erp_msg
    erp_msg.speed = 0  # Set an appropriate speed for turning
    erp_msg.brake = 0
    erp_msg.steer = 2000  # Set an appropriate steering angle for right turn
    pub.publish(erp_msg)
    print('right')

def drive_left():
    global erp_msg
    erp_msg.speed = 0  # Set an appropriate speed for turning
    erp_msg.brake = 0
    erp_msg.steer = -2000  # Set an appropriate steering angle for right turn
    pub.publish(erp_msg)
    print('left')

def find_cluster_center(marker):
    num_points = len(marker.points)

    if num_points < 2 or num_points % 2 != 0:
        rospy.logwarn("Invalid marker format. Expected LINE_LIST with pairs of points.")
        return None

    center = Point()
    center.x = center.y = center.z = 0.0

    for point in marker.points:
        center.x += point.x
        center.y += point.y
        center.z += point.z

    center.x /= num_points
    center.y /= num_points
    center.z /= num_points

    return center

def euclidean_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def find_farthest_point(target_point, point_list):
    if not point_list:
        return None
    
    farthest_point = point_list[0]
    farthest_distance = euclidean_distance(target_point, farthest_point)

    for point in point_list:
        distance = euclidean_distance(target_point, point)
        if distance > farthest_distance:
            farthest_point = point
            farthest_distance = distance

    return farthest_point

# 기존 코드에서 필요한 부분 수정 및 추가
def marker_array_callback(msg):
    center_marker_array = MarkerArray()

    for marker in msg.markers:
        if marker.type == Marker.LINE_LIST:
            cluster_center = find_cluster_center(marker)
            
            if cluster_center:
                farthest_point = find_farthest_point(cluster_center, marker.points)
                angle = math.degrees(math.atan2(farthest_point.y, farthest_point.x))
                if (angle < 70 and angle >= 0) or (angle > -70 and angle <= 0):
                    continue
                
                center_marker = Marker()
                center_marker.header = marker.header
                center_marker.ns = "cluster_centers"
                center_marker.id = marker.id
                center_marker.type = Marker.SPHERE
                center_marker.action = Marker.ADD
                center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.1
                center_marker.color.a = 1.0
                center_marker.color.r = 1.0
                center_marker.color.g = 0.0
                center_marker.color.b = 0.0
                center_marker.pose.position = farthest_point
                center_marker.pose.orientation.w = 1.0
                center_marker_array.markers.append(center_marker)
                
    #print(center_marker_array.markers.pose)
    # center_marker_array를 원점에서 가까운 순으로 정렬
    sorted_markers = sorted(center_marker_array.markers, key=lambda marker: euclidean_distance(marker.pose.position, Point(0, 0, 0)))
    
    # 정렬된 마커들을 새로운 center_marker_array로 설정
    center_marker_array.markers = sorted_markers
    nearest_point = center_marker_array.markers[0].pose.position
    #print(nearest_point)
    #drive_func(nearest_point)
    center_marker_pub.publish(center_marker_array)
#rospy.loginfo("11111111111111111111111111")



def drive_func(cluster_center):
    distance = math.sqrt(cluster_center.x** 2 + cluster_center.y** 2)
    angle = math.degrees(math.atan2(cluster_center.y, cluster_center.x))
    
    if distance < 0.8:
            
        if angle >= 160 or angle <= -160:
            print(distance, angle)
            drive_stop()
        elif angle < 160 and angle >= 90:
            print(distance, angle)
            drive_left()
        elif angle < -90 and angle >= -160:
            print(distance, angle)
            drive_right()
        else:
   
            pass
    else:
        drive_go() 

if __name__ == "__main__":
    rospy.init_node("cluster_center_publisher")
    rospy.Subscriber("/adaptive_clustering/markers", MarkerArray, marker_array_callback)
    center_marker_pub = rospy.Publisher("center_marker_array", MarkerArray, queue_size=10)
    
    rospy.spin()

