#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarienceStamped
from std_msgs.msg import String
from nav_msgs.msg import Point #PoseWithCovarienceStamped
from visualization_msgs.msg import Marker

import time
from math import pi
import numpy as np

class Slam_Drive():
    def __init__(self):

        self.goal_pub = rospy.Publisher("/move_base/goal", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.path_sub = rospy.Subscriber("_path", String , self.path_callback)
        self.odom_sub = rospy.Subscriber("/robot_pose", PoseWithCovarienceStamped, self.odom_callback)
        self.wheelbase = 0.21
        self.aeb = 0
        #v2x variable
        self.path_topic = "x"
        self.mode = "x"

        self.poses = [0.0, 0.0]
        
        # ref and path [x,y] 
        self.ref = np.genfromtxt("/home/agilex/way.csv", delimiter=',', skip_header = 1)
        self.path_a = np.genfromtxt("/home/agilex/path_a.csv", delimiter=',', skip_header = 1)
        self.path_b = np.genfromtxt("/home/agilex/path_b.csv", delimiter=',', skip_header = 1)
        self.path_c = np.genfromtxt("/home/agilex/path_c.csv", delimiter=',', skip_header = 1)
        self.now_path = self.ref

    def points_marker(self,points, frame_id="map", ns="points", id=0, color=(1,0,0)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.ns = ns
        marker.id = id

        marker.scale.x = 0.2  # specifies the radius of the points
        marker.scale.y = 0.2
 
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.points = []

        p = Point()
        p.x, p.y = points
            #print("point: ",point)
            #print("p.x: ",p.x)
            #print("p.y: ",p.y)
            #print("p.z: ",p.z)
        marker.points.append(p)
        #marker.points = p

        return marker
    
    def find_lookahead_idx(self, pose_x, pose_y, ref):
        start = time.time()

        #find the lookahead index directly in front of the car
        self.lookahead_distance = 1.0
        distances = np.sqrt((ref[:, 0]) ** 2 + (ref[:, 1]) ** 2)
        #print(distances[0:6])
        #distances = np.sqrt((ref[:, 0] - pose_x) ** 2 + (ref[:, 1] - pose_y) ** 2)
        #min_idx = np.argmin(distances)       ##find index of minimum value
        min_idx = np.argmin(distances)       ##too slow function fucking 
        #print(min_idx)
        #lookahead_idx = min_idx      
        lookahead_idx = min_idx
       
        while True:
            lookahead_idx += 1
            if lookahead_idx >= len(ref):
                lookahead_idx = 0
            #dist_to_lookahead = np.hypot(ref[lookahead_idx, 0] - pose_x, ref[lookahead_idx, 1] - pose_y)  ##calculate distance between robot pose with ref of lookahead
            dist_to_lookahead = np.hypot(ref[lookahead_idx, 0], ref[lookahead_idx, 1])
            #dist_to_lookahead = np.sqrt(ref[lookahead_idx, 0]**2 + )
            if dist_to_lookahead >= self.lookahead_distance:   ##if waypoint is farther than the lookahead distance
                break

        return lookahead_idx
    
    def odom_callback(self,data):
        self.poses = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        #self.quat = data.pose.pose.orientation        

    def path_callback(self, data):
        self.path_topic = data.data
        print("now path : ", self.path_topic)
                #v2x
        if self.path_topic == "A":
            self.now_path = self.path_a

        elif self.path_topic == "B":
            self.now_path = self.path_b

        elif self.path_topic == "C":
            self.now_path = self.path_c

        ## or 
        '''if self.path_topic == "A":
            self.now_path.append(self.path_a)

        elif self.path_topic == "B":
            self.now_path.append(self.path_b)

        elif self.path_topic == "C":
            self.now_path.append(self.path_c)
        '''
            
    def scan_callback(self,lidar):
        lidar_ranges = lidar.ranges
        front_scan = lidar_ranges[150:280]

        if front_scan < 0.3:
            self.aeb = 1
        else:
            self.aeb = 0


    def main(self):
        pose_x = self.poses[0]
        pose_y = self.poses[1]

        lookahead_idx = self.find_lookahead_idx(pose_x, pose_y, self.now_path)

        print('now pose_x : ',pose_x, 'pose_y : ', pose_y) # now robot pose

        lookahead_point2 = self.now_path[lookahead_idx]
        lookahead_point2 = lookahead_point2[:2]
        print('lookahead_point2: ',lookahead_point2)
        #print(self.global_ref)
        marker = self.points_marker(lookahead_point2)


        drive_msg = PoseWithCovarienceStamped()
        drive_msg.header.stamp = rospy.Time.now()

        # publish move_base goal
        goal_x = self.now_path[:,0]
        goal_y = self.now_path[:,1]
        
        drive_msg.pose.pose.position.x = goal_x[lookahead_idx]
        drive_msg.pose.pose.position.y = goal_y[lookahead_idx]

        if self.aeb == 1 :
            drive_msg.pose.pose.position.x = goal_x[lookahead_idx - 1]
            drive_msg.pose.pose.position.y = goal_y[lookahead_idx - 1]

        self.goal_pub.publish(drive_msg)
        
if __name__ == "__main__":
    try:
        rospy.init_node("slam_drive")
        slam_drive = Slam_Drive()

        while not rospy.is_shutdown():
            slam_drive.main()
            rospy.Rate(60).sleep()

    except KeyboardInterrupt:
        print("code check plz") 
