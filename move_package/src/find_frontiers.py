#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
import cv2
from copy import copy
import sys
from rospy.timer import TimerEvent

class Uncharter:
    def __init__(self,ns):
        
        self.ns = ns

        rospy.init_node(self.ns + 'uncharter', anonymous=False)

        # Create subscribers for the map and initialpose topics
        rospy.Subscriber(self.ns + '/map', OccupancyGrid, self.map_callback)

        self.frontier_pub = rospy.Publisher(self.ns + '/frontiers', Float32MultiArray, queue_size=10)

        # Initialize variables to store map information
        self.resolution = None
        self.x_origin = None
        self.y_origin = None
        self.width = None
        self.height = None
        self.data = None
        self.all_pts = None

        # Set up a timer for periodically publishing the current frontier
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_current_frontiers)

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.x_origin = msg.info.origin.position.x
        self.y_origin = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = msg.data

        self.display_map()

    def display_map(self):
        img = np.zeros((self.height, self.width, 1), dtype=np.uint8)
        for i in range(0, self.height):
            for j in range(0, self.width):
                img_value = self.data[i * self.width + j]
                if img_value == 100:
                    img[i, j] = 0
                elif img_value == 0:
                    img[i, j] = 255
                elif img_value == -1:
                    img[i, j] = 205

        o = cv2.inRange(img, 0, 1)
        edges = cv2.Canny(img, 0, 255)
        contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(o, contours, -1, (255, 255, 255), 5)
        o = cv2.bitwise_not(o)
        res = cv2.bitwise_and(o, edges)

        frontier = copy(res)
        contours, hierarchy = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)

        contours, hierarchy = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        all_pts = []
        area_threshold = 100  # Adjust this threshold as needed

        for i in range(0, len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area > area_threshold:
                M = cv2.moments(cnt)
                try:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(img, (cx, cy), 5, (128, 128, 128), -1)
                    xr = cx * self.resolution + self.x_origin
                    yr = cy * self.resolution + self.y_origin
                    pt = [np.array([xr, yr])]
                    if len(all_pts) > 0:
                        all_pts = np.vstack([all_pts, pt])
                    else:
                        all_pts = pt
                except:
                    pass
        
        self.all_pts = all_pts
        cv2.imshow(self.ns + '/map + ' + self.ns + '/frontiers', img)
        cv2.waitKey(1)

    def publish_current_frontiers(self,event:TimerEvent):
        if self.all_pts is not None and np.size(self.all_pts) > 1:
            pub_msg = Float32MultiArray()
            if np.size(self.all_pts) > 2:
                pub_msg.data = self.all_pts.flatten()
            else:
                pub_msg.data = self.all_pts
            pub_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
            pub_msg.layout.dim[0].label = "points"
            pub_msg.layout.dim[0].size = np.shape(self.all_pts)[0]
            pub_msg.layout.dim[1].label = "dimensions"
            pub_msg.layout.dim[1].size = np.shape(self.all_pts)[1]
            self.frontier_pub.publish(pub_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    
    Uncharter(ns).run()
