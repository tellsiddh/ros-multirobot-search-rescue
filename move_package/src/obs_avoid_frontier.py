#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from copy import copy
import random

def range_idx(angle_min, angle_increment, angle):
    return round((angle - angle_min) / angle_increment)

class FrontierAvoider:
    def __init__(self, ns):
        if ns == "none":
            self.ns = ""
        else:
            self.ns = ns
        rospy.init_node(self.ns + '_frontier_avoider', anonymous=True)

        # Now it's safe to use rospy.get_time()
        unique_name_with_time = self.ns + '_frontier_avoider_' + str(rospy.get_time())
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Obstacle avoidance
        self.dir = random.choice([-1, 1])
        self.blocked = False

        # Frontier exploration
        self.resolution = None
        self.origin = None
        self.map_data = None
        self.width = None
        self.height = None
        self.frontiers = []
        self.current_goal = None

    def scan_callback(self, msg):
        fov = [range_idx(msg.angle_min, msg.angle_increment, -0.548),
               range_idx(msg.angle_min, msg.angle_increment, 0.548)]

        sub_ranges = np.array(msg.ranges[fov[0]:fov[1]+1])
        min_range = np.min(sub_ranges[sub_ranges > 0.09])

        if min_range < 1.5:
            if not self.blocked:
                self.blocked = True
                self.dir = random.choice([-1, 1])
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = 0.5 * self.dir
            self.cmd_vel_pub.publish(cmd_vel_msg)
        else:
            self.blocked = False
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.5
            self.cmd_vel_pub.publish(cmd_vel_msg)

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        self.find_frontiers()

    def find_frontiers(self):
        img = np.zeros((self.height, self.width, 1), dtype=np.uint8)
        for i in range(self.height):
            for j in range(self.width):
                img_value = self.map_data[i, j] 
                if img_value == 100:
                    img[i, j] = 0
                elif img_value == 0:
                    img[i, j] = 255
                elif img_value == -1:
                    img[i, j] = 205

        # Process the image to find frontiers
        o = cv2.inRange(img, 0, 1)
        edges = cv2.Canny(img, 0, 255)
        contours, _ = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(o, contours, -1, (255, 255, 255), 5)
        o = cv2.bitwise_not(o)
        res = cv2.bitwise_and(o, edges)

        frontier = copy(res)
        contours, _ = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)

        self.process_frontiers(contours)

    def process_frontiers(self, contours):
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                xr = cx * self.resolution + self.origin.x
                yr = cy * self.resolution + self.origin.y
                self.frontiers.append((xr, yr))

        self.select_closest_frontier_as_goal()

    def select_closest_frontier_as_goal(self):
        if self.frontiers:
            self.current_goal = min(self.frontiers, key=lambda pt: np.linalg.norm(np.array(pt)))

    def move_towards_goal(self):
        if self.current_goal and not self.blocked:
            # Simple movement towards the goal
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.5  # Move forward towards the goal
            self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.move_towards_goal()
            rate.sleep()

    def shutdown_callback(self):
        self.cmd_vel_pub.publish(Twist())  # Stop the drone

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    frontier_avoider = FrontierAvoider(ns)
    rospy.on_shutdown(frontier_avoider.shutdown_callback)
    frontier_avoider.run()
