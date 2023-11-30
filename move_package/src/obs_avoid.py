#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math

def range_idx(angle_min, angle_increment, angle):
    return round((angle - angle_min) / angle_increment)

class DroneAvoider:
    def __init__(self, ns):
        if ns == "none":
            self.ns = ""
        else:
            self.ns = ns
        rospy.init_node(self.ns + '_drone_avoider', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.repulsive_strength = 1.0 
        self.obstacle_distance_threshold = 1.0 
        self.drone_distance_threshold = 1.0
        self.other_drones_positions = {}
        for drone in other_drones:
            rospy.Subscriber(drone + '/ground_truth/state', Odometry, self.other_drones_callback, drone)

    def scan_callback(self, msg):
        closest_obstacle_distance = min(msg.ranges)
        if closest_obstacle_distance < self.obstacle_distance_threshold:
            obstacle_angle = msg.ranges.index(closest_obstacle_distance) * msg.angle_increment + msg.angle_min
            rep_force = self.calculate_repulsive_force(obstacle_angle, closest_obstacle_distance)
            self.apply_repulsive_force(rep_force)
        else:
            self.cmd_vel_pub.publish(Twist())

    def other_drones_callback(self, msg, drone_ns):
        self.other_drones_positions[drone_ns] = msg.pose.pose.position

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        self.find_frontiers()

    def calculate_repulsive_force(self, angle, distance):
        force_magnitude = self.repulsive_strength / (distance ** 2)
        force_angle = angle + math.pi  # 180 degrees from the obstacle - It is still hitting the obstacles. Need to fix it
        return force_magnitude, force_angle

    def apply_repulsive_force(self, force):
        force_magnitude, force_angle = force
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = force_magnitude * math.cos(force_angle)
        cmd_vel_msg.linear.y = force_magnitude * math.sin(force_angle)
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    other_drones = ['uav1', 'uav2']
    drone_avoider = DroneAvoider(ns)
    drone_avoider.run()
