#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math

class DroneAvoider:
    def __init__(self, ns, other_drones):
        self.ns = "" if ns == "none" else ns
        rospy.init_node(self.ns + '_drone_avoider', anonymous=True)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.my_odom_sub = rospy.Subscriber(self.ns + '/odom', Odometry, self.my_odom_callback)

        # Parameters
        self.repulsive_strength = 1.0
        self.obstacle_distance_threshold = 1.0
        self.drone_distance_threshold = 1.0
        self.my_position = None
        self.other_drones_positions = {}

        # Subscribe to other drones' positions
        for drone in other_drones:
            rospy.Subscriber(drone + '/odom', Odometry, self.other_drones_callback, drone)

    def my_odom_callback(self, msg):
        self.my_position = msg.pose.pose.position

    def other_drones_callback(self, msg, drone_ns):
        self.other_drones_positions[drone_ns] = msg.pose.pose.position

    def scan_callback(self, msg):
        force_x, force_y = 0, 0
        for i, distance in enumerate(msg.ranges):
            if distance < self.obstacle_distance_threshold:
                angle = i * msg.angle_increment + msg.angle_min
                force_magnitude = self.repulsive_strength / (distance ** 2)
                force_x += force_magnitude * math.cos(angle)
                force_y += force_magnitude * math.sin(angle)

        # Combine forces from drones
        drone_force_x, drone_force_y = self.check_and_avoid_drones()
        total_force_x = force_x + drone_force_x
        total_force_y = force_y + drone_force_y

        # Apply combined forces
        self.apply_repulsive_force((total_force_x, total_force_y))

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))

    def check_and_avoid_drones(self):
        force_x, force_y = 0, 0
        for drone_ns, position in self.other_drones_positions.items():
            if self.is_close_to_drone(position):
                drone_force = self.calculate_drone_avoidance_force(position)
                force_x += drone_force[0]
                force_y += drone_force[1]
        return force_x, force_y

    def is_close_to_drone(self, drone_position):
        if self.my_position is None:
            return False
        distance = math.sqrt((self.my_position.x - drone_position.x) ** 2 +
                             (self.my_position.y - drone_position.y) ** 2 +
                             (self.my_position.z - drone_position.z) ** 2)
        return distance < self.drone_distance_threshold

    def calculate_drone_avoidance_force(self, drone_position):
        relative_x = drone_position.x - self.my_position.x
        relative_y = drone_position.y - self.my_position.y
        distance = math.sqrt(relative_x ** 2 + relative_y ** 2)
        force_magnitude = self.repulsive_strength / (distance ** 2)
        force_angle = math.atan2(relative_y, relative_x) + math.pi
        return force_magnitude * math.cos(force_angle), force_magnitude * math.sin(force_angle)

    def apply_repulsive_force(self, force):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = force[0]
        cmd_vel_msg.linear.y = force[1]
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else "none"
    other_drones = ['uav1', 'uav2']  # Example other drones
    drone_avoider = DroneAvoider(ns, other_drones)
    drone_avoider.run()
