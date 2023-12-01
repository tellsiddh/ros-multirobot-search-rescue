#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan

class DroneAvoider:
    def __init__(self, ns):
        self.ns = ns
        rospy.init_node(self.ns + '_drone_avoider', anonymous=False)

        # Subscribers
        rospy.Subscriber(self.ns + '/ground_truth/state', Odometry, self.odometry_callback)
        rospy.Subscriber('/drones_positions', PoseArray, self.drones_positions_callback)
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.scan_callback)

        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)

        # Variables
        self.robot_pose = None
        self.other_drones_positions = []
        self.obstacle_force = np.array([0.0, 0.0])
        self.in_repulsion_radius = False  # Flag for being in the repulsion radius
        self.initial_move = False  # Flag for initial movement

    def odometry_callback(self, msg):
        rospy.loginfo(f"{self.ns} Odometry callback triggered")
        rospy.loginfo(f"{self.ns} Current Pose: {msg.pose.pose}")
        self.robot_pose = msg.pose.pose
        self.calculate_and_move()

    def drones_positions_callback(self, msg):
        rospy.loginfo(f"{self.ns} Drones positions callback triggered")
        self.other_drones_positions = [pose for pose in msg.poses]
        rospy.loginfo(f"{self.ns} Other Drones' Positions: {self.other_drones_positions}")

    def scan_callback(self, msg):
        self.calculate_obstacle_force(msg)


    def calculate_obstacle_force(self, msg):
        # Example implementation of obstacle force calculation based on laser scan data
        force_mag = 0.0
        direction = np.array([0.0, 0.0])

        # Threshold distance for obstacle repulsion
        threshold_distance = 1

        for i, distance in enumerate(msg.ranges):
            if distance < threshold_distance:
                # Calculate repulsion force magnitude
                force_mag += 1.0 / (distance ** 2)

                # Calculate the angle of the scan point
                angle = msg.angle_min + i * msg.angle_increment
                # Add the force vector for this scan point
                direction += np.array([np.cos(angle), np.sin(angle)])

        # Normalize the direction vector
        if np.linalg.norm(direction) > 0:
            direction /= np.linalg.norm(direction)

        self.obstacle_force = force_mag * direction

    def calculate_repulsive_force(self):
        repulsive_force = np.array([0.0, 0.0])
        repulsion_radius = 0.9
        in_radius = False

        for other_drone in self.other_drones_positions:
            distance = np.linalg.norm(np.array([self.robot_pose.position.x, self.robot_pose.position.y]) - 
                                      np.array([other_drone.position.x, other_drone.position.y]))

            if distance < repulsion_radius:
                in_radius = True
                force = (1 / distance - 1 / repulsion_radius) / distance ** 2
                direction = np.array([self.robot_pose.position.x, self.robot_pose.position.y]) - \
                            np.array([other_drone.position.x, other_drone.position.y])
                repulsive_force += force * direction

        return repulsive_force, in_radius

    def calculate_and_move(self):
        if not self.robot_pose:
            return

        drone_force, drone_in_radius = self.calculate_repulsive_force()
        combined_force = drone_force + self.obstacle_force

        cmd_vel_msg = Twist()

        # Check if there's any force from drones or obstacles
        if np.linalg.norm(drone_force) > 0 or np.linalg.norm(self.obstacle_force) > 0:
            self.in_repulsion_radius = True
            combined_force /= (np.linalg.norm(combined_force) + 1e-6)  # Normalize and avoid division by zero
            cmd_vel_msg.linear.x = combined_force[0]
            cmd_vel_msg.linear.y = combined_force[1]
        else:
            self.in_repulsion_radius = False

        # If not in repulsion radius and not in initial movement, stop the drone
        if not self.in_repulsion_radius and not self.initial_move:
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
        elif self.initial_move:
            # If it's the initial move, set a random direction
            cmd_vel_msg.linear.x = random.uniform(-0.5, 0.5)
            cmd_vel_msg.linear.y = random.uniform(-0.5, 0.5)
            self.initial_move = False

        rospy.loginfo(f"{self.ns} Publishing cmd_vel: linear x: {cmd_vel_msg.linear.x}, linear y: {cmd_vel_msg.linear.y}")
        self.cmd_vel_pub.publish(cmd_vel_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    drone_avoider = DroneAvoider(ns)
    drone_avoider.run()