#!/usr/bin/env python3

import rospy
import numpy as np
import sys
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

    def odometry_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.calculate_and_move()

    def drones_positions_callback(self, msg):
        self.other_drones_positions = [pose for pose in msg.poses]

    def scan_callback(self, msg):
        self.calculate_obstacle_force(msg)

    def calculate_obstacle_force(self, msg):
        force_mag = 0.0
        direction = np.array([0.0, 0.0])
        threshold_distance = 2  # Set the obstacle detection threshold distance

        for i, distance in enumerate(msg.ranges):
            if distance < threshold_distance:
                force_mag += 1.0 / (distance ** 2)
                angle = msg.angle_min + i * msg.angle_increment
                direction += np.array([np.cos(angle), np.sin(angle)])

        if np.linalg.norm(direction) > 0:
            direction /= np.linalg.norm(direction)

        self.obstacle_force = force_mag * direction

    def calculate_repulsive_force(self):
        repulsive_force = np.array([0.0, 0.0])
        repulsion_radius = 0.9  # Set the drone detection threshold distance

        for other_drone in self.other_drones_positions:
            distance = np.linalg.norm(np.array([self.robot_pose.position.x, self.robot_pose.position.y]) - 
                                      np.array([other_drone.position.x, other_drone.position.y]))

            if distance < repulsion_radius:
                force = (1 / distance - 1 / repulsion_radius) / distance ** 2
                direction = np.array([self.robot_pose.position.x, self.robot_pose.position.y]) - \
                            np.array([other_drone.position.x, other_drone.position.y])
                repulsive_force += force * direction

        return repulsive_force

    def calculate_and_move(self):
        if not self.robot_pose:
            return

        drone_force = self.calculate_repulsive_force()
        combined_force = drone_force + self.obstacle_force

        cmd_vel_msg = Twist()

        if np.linalg.norm(combined_force) > 0:
            combined_force /= np.linalg.norm(combined_force)  # Normalize the force vector
            cmd_vel_msg.linear.x = combined_force[0]
            cmd_vel_msg.linear.y = combined_force[1]
        else:
            # Stop the drone if no repulsive forces are acting on it
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0

        self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    drone_avoider = DroneAvoider(ns)
    drone_avoider.run()
