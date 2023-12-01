#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray

class DroneAvoider:
    def __init__(self, ns):
        self.ns = ns
        rospy.init_node(self.ns + '_drone_avoider', anonymous=False)

        # Subscribers
        rospy.Subscriber(self.ns + '/ground_truth/state', Odometry, self.odometry_callback)
        rospy.Subscriber('/drones_positions', PoseArray, self.drones_positions_callback)

        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)

        # Variables
        self.robot_pose = None
        self.other_drones_positions = []

    def odometry_callback(self, msg):
        rospy.loginfo(f"{self.ns} Odometry callback triggered")
        rospy.loginfo(f"{self.ns} Current Pose: {msg.pose.pose}")
        self.robot_pose = msg.pose.pose
        self.calculate_and_move()

    def drones_positions_callback(self, msg):
        rospy.loginfo(f"{self.ns} Drones positions callback triggered")
        # Directly assign the positions from the message to other_drones_positions
        self.other_drones_positions = [pose for pose in msg.poses]
        rospy.loginfo(f"{self.ns} Other Drones' Positions: {self.other_drones_positions}")

    def calculate_repulsive_force(self):
        repulsive_force = np.array([0.0, 0.0])
        repulsion_radius = 0.9  # Adjust this value as needed
        drone_in_radius = False  # Flag to check if any drone is within the radius

        for other_drone in self.other_drones_positions:
            distance = np.linalg.norm(np.array([self.robot_pose.position.x, self.robot_pose.position.y]) - 
                                    np.array([other_drone.position.x, other_drone.position.y]))
            rospy.loginfo(f"{self.ns} Distance to other drone: {distance}")

            if distance < repulsion_radius:
                drone_in_radius = True
                force = (1 / distance - 1 / repulsion_radius) / distance**2
                direction = np.array([self.robot_pose.position.x, self.robot_pose.position.y]) - \
                            np.array([other_drone.position.x, other_drone.position.y])
                repulsive_force += force * direction
            else:
                rospy.loginfo(f"{self.ns} Other drone outside repulsion radius: Distance = {distance}")

        rospy.loginfo(f"{self.ns} Calculated repulsive force: {repulsive_force}")
        return repulsive_force, drone_in_radius

    def calculate_and_move(self):
        if not self.robot_pose:
            return

        repulsive_force, drone_in_radius = self.calculate_repulsive_force()

        cmd_vel_msg = Twist()
        if drone_in_radius:
            # Normalize the repulsive force to get the direction of movement
            if np.linalg.norm(repulsive_force) != 0:
                repulsive_force /= np.linalg.norm(repulsive_force)
                cmd_vel_msg.linear.x = repulsive_force[0]
                cmd_vel_msg.linear.y = repulsive_force[1]
        else:
            # Stop the drone if no other drones are within the repulsion radius
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0

        # Publish the Twist message to the cmd_vel topic
        rospy.loginfo(f"{self.ns} Publishing cmd_vel: linear x: {cmd_vel_msg.linear.x}, linear y: {cmd_vel_msg.linear.y}")
        self.cmd_vel_pub.publish(cmd_vel_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    drone_avoider = DroneAvoider(ns)
    drone_avoider.run()
