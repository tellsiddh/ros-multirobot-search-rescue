#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseArray, Twist
from sensor_msgs.msg import LaserScan
import sys
import numpy as np

class PotentialFieldController:
    def __init__(self, ns):
        self.ns = ns
        self.obstacle_range = 1
        self.robot_range = 5 # 5
        self.k_rep = 5
        self.robot_position = None
        self.other_robots = None
        self.obstacle_off = [0, 0]

        # ROS publishers and subscribers
        self.vel_pub = rospy.Publisher(f'{ns}/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('drones_positions', PoseArray, self.robot_positions_callback)
        rospy.Subscriber(f'{ns}/scan', LaserScan, self.scan_callback)

        rospy.on_shutdown(self.shutdown_handler)

    def robot_positions_callback(self, msg):
        # Extract positions of other robots based on the provided namespace
        if self.ns == "":
            # If the namespace is "", include poses 2 and 3
            self.robot_position = msg.poses[0]
            self.other_robots = [msg.poses[1], msg.poses[2]]
        elif self.ns == "uav1":
            # If the namespace is "uav1", include poses 1 and 3
            self.robot_position = msg.poses[1]
            self.other_robots = [msg.poses[0], msg.poses[2]]
        elif self.ns == "uav2":
            # If the namespace is "uav2", include poses 1 and 2
            self.robot_position = msg.poses[2]
            self.other_robots = [msg.poses[0], msg.poses[1]]

    def scan_callback(self, msg):
        # Get the distance to the closest obstacle in front of the robot
        fixed_range = [value for value in msg.ranges if 0.1 < value < self.obstacle_range]
        if not fixed_range:
            self.obstacle_off = [0, 0]
        else:
            obstacle_distance = min(fixed_range)
            min_dist_index = msg.ranges.index(obstacle_distance)
            obstacle_angle = min_dist_index * msg.angle_increment + msg.angle_min
            #print('obstacle angle: ' + str(obstacle_angle * 180/math.pi))
            x_off = obstacle_distance * math.sin(obstacle_angle)
            y_off = obstacle_distance * math.cos(obstacle_angle)
            #print('obstacle angle: ' + str(math.atan2(y_off,x_off) * 180/math.pi))
            self.obstacle_off = [x_off, y_off]

    def repulsive_potential(self, q, q_i, p_0):
        q = np.array(q)
        q_i = np.array(q_i)
        p = np.linalg.norm(q - q_i)
        if p <= p_0 and p != 0:
            return self.k_rep * (1/p - 1/p_0) * (q - q_i) / (p**3)
        else:
            return np.zeros_like(q)

    def calculate_total_repulsive_force(self):
        total_force = np.array([0.0, 0.0])
        q = np.array([-self.robot_position.position.y, self.robot_position.position.x])

        # Repulsion from other robots
        for other_robot in self.other_robots:
            q_i = np.array([-other_robot.position.y, other_robot.position.x])
            force = self.repulsive_potential(q, q_i, self.robot_range)
            total_force += force

        # print('drone repulsion angle: ' + str(math.atan2(total_force[1],total_force[0]) * 180/math.pi))
        # print('drone repulsion mag: ' + str(np.linalg.norm(total_force)))
        # print('drone total force: ' + str(total_force))

        # Repulsion from obstacles
        q_obs = q + np.array(self.obstacle_off)
        print('off: ' + str(self.obstacle_off))
        print('q: ' + str(q))
        print('q_obs: ' + str(q_obs))
        force = self.repulsive_potential(q, q_obs, self.obstacle_range)
        total_force += force

        return total_force
    
    def shutdown_handler(self):
        # Stop the robot when the script is shutdown
        twist = Twist()
        for i in range(0,2):
            self.vel_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.robot_position is not None:
                total_force = self.calculate_total_repulsive_force()
                print('repulsion angle: ' + str(math.atan2(total_force[1],total_force[0]) * 180/math.pi))
                print('repulsion mag: ' + str(np.linalg.norm(total_force)))
                #print('total force: ' + str(total_force))
                if np.linalg.norm(total_force) != 0:
                    total_force /= np.linalg.norm(total_force)
                    total_force /= 2
                print('changed force: ' + str(total_force))
                twist = Twist()
                twist.linear.x = total_force[1]
                twist.linear.y = -total_force[0]
                #print(twist)
                self.vel_pub.publish(twist)
                #print(twist)
            rate.sleep()

if __name__ == "__main__":

    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    if ns == 'none':
        ns = ""

    rospy.init_node(ns + 'potential_field_controller')

    controller = PotentialFieldController(ns)
    controller.run()
