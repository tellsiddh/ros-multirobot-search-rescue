#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random

def range_idx(angle_min, angle_increment, angle):
    return round((angle - angle_min) / angle_increment)

class Avoider:
    def __init__(self, ns, delay):
        if ns == "none":
            self.ns = ""
        else:
            self.ns = ns

        rospy.init_node(self.ns + 'avoider', anonymous=False)
        rospy.sleep(delay)
        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown_callback)

        self.repulsion_factor = 5
        self.obstacle_range = 10

    def repulsive_force(self, d_obstacle):
        # Repulsive potential
        if d_obstacle < self.obstacle_range:
            F_rep = self.repulsion_factor / (d_obstacle ** 2)
        else:
            F_rep = 0
        return F_rep

    def scan_callback(self, msg):
        fov = [range_idx(msg.angle_min, msg.angle_increment, -0.548),
               range_idx(msg.angle_min, msg.angle_increment, 0.548)]

        sub_ranges = np.array(msg.ranges[fov[0]:fov[1]+1])
        min_range = np.min(sub_ranges[sub_ranges > 0.09])

        F_rep = self.repulsive_force(min_range)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = max(0.1, 0.5 - F_rep)  # Decrease speed when close to an obstacle
        cmd_vel_msg.angular.z = F_rep  # Turn away from the obstacle
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def shutdown_callback(self):
        cmd_vel_msg = Twist()
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    delay = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    controller = Avoider(ns, delay)
    controller.run()
