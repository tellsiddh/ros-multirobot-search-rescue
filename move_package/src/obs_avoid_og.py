#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random

def range_idx(angle_min,angle_increment,angle):
        return round((angle - angle_min) / angle_increment)

class Avoider:
    def __init__(self, ns, delay):
        if ns == "none":
            self.ns = ""
        else:
            self.ns = ns

        rospy.init_node(self.ns + 'avoider', anonymous=False)
        rospy.sleep(delay)
        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)

        self.dir = random.choice([-1,1])
        self.blocked = False
        
        # Create a subscriber for the scan topic
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.scan_callback)

        # Register shutdown callback
        rospy.on_shutdown(self.shutdown_callback)

    def scan_callback(self, msg):
        fov = [range_idx(msg.angle_min,msg.angle_increment,-0.548),
               range_idx(msg.angle_min,msg.angle_increment,0.548)]

        sub_ranges = np.array(msg.ranges[fov[0]:fov[1]+1])
        min_range = np.min(sub_ranges[sub_ranges > 0.09])

        if min_range < 1.5:
            if self.blocked == False:
                self.blocked = True
                self.dir = random.choice([-1,1])
            # Create a Twist message to publish angular movement in the z-axis
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = 0.5 * self.dir

            # Publish the Twist message to the cmd_vel topic
            self.cmd_vel_pub.publish(cmd_vel_msg)
        else:
            self.blocked = False
            # Create a Twist message to publish linear movement in the x-axis
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.5

            # Publish the Twist message to the cmd_vel topic
            self.cmd_vel_pub.publish(cmd_vel_msg)
                    
    def run(self):
        rospy.spin()

    def shutdown_callback(self):
        # Stop the movement
        cmd_vel_msg = Twist()
        for i in range(0,2):
            self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':
    # Set a default namespace if not provided
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    
    # Wait for liftoff if launched
    delay = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    controller = Avoider(ns,delay)
    controller.run()
