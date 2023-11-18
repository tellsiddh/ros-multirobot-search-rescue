#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Lifter:
    def __init__(self, ns, z):
        if ns == "none":
            self.ns = ""
        else:
            self.ns = ns

        rospy.init_node(self.ns + 'lifter', anonymous=False)
        
        # Desired z position
        self.desired_z = z
        
        # Linear velocity in the z-axis
        self.linear_vel_z = 0.2
        
        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=10)
        
        # Create a subscriber for the odometry topic
        rospy.Subscriber(self.ns + '/ground_truth/state', Odometry, self.odometry_callback)

    def odometry_callback(self, msg):
        # Get the z position from the odometry message
        current_z = msg.pose.pose.position.z
        
        # Check if the drone's z position is not the desired value
        if abs(self.desired_z - current_z) >= 0.05:
            # Create a Twist message to publish linear movement in the z-axis
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.z = self.linear_vel_z
            
            # Publish the Twist message to the cmd_vel topic
            self.cmd_vel_pub.publish(cmd_vel_msg)
        else:
            # Stop the drone movement
            stop_cmd_vel_msg = Twist()
            for i in range(0,2):
                self.cmd_vel_pub.publish(stop_cmd_vel_msg)
            
            # Shutdown the script
            rospy.loginfo(self.ns + " reached desired z position.")
            rospy.signal_shutdown(self.ns + " reached desired z position.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Set a default namespace if not provided
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    
    # Set a default desired z position if not provided
    z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    controller = Lifter(ns,z)
    controller.run()
