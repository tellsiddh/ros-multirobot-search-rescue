#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sys
import math

class ScanFilter:
    def __init__(self, ns, fov):
        self.ns = ns
        self.fov = fov

        # Define the scan and filtered_scan topics
        scan_topic = f'{ns}/scan'
        filtered_scan_topic = f'{ns}/filtered_scan'

        # Initialize ROS node
        rospy.init_node(f'{ns}scan_filter_node', anonymous=True)

        # Subscribe to the scan topic
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

        # Publisher for the filtered scan
        self.filtered_scan_pub = rospy.Publisher(filtered_scan_topic, LaserScan, queue_size=10)

        print(f'filtering {ns}/scan')

    def scan_callback(self, scan_msg):
        # Calculate the central index (corresponding to angle 0)
        central_index = len(scan_msg.ranges) // 2

        left_index = central_index - int((self.fov/2) / (scan_msg.angle_increment * (180.0 / math.pi)))

        right_index = central_index + int((self.fov/2) / (scan_msg.angle_increment * (180.0 / math.pi)))

        # Extract the ranges corresponding to the desired angle range
        filtered_ranges = scan_msg.ranges[left_index:right_index+1]

        # Create a new LaserScan message with the filtered ranges
        filtered_scan_msg = LaserScan()
        filtered_scan_msg.header = scan_msg.header
        filtered_scan_msg.angle_min = -(self.fov/2) * (math.pi / 180.0)
        filtered_scan_msg.angle_max = (self.fov/2) * (math.pi / 180.0)
        filtered_scan_msg.angle_increment = scan_msg.angle_increment
        filtered_scan_msg.time_increment = scan_msg.time_increment
        filtered_scan_msg.scan_time = scan_msg.scan_time
        filtered_scan_msg.range_min = scan_msg.range_min
        filtered_scan_msg.range_max = scan_msg.range_max
        filtered_scan_msg.ranges = filtered_ranges
        filtered_scan_msg.intensities = scan_msg.intensities[left_index:right_index+1]

        # Publish the filtered scan
        self.filtered_scan_pub.publish(filtered_scan_msg)

if __name__ == '__main__':
    # Provide the robot namespace as a command-line argument

    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    if ns == 'none':
        ns = ""

    fov = int(sys.argv[2]) if len(sys.argv) >= 3 else 60

    try:
        scan_filter = ScanFilter(ns, fov)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
