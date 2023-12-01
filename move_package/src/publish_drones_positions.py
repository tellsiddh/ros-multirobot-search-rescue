#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose

class DronesPositionsPublisher:
    def __init__(self):
        # Initialize node
        rospy.init_node('drones_positions_publisher', anonymous=True)

        # Initialize subscribers to the drones' odometry topics
        rospy.Subscriber('/uav1/odometry', Odometry, self.uav1_odometry_callback)
        rospy.Subscriber('/uav2/odometry', Odometry, self.uav2_odometry_callback)

        # Initialize publisher for drones_positions
        self.pub = rospy.Publisher('/drones_positions', PoseArray, queue_size=10)

        # Variables to hold drone positions
        self.uav1_pose = Pose()
        self.uav2_pose = Pose()

    def uav1_odometry_callback(self, msg):
        self.uav1_pose = msg.pose.pose

    def uav2_odometry_callback(self, msg):
        self.uav2_pose = msg.pose.pose

    def publish_drones_positions(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Create PoseArray message
            pose_array = PoseArray()

            # Add UAVs' poses to the PoseArray
            pose_array.poses.append(self.uav1_pose)
            pose_array.poses.append(self.uav2_pose)

            # Publish the PoseArray
            self.pub.publish(pose_array)

            rate.sleep()

if __name__ == '__main__':
    publisher = DronesPositionsPublisher()
    try:
        publisher.publish_drones_positions()
    except rospy.ROSInterruptException:
        pass
