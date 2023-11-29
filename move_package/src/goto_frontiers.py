#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import sys

class Explorer:
    def __init__(self, ns):
        self.ns = ns
        rospy.init_node(self.ns + 'explorer', anonymous=False)

        # Create subscribers for the odometry, frontiers, and move_base status topics
        rospy.Subscriber(self.ns + '/ground_truth/state', Odometry, self.odometry_callback)
        rospy.Subscriber(self.ns + '/frontiers', Float32MultiArray, self.frontiers_callback)
        rospy.Subscriber(self.ns + '/move_base/status', GoalStatusArray, self.move_base_status_callback)

        # Create a publisher for the goal
        self.goal_publisher = rospy.Publisher(self.ns + '/move_base_simple/goal', PoseStamped, queue_size=10)

        # Initialize variables
        self.robot_pose = None
        self.frontiers = None
        self.current_goal = None
        self.blacklist = set()

    def odometry_callback(self, msg):
        self.robot_pose = msg.pose.pose
        if self.frontiers is not None and self.current_goal is None:
            self.navigate_to_frontier()

    def frontiers_callback(self, msg):
        # Reshape the frontiers array based on the given dim sizes
        dim_sizes = msg.layout.dim
        reshaped_frontiers = np.array(msg.data).reshape(dim_sizes[0].size, dim_sizes[1].size)

        self.frontiers = reshaped_frontiers

    def move_base_status_callback(self, msg):
        # Check if move_base is in recovery behavior
        if msg.status_list:
            if msg.status_list[0].status == 4:  # 4 corresponds to RECOVERY status
                if self.current_goal is not None:
                    self.blacklist.add(tuple(self.current_goal))
                    self.current_goal = None
                    rospy.logwarn("Stuck, selecting new goal.")
            if msg.status_list[0].status == 3:  # 3 corresponds to SUCCEEDED status
                if self.current_goal is not None:
                    self.blacklist.add(tuple(self.current_goal))
                    self.current_goal = None
                    rospy.loginfo("Goal reached, selecting new goal.")

    def navigate_to_frontier(self):

        # Extract the x, y positions from the frontiers array
        frontiers_positions = self.frontiers[:, :2]

        # Calculate the Euclidean distance from the robot to each frontier point
        robot_distances = np.linalg.norm(frontiers_positions - np.array([self.robot_pose.position.x, self.robot_pose.position.y]), axis=1)

        # Find the indices of frontiers that are not in the blacklist
        valid_frontier_indices = [i for i in range(len(robot_distances)) if tuple(frontiers_positions[i]) not in self.blacklist]
        
        if not valid_frontier_indices:
            # If all frontiers are blacklisted, select a random spot near the robot
            radius = 2.0  # Adjust the radius as needed
            theta = np.random.uniform(0, 2 * np.pi)
            random_x = self.robot_pose.position.x + radius * np.cos(theta)
            random_y = self.robot_pose.position.y + radius * np.sin(theta)

            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.orientation.w = 1.0
            goal_msg.pose.position.x = random_x
            goal_msg.pose.position.y = random_y
            
            self.goal_publisher.publish(goal_msg)
            self.current_goal = np.array([random_x,random_y])
            rospy.loginfo("All frontiers are blacklisted. Navigating to random nearby spot: {}".format(self.current_goal))

        else:

            # Calculate the Euclidean distance from the current goal to each frontier point
            # if self.current_goal is not None:
            #     goal_distances = np.linalg.norm(frontiers_positions - self.current_goal, axis=1)

            # Find the index of the closest valid frontier
            closest_frontier_index = valid_frontier_indices[np.argmin(robot_distances[valid_frontier_indices])]

            # Extract the x, y position of the closest frontier
            closest_frontier_position = frontiers_positions[closest_frontier_index]

            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.orientation.w = 1.0
            goal_msg.pose.position.x = closest_frontier_position[0]
            goal_msg.pose.position.y = closest_frontier_position[1]

            self.goal_publisher.publish(goal_msg)

            # Update the current_goal
            self.current_goal = closest_frontier_position
            rospy.loginfo("{} navigating to frontier: {}".format(self.ns, str(self.current_goal)))
        
        rospy.sleep(5)

    def distance_to_goal(self, goal):
        # Calculate the Euclidean distance from the robot to the goal
        return np.linalg.norm(np.array([self.robot_pose.position.x, self.robot_pose.position.y]) - goal)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    Explorer(ns).run()
