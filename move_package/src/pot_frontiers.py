#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseArray, Twist, PointStamped
from sensor_msgs.msg import LaserScan
import sys
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import time
import tf

class PotentialFieldController:
    def __init__(self, ns):
        self.ns = ns
        self.obstacle_range = 1
        self.robot_range = 2
        self.k_rep = 5
        self.robot_pose = None
        self.other_robots = None
        # self.obstacle_list = None
        self.obstacle_off = [0, 0]
        self.obstacle_angle = None

        self.frontiers = None
        self.k_att = 0
        self.blacklist = set()
        self.current_goal = None
        self.reach_thresh = 1

        # ROS publishers and subscribers
        self.vel_pub = rospy.Publisher(f'{ns}/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('drones_positions', PoseArray, self.robot_poses_callback)
        rospy.Subscriber(f'{ns}/scan', LaserScan, self.scan_callback)


        rospy.Subscriber(f'{ns}/ground_truth/state', Odometry, self.odometry_callback)
        rospy.Subscriber(f'{ns}/frontiers', Float32MultiArray, self.frontiers_callback)
        self.goal_publisher = rospy.Publisher(f'{ns}/pot/goal', PointStamped, queue_size=10)

        rospy.on_shutdown(self.shutdown_handler)

    def robot_poses_callback(self, msg):
        # Extract positions of other robots based on the provided namespace
        if self.ns == "":
            # If the namespace is "", include poses 2 and 3
            #self.robot_pose = msg.poses[0]
            self.other_robots = [msg.poses[1], msg.poses[2]]
        elif self.ns == "uav1":
            # If the namespace is "uav1", include poses 1 and 3
            #self.robot_pose = msg.poses[1]
            self.other_robots = [msg.poses[0], msg.poses[2]]
        elif self.ns == "uav2":
            # If the namespace is "uav2", include poses 1 and 2
            #self.robot_pose = msg.poses[2]
            self.other_robots = [msg.poses[0], msg.poses[1]]

    def scan_callback(self, msg):

        # if self.robot_pose is not None:
        #     self.obstacle_list = []
        #     for i, distance in enumerate(msg.ranges):
        #         if distance < self.obstacle_range and distance > 0.1:
        #             angle = i * msg.angle_increment + msg.angle_min
        #             x_off = distance * math.sin(angle)
        #             y_off = distance * math.cos(angle)
        #             obstacle_x = self.robot_pose.position.x + x_off
        #             obstacle_y = self.robot_pose.position.y + y_off

        #             self.obstacle_list.append([obstacle_x, obstacle_y])
        
        # Get the distance to the closest obstacle in front of the robot
        fixed_range = [value for value in msg.ranges if 0.1 < value < self.obstacle_range]
        
        if not fixed_range:
            self.obstacle_off = [0, 0]
        else:
            obstacle_distance = min(fixed_range)
            min_dist_index = msg.ranges.index(obstacle_distance)
            obstacle_angle = min_dist_index * msg.angle_increment + msg.angle_min - self.get_robot_yaw()
            #print(obstacle_angle*180/math.pi)
            x_off = obstacle_distance * math.sin(obstacle_angle)
            y_off = obstacle_distance * math.cos(obstacle_angle)
            self.obstacle_off = [x_off, y_off]

    def odometry_callback(self, msg):
        # Callback to update the robot's pose based on ground truth
        self.robot_pose = msg.pose.pose

    def frontiers_callback(self, msg):
        # Reshape the frontiers array based on the given dim sizes
        dim_sizes = msg.layout.dim
        reshaped_frontiers = np.array(msg.data).reshape(dim_sizes[0].size, dim_sizes[1].size)

        self.frontiers = reshaped_frontiers

    def repulsive_potential(self, q, q_i, p_0):
        q = np.array(q)
        q_i = np.array(q_i)
        p = np.linalg.norm(q - q_i)
        if p <= p_0 and p != 0:
            # angle = math.atan2(q[1] - q_i[1], q[0] - q_i[0])
            # grad_x = self.k_rep * (1/p - 1/p_0) * math.cos(angle)
            # grad_y = self.k_rep * (1/p - 1/p_0) * math.sin(angle)
            # return np.array([grad_x, grad_y])
            return self.k_rep * (1/p - 1/p_0) * (q - q_i) / (p**3)
        else:
            return np.zeros_like(q)

    def calculate_total_repulsive_force(self):
        total_force = np.array([0.0, 0.0])
        q = np.array([-self.robot_pose.position.y, self.robot_pose.position.x])

        # Repulsion from other robots
        for other_robot in self.other_robots:
            q_i = np.array([-other_robot.position.y, other_robot.position.x])
            force = self.repulsive_potential(q, q_i, self.robot_range)
            total_force += force

        # Repulsion from obstacles
        q_obs = q + np.array(self.obstacle_off)
        force = self.repulsive_potential(q, q_obs, self.obstacle_range)
        total_force += force

        # for i in range(len(temp_obslist)):
        #     q_i = np.array(temp_obslist[i])
        #     force = self.repulsive_potential(q, q_i, self.robot_range)
        #     total_force += force

        return total_force
    
    def set_goal(self):

        if self.frontiers is not None: 
            # Extract the x, y positions from the frontiers array
            frontiers_positions = self.frontiers[:, :2]

            # Calculate the Euclidean distance from the robot to each frontier point
            robot_distances = np.linalg.norm(frontiers_positions - np.array([self.robot_pose.position.x, self.robot_pose.position.y]), axis=1)

            # Find the indices of frontiers that are not in the blacklist
            valid_frontier_indices = [i for i in range(len(robot_distances)) if tuple(frontiers_positions[i]) not in self.blacklist]

            if valid_frontier_indices:
                # Find the index of the closest valid frontier
                closest_frontier_index = valid_frontier_indices[np.argmin(robot_distances[valid_frontier_indices])]

                # Extract the x, y position of the closest frontier
                closest_frontier_position = frontiers_positions[closest_frontier_index]

                goal_msg = PointStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = 'world'
                goal_msg.point.x = closest_frontier_position[0]
                goal_msg.point.y = closest_frontier_position[1]
                self.goal_publisher.publish(goal_msg)

                print(self.ns + ' navigating to ' + str(closest_frontier_position))

                self.current_goal = [-closest_frontier_position[1],closest_frontier_position[0]]

    def goal_status(self):
        
        eucl_dist = np.linalg.norm(self.current_goal - np.array([-self.robot_pose.position.y, self.robot_pose.position.x]))
        
        if eucl_dist <= self.reach_thresh:
            done_pos = [self.current_goal[1],-self.current_goal[0]]
            self.blacklist.add(tuple(done_pos))
            self.current_goal = None
            print(self.ns + ' reached current goal')

    def calculate_attractive_force(self):

        dist = self.current_goal - np.array([-self.robot_pose.position.y, self.robot_pose.position.x])

        force = self.k_att * dist * np.linalg.norm(dist)**2

        return force

    def get_robot_yaw(self):
        # Get the robot's yaw angle from the orientation quaternion
        quaternion = [self.robot_pose.orientation.x,
                    self.robot_pose.orientation.y,
                    self.robot_pose.orientation.z,
                    self.robot_pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        print(self.ns + ' euler[2]: ' + str(euler[2]))
        return euler[2]  # Yaw angle is the third element in the Euler angles

    def rotate_vector(self, vector, angle):
        # Rotate a 2D vector by a given angle (in radians)
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        rotated_vector = np.dot(vector, rotation_matrix)
        return rotated_vector

    def shutdown_handler(self):
        # Stop the robot when the script is shutdown
        twist = Twist()
        for i in range(0,2):
            self.vel_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            if self.current_goal is None:
                # twist = Twist()
                # twist.angular.z = 1.0  # You can adjust the angular velocity as needed
                # for i in range(0,3):
                #     self.vel_pub.publish(twist)

                # # Wait for a short duration to allow the robot to rotate
                # time.sleep(2.0)  # You can adjust the duration as needed

                # # Check orientation.z to determine when a full 360 degrees rotation is completed
                # while abs(self.robot_pose.orientation.z) > 0.1:  # Adjust the threshold as needed
                #     print(self.robot_pose.orientation.z)

                # twist.angular.z = 0.0  # Stop the rotation
                # for i in range(0,3):
                #     self.vel_pub.publish(twist)
                self.set_goal()

            if self.current_goal is not None:
                total_repulsive_force = self.calculate_total_repulsive_force()
                attractive_force = self.calculate_attractive_force()
                total_force = total_repulsive_force + attractive_force
                if np.linalg.norm(total_force) != 0:
                    total_force /= np.linalg.norm(total_force)
                    total_force /= 2
                twist = Twist()
                new_force = self.rotate_vector([total_force[1],-total_force[0]],self.get_robot_yaw())
                print(self.ns + 'new_force: ' + str(new_force))
                print(self.ns + 'old_force: ' + str([total_force[1],-total_force[0]]))
                # twist.linear.x = total_force[1] 
                # twist.linear.y = -total_force[0]
                twist.linear.x = new_force[0]
                twist.linear.y = new_force[1]
                self.vel_pub.publish(twist)
                #print(twist)

                self.goal_status()

            rate = rospy.Rate(10)  # 10 Hz
            rate.sleep()

if __name__ == "__main__":

    ns = sys.argv[1] if len(sys.argv) >= 2 else ""
    if ns == 'none':
        ns = ""

    rospy.init_node(ns + 'potential_field_controller')

    controller = PotentialFieldController(ns)
    controller.run()
