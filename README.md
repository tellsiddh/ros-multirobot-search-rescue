# ros-multirobot-search-rescue

MultiRobot Simulation using Gazebo for Search and Rescue operations in earthquake affected areas.

## Drone Mapping

### Dependencies
 Clone hector_quadrotor_noetic into your catkin workspace, as instructed here: https://github.com/RAFALAMAO/hector-quadrotor-noetic

Clone quadrotor_navigation into your catkin workspace.
```sh
git clone https://bitbucket.org/theconstructcore/quadrotor_navigation/src/master/
```
Install the rtabmap_ros package.
```sh
sudo apt install ros-noetic-rtabmap-ros
```
### Installation
Clone this repository into your catkin workspace.
```sh
git clone https://github.com/tellsiddh/ros-multirobot-search-rescue.git
```
### How to Use
0. Compile your catkin workspace.
```sh
catkin_make
```
1. Launch a world with gazebo_ros. For example:
```sh
roslaunch hector_gazebo_worlds small_indoor_scenario.launch
```
2. Launch a quadrotor with an RGBD camera, 2D laser, and odometry.
```sh
roslaunch hector_quadrotor_gazebo spawn_quadrotor_with_asus_with_laser.launch
```
3. Run the teleop_twist_keyboard to move the drone. **Note**: Before pressing 't' to lift the drone, make sure there is sufficient space in front of it. During initial lift, the drone will first move forward, then float back to its original location.
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
4. Launch the rtab mapping node.
```sh
roslaunch rtab_package mapping.launch
```
5. Visualize the map in RVIZ as you move the drone.
```sh
roslaunch rtabmap_demos demo_turtlebot_rviz.launch
```
You can see how a swarm could be launched with spawn_two_quadrotors.launch, but swarm mapping is not implemented yet.
