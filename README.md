
# ros-multirobot-search-rescue

MultiRobot Simulation using Gazebo for Search and Rescue operations in earthquake affected areas.

  

## Drone Mapping

### Dependencies
 Clone my fork of hector_quadrotor_noetic into your catkin workspace.
```sh
git clone https://github.com/anaammostafiz/hector-quadrotor-noetic
```
Install the rtabmap_ros package.
```sh
sudo apt install ros-noetic-rtabmap-ros
```
### Installation
1. Clone this repository into your catkin workspace.
```sh
git clone https://github.com/tellsiddh/ros-multirobot-search-rescue.git
```
2. Switch to this drone-mapping branch.
```sh
git checkout -b drone-mapping
```

### How to Use
0. Compile your catkin workspace.
```sh
catkin_make
```
1. Launch a world with gazebo_ros. For example:
```sh
roslaunch gazebo_ros <world>.launch
```
2. Launch a quadrotor with Kinect.
```sh
roslaunch hector_quadrotor_gazebo spawn_quadrotor_with_kinect.launch
```
3. Run the teleop_twist_keyboard to move the drone.
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
4. Launch the Kinect+Odometry mapping node.
```sh
roslaunch rtab_package mapping.launch
```
5. Visualize the map in RVIZ as you move the drone.
```sh
roslaunch rtabmap_demos demo_turtlebot_rviz.launch
```
You can see how a swarm could be launched with spawn_two_quadrotors.launch, but swarm mapping is not supported yet.

#### Development Notes
RVIZ map looks noisier than expected, since it includes the point clouds for empty space. Need to see if this is actually a problem. 
