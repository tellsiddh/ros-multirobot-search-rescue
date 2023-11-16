# ros-multirobot-search-rescue

MultiRobot Simulation using Gazebo for Search and Rescue operations in earthquake affected areas.

## Drone Mapping

### Dependencies
 Clone my fork of hector-quadrotor-noetic into your catkin workspace, as instructed here: https://github.com/anaammostafiz/hector-quadrotor-noetic

Install the rtabmap_ros package.
```sh
sudo apt install ros-noetic-rtabmap-ros
```
Install the multirobot_map_merge package.
```sh
sudo apt install ros-noetic-multirobot-map-merge
```
### Installation
Clone this repository into your catkin workspace.
```sh
git clone https://github.com/tellsiddh/ros-multirobot-search-rescue.git
```
Compile your catkin workspace.
```sh
catkin_make
```
### How to Use (Two Drones)
1. Launch the Gazebo maze world with two drones.
```sh
roslaunch rtab_package maze2drones.launch
```
2. Launch the rtabmap node for each drone. Make sure each database_path is a distinct .db file.
```sh
roslaunch rtab_package mapping.launch ns:=uav1 database_path:=rtabmap_1.db
```
```sh
roslaunch rtab_package mapping.launch ns:=uav2 database_path:=rtabmap_2.db
```
3. Run the map merger and visualize it in rviz. The rviz command takes the relative path to the map_merge.rviz file.  
```sh
roslaunch rtab_package map_merge2.launch
```
```sh
rosrun rviz rviz -d src/ros-multirobot-search-rescue/rtab_package/rviz/map_merge.rviz
```
4. Run the teleop_twist_keyboard to move drone 1. To move drone 2, replace uav1 with uav2.
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=uav1/cmd_vel
```

### How to Use (One Drone)
1. Launch the Gazebo maze world. For the earthquake world, use earthquake.launch instead.
```sh
roslaunch rtab_package maze.launch
```
2. Launch one drone.
```sh
roslaunch hector_quadrotor_gazebo spawn_quadrotor_with_asus_with_laser.launch
```
3. Launch the rtabmap node, specifying the database_path like above if needed. Then, visualize it in rviz. 
```sh
roslaunch rtab_package mapping.launch
```
```sh
roslaunch rtabmap_demos demo_turtlebot_rviz.launch
```
4. Run the teleop_twist_keyboard to move the drone.
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Post-processing
The .db files containing 2D, 3D, and image data are stored in your ~/.ros folder. To view, merge, and export the results, follow this tutorial: https://github.com/introlab/rtabmap/wiki/Multi-Session-Mapping-with-RTAB-Map-Tango.

Note: If you are going to reuse database_path's on consecutive mapping sessions, make sure to move or delete the old .db files.