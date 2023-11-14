# ros-multirobot-search-rescue

MultiRobot Simulation using Gazebo for Search and Rescue operations in earthquake affected areas.

## Drone Mapping

### Dependencies
 Clone my fork of hector_quadrotor_noetic into your catkin workspace, as instructed here: https://github.com/anaammostafiz/hector-quadrotor-noetic

Install the rtabmap_ros package.
```sh
sudo apt install ros-noetic-rtabmap-ros
```
### Installation
Clone this repository into your catkin workspace.
```sh
git clone https://github.com/tellsiddh/ros-multirobot-search-rescue.git
```
### How to Use (Two Drones)
0. Compile your catkin workspace.
```sh
catkin_make
```
1. Launch the Gazebo maze world with two drones.
```sh
roslaunch rtab_package maze2drones.launch
```
2. Run the teleop_twist_keyboard to move drone 1. To move drone 2, replace uav1 with uav2.
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=uav1/cmd_vel
```
4. Launch the rtabmap node for each drone in a separate terminal. The database_path names can be whatever .db, as long as they are distinct.
```sh
roslaunch rtab_package mapping.launch ns:=uav1 database_path:=rtabmap_1.db
```
```sh
roslaunch rtab_package mapping.launch ns:=uav2 database_path:=rtabmap_2.db
```
5. After moving the drones to create the maps, shutdown the rtabmap nodes. Then, try to merge the maps.
```sh
cd ~/.ros
```
```sh
rtabmap-reprocess "rtabmap_1.db;rtabmap_2.db" merged.db 
```
6. View the results. In the GUI, you can also export the 2D and 3D maps.
```sh
rtabmap-databaseViewer merged.db 
```
Note: If you are going to reuse database_paths on consecutive swarm mapping sessions, make sure to delete the old .db files from your ~/.ros folder.

### How to Use (One Drone)
0. Compile your catkin workspace.
```sh
catkin_make
```
1. Launch the Gazebo maze world. For the earthquake world, use earthquake.launch instead.
```sh
roslaunch rtab_package maze.launch
```
2. Launch one drone.
```sh
roslaunch hector_quadrotor_gazebo spawn_quadrotor_with_asus_with_laser.launch
```
3. Run the teleop_twist_keyboard to move the drone.
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
4. Launch the rtabmap node.
```sh
roslaunch rtab_package mapping.launch
```
5. Visualize the map in RVIZ as you move the drone.
```sh
roslaunch rtabmap_demos demo_turtlebot_rviz.launch
```