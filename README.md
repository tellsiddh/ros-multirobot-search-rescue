# ros-multirobot-search-rescue

MultiRobot Simulation using Python and Gazebo for Search and Rescue operations.

## Python only
See algorithms folder.

## Gazebo

### Dependencies
 Clone my fork of hector-quadrotor-noetic into your catkin workspace, as instructed here: https://github.com/anaammostafiz/hector-quadrotor-noetic

Install the rtabmap_ros package.
```sh
sudo apt install ros-noetic-rtabmap-ros
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
### How to Use
1. Launch the Gazebo maze world or sphere world with three drones.
```sh
roslaunch rtab_package maze3drones.launch
```
```sh
roslaunch rtab_package sphere3drones.launch
```
2. Launch the mapping nodes.
```sh
roslaunch rtab_package mapping3drones.launch
```
3. Launch the frontier exploration. You can optionally launch the potential field spreader first, and then kill it when the drones reach equilibrium.

```sh
roslaunch move_package pot_spread3.launch
```
```sh
roslaunch move_package swarm_frontier.launch
```

4. Visualize the 2D exploration in rviz. You may need to move the rviz view to find the drones.
```sh
roslaunch move_package swarm_rviz.launch
```

### Post-processing
The .db files containing 2D, 3D, and image data are stored in your ~/.ros folder. To view, merge, and export the results, follow this tutorial: https://github.com/introlab/rtabmap/wiki/Multi-Session-Mapping-with-RTAB-Map-Tango.

Note: On consecutive mapping sessions, make sure to move or delete the old .db files.