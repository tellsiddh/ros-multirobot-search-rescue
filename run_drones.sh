#!/bin/bash

gnome-terminal -- bash -c "roslaunch rtab_package maze.launch; exec bash"
sleep 5
gnome-terminal -- bash -c "roslaunch hector_quadrotor_gazebo spawn_quadrotor_with_asus_with_laser.launch; rosrun move_package liftoff.py; exec bash"
sleep 5
gnome-terminal -- bash -c "roslaunch rtab_package mapping.launch; exec bash"
sleep 5
gnome-terminal -- bash -c "roslaunch move_package frontier_rviz.launch; exec bash"
sleep 5
gnome-terminal -- bash -c "rosrun move_package find_frontiers.py; exec bash"
sleep 5
gnome-terminal -- bash -c "rosrun move_package goto_frontiers.py; exec bash"
sleep 5
gnome-terminal -- bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"
