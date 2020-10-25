# Overview:

This is a simple publisher/subscriber project 

## Assumptions/dependencies

 -Ubuntu 18.04

 -ROS Melodic

 -This project assumes the user has a created a catkin workspace by following the 
following tutorial: 

http://wiki.ros.org/catkin/Tutorials/create_a_workspace
 
## Build

To build this project, download this project into your catkin_ws/src file and run the following:

cd ~catkin_ws

catkin_make

## Running

To run this project, start the roscore by running the following command:

roscore

To start the talker node, open a new window and run the following:

rosrun beginner_tutorials/talker

To start the listener node, open a new window and run the following:

rosrun beginner_tutorials/listener
