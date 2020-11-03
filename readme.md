# Overview:

This is a simple ros service example project 

## Assumptions/dependencies

 -Ubuntu 18.04

 -ROS Melodic

 -This project assumes the user has a created a catkin workspace by following the 
following tutorial: 

http://wiki.ros.org/catkin/Tutorials/create_a_workspace
 
## Build

To build this project, download this project into your catkin_ws/src file and run the following:

cd ~/catkin_ws

catkin_make

## Running

To run this project, run the launch file with a starting message:

roslaunch beginner_tutorials conversation.launch my_message:="<ebter your start message here>"

Answer the question by inputing a y or n into the terminal.


To call the service chatter in the command line, run the launch file using the above command. Before answering, open a new terminal and run the following:

rosservice call /chatter "request_message: '<enter your response here>'" 


