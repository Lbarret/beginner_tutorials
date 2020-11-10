# Overview:

This is a simple ros example project that shows the functionality of services, subscriber/publishers, and tf frames

## Assumptions/dependencies

 -Ubuntu 18.04

 -ROS Melodic

 -This project assumes the user has a created a catkin workspace by following the 
following tutorial: 

http://wiki.ros.org/catkin/Tutorials/create_a_workspace
 
## Build

To build this project, download this project into your catkin_ws/src file and run the following:

- cd ~/catkin_ws

- catkin_make

## Running

To run this project, run the launch file with a starting message:

- roslaunch beginner_tutorials conversation.launch my_message:="enter your start message here" 

Optional: add the argument record:=true to record all the data for 15 seconds. The default will be set to false.

- roslaunch beginner_tutorials conversation.launch my_message:="hello" record:=true

Answer the question by inputing a y or n into the terminal.

## Inspecting Rosbag

To inspect rosbag, locate where the rosbag is saved. These usually are saved in the home/.ros/ directory. Run the following:

- rosbag info ROSBAG FILE

To make sure the ROSBAG file plays correctly, run the listener node and play the rosbag file:

- roscore

New Terminal:

- rosrun beginner_tutorials listener

New Terminal:
- rosbag play ROSBAG FILE

The rosbag file should be publishing to the chatter topic which the listener is subscribing to.

## Inspecting TF frames

To inspect TF frames, run the launch file above. In a separate terminal, run the following:

- rosrun rqt_tf_tree rqt_tf_tree

You can also inspect tf frames using tf_echo by running the following:

- rosrun tf tf_echo world talk

## Running rostests

To run the test, you first have to build the tests.

- cd ~/catkin_ws

- catkin_make tests

To run the test, run the following:

- rostest beginner_tutorials conversation.launch my_message:="hello"

It will ask you to answer the question and then the test will run after that.
 
