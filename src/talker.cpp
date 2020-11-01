/**
 * @file talker.cpp
 * @version 1.0
 * @brief Ros publisher node that publishes to the chatter topic
 * @Created on: Sep 28, 2020
 * @copyright 2020 
 * @Author Loic Barret
 */
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/chat_service.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

bool chat(beginner_tutorials::chat_service::Request &req, beginner_tutorials::chat_service::Response &res) {
	ROS_INFO_STREAM("request: " << req.request_message);
	if(req.request_message == "y") {
		res.response_message = "Duh";
	} else {
		res.response_message = "WRONG";
	}
	ROS_INFO_STREAM("sending back response: " << res.response_message);
	return true;
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;


  ros::ServiceServer service = nh.advertiseService("chatter", chat);
  ROS_INFO("Waiting for response");
  ros::spin();


  return 0;
}
