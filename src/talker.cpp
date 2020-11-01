/**
 * @file talker.cpp
 * @version 2.0
 * @brief Ros server node that responds to the sevice chat_service 
 * @Created on: Oct 31, 2020
 * @copyright 2020 
 * @Author Loic Barret
 */
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/chat_service.h"

/**
 * This program demonstrates the functionality of services over the ROS system.
 */

/**
* @brief Reads in the request variable and writes to the response variable accordingly
* @param &req the request variable from chat_service
* @param &res the response variable from chat_service
* @return bool
*/
bool chat(beginner_tutorials::chat_service::Request &req,
  beginner_tutorials::chat_service::Response &res) {
  ROS_DEBUG_STREAM("request: " << req.request_message);
  if (req.request_message == "y") {
    res.response_message = "Duh";
  } else if (req.request_message == "n") {
    res.response_message = "WRONG";
  } else {
    res.response_message = "warning";
  }
  ROS_DEBUG_STREAM("sending back response: " << res.response_message);
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

  /**
   * ServiceServer creates a server of the service chat_service
   */
  ros::ServiceServer service = nh.advertiseService("chatter", chat);

  /**
   * While the service hasn't been called, inform the user that the node is waiting
   * for a response.
   */
  ROS_INFO("Waiting for response");

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();


  return 0;
}
