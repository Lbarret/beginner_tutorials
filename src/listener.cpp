/**
 * @file listener.cpp
 * @version 2.0
 * @brief Ros client node that requests the service chat_service 
 * @Created on: Oct 31, 2020
 * @copyright 2020 
 * @Author Loic Barret
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/chat_service.h"
#include <iostream>

/**
 * This program demonstrates the functionality of services over the ROS system.
 */


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
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;


  std::string message;

  /**
   * getParam gets the parameter fed in by the user while launching the .launch file
   */
  nh.getParam("message", message);
  
  std::cout <<"\n" << message << "?\n\n...Really? That's all you have to say? \n\nWhatever. \n\n";

  
  while (ros::ok()){
    
    /** 
     * ServiceClient creates a client of the service chat_service
     */
    ros::ServiceClient client = nh.serviceClient<beginner_tutorials::chat_service>("chatter");
    
    /**
     * srv is a service object that contains the attributes in chat_service.srv. The user 
     * populates the request message with their answer to the question
     */
    beginner_tutorials::chat_service srv;
    std::cout << "Do you think robots awesome? (y/n): ";
    std::cin >> srv.request.request_message;

    /**
     * Checks to see if the client can be called. If it can, feed the srv object to the service.
     * If the client can't be called, return an error telling the user it couldn't call the service.
     * Then the program checks the response. If it recieved any request other than a y or n, it outputs
     * a warning telling the user it did not recieve a valid request. If it recieved a valid request, 
     * it outputs the corresponding response.
     */
    if (client.call(srv)) {
      if (srv.response.response_message == "warning") {
        ROS_WARN_STREAM("Did not recieve a y or n request");
      } else {
        ROS_INFO_STREAM(srv.response.response_message);
      }
    } else {
      ROS_ERROR_STREAM("Failed to call service chat_service");
      return 1;
    }

    /**
     * If the user responds n to the question, the program gives off a fatal error and shuts ros down.
     */
    if(srv.request.request_message == "n") {
      ROS_FATAL_STREAM("User was too dumb to realize the superiority of robots");
      ros::shutdown();
    }
  }

  return 0;
}
