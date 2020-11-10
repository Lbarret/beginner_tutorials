/**
 * @file listener.cpp
 * @version 3.0
 * @brief Ros client node that requests the service chat_service 
 * and subscribes to the chatter topic
 * @Created on: Nov 8, 2020
 * @copyright 2020 
 * @Author Loic Barret
 */
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/chat_service.h"

/**
 * This program demonstrates the functionality of services and subscribers over the ROS system.
 */

/**
* @brief reads what is coming in the chatter topic
* @param &msg the message being transcribed on the chatter topic
* @return void
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard: " << msg->data.c_str());
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
  std::cout << "\n" << message;
  std::cout << "?\n\n...Really? That's all you have to say? \n\nWhatever. \n\n";
  while (ros::ok()) {
    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    /** 
     * ServiceClient creates a client of the service chat_service
     */
    ros::ServiceClient client =
    nh.serviceClient<beginner_tutorials::chat_service>("conv");
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
    }

    /**
     * If the user responds n to the question, the program gives off a fatal error 
     */
    if (srv.request.request_message == "n") {
      ROS_FATAL_STREAM("User unable to realize the superiority of robots");
    }
    ros::spinOnce();
  }



  return 0;
}
