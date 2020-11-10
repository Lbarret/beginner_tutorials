/**
 * @file talker.cpp
 * @version 3.0
 * @brief Ros broadcaster node that broadcasts a tf frame 
 * @Created on: Nov 8, 2020
 * @copyright 2020 
 * @Author Loic Barret
 */

#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/chat_service.h"

/**
 * This program demonstrates the functionality of publisher/subscribers, services, and tf frames over the ROS system.
 */

/**
* @brief Reads in the request variable and writes to the response variable accordingly
* @param &req the request variable from chat_service
* @param &res the response variable from chat_service
* @return bool
*/
bool chat(beginner_tutorials::chat_service::Request &req,
  beginner_tutorials::chat_service::Response &res) {
  ROS_INFO_STREAM("request: " << req.request_message);
  if (req.request_message == "y") {
    res.response_message = "Duh";
  } else if (req.request_message == "n") {
    res.response_message = "WRONG";
  } else {
    res.response_message = "warning";
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

  /**
   * Creates a tf broadcaster object
   * 
   * 
   */ 
  tf::TransformBroadcaster br;
  tf::Transform transform;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */  
  int count = 0;
  while (ros::ok()) {
    /**
     * ServiceServer creates a server of the service chat_service
     */
    ros::ServiceServer service = nh.advertiseService("conv", chat);

    std_msgs::String msg;

    /**
     * ServiceServer creates a server of the service chat_service
     */
    std::stringstream ss;
    ss << "robots > humans " << count;
    msg.data = ss.str();

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    /**
     * loop_rate.sleep() ensures the loop rate is achieved
     */
    loop_rate.sleep();
    ++count;

    /**
     * set the origin and roation of the transform matrix. Apply that transform
     * to a child frame "talk" with "world" parent frame 
     */    
    transform.setOrigin(tf::Vector3(0.0, 2.0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
     "world", "talk"));

    /**
     * While the service hasn't been called, inform the user that the node is waiting
     * for a response.
     */
    ROS_INFO_STREAM("Waiting for response");

    ros::spinOnce();
  }

  return 0;
}
