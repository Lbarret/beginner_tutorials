/**
 * @file utest.cpp
 * @version 1.0
 * @brief Rostest node to test talker node 
 * @Created on: Nov 8, 2020
 * @copyright 2020 
 * @Author Loic Barret
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/chat_service.h"

/**
 * Declare a test
 */
TEST(TestSuite, testCase1) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<beginner_tutorials::chat_service>("conv");

  /**
   * test whether the client exists
   */
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
  beginner_tutorials::chat_service srv;
  srv.request.request_message = "y";
  client.call(srv);
  /**
   * test whether the correct response is given
   */
  EXPECT_EQ("Duh", srv.response.response_message);
}

/**
 * Run all the tests that were declared with TEST()
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
