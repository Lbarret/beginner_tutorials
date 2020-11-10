// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
#include "beginner_tutorials/chat_service.h"
//#include "beginner_tutorials/talker.h"


// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase1)
{
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<beginner_tutorials::chat_service>("conv");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
  beginner_tutorials::chat_service srv;
  srv.request.request_message = "y";
  client.call(srv);
  EXPECT_EQ("Duh", srv.response.response_message);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}