#include <gtest/gtest.h>
#include "FrontierExplorer.hpp"
#include "ros/ros.h"

class TestSubPub {
 public:
  void testMarkerPublish(const visualization_msgs::MarkerArray::ConstPtr& msg) {
  }
  void testVelocityPublish(const geometry_msgs::Twist::ConstPtr& msg) {
  }
};

class FrontierExplorerTest : public ::testing::Test {
 public:
  ros::NodeHandle nh;
  FrontierExplorer prometheus;
  TestSubPub test;
  /**
   *  @brief Setup Function used during tests
   *
   *  @param none
   *
   *  @return void
   */
  void SetUp() {
  }

  /**
   *  @brief Teardown function used during tests
   *
   *  @param none
   *
   *  @return void
   */
  void TearDown() {
  }
};

TEST_F(FrontierExplorerTest, testAllMarkerPublish) {
  ros::Rate loopRate(10);
  ros::Subscriber sub = nh.subscribe("/all_frontier_marker_array", 1000,
                                     &TestSubPub::testMarkerPublish, &test);
  loopRate.sleep();
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(FrontierExplorerTest, testFrontierSegmentedPublish) {
  ros::Rate loopRate(10);
  ros::Subscriber sub = nh.subscribe("/frontier_marker_array", 1000,
                                     &TestSubPub::testMarkerPublish, &test);
  loopRate.sleep();
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(FrontierExplorerTest, testFrontierClusteredPublish) {
  ros::Rate loopRate(10);
  ros::Subscriber sub = nh.subscribe("/frontier_clustor_array", 1000,
                                     &TestSubPub::testMarkerPublish, &test);
  loopRate.sleep();
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(FrontierExplorerTest, testReachAvoidPublish) {
  ros::Rate loopRate(10);
  ros::Subscriber sub = nh.subscribe("/reach_avoid_region", 1000,
                                     &TestSubPub::testMarkerPublish, &test);
  loopRate.sleep();
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(FrontierExplorerTest, testVelPublish) {
  ros::Rate loopRate(10);
  ros::Subscriber sub = nh.subscribe("/mobile_base/commands/velocity", 1000,
                                     &TestSubPub::testVelocityPublish, &test);
  loopRate.sleep();
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(FrontierExplorerTest, testGridSubscriber) {
  ros::Rate loopRate(10);
  ros::Publisher gridPub = nh.advertise < nav_msgs::OccupancyGrid
      > ("/map", 10);
  loopRate.sleep();
  EXPECT_EQ(1, gridPub.getNumSubscribers());
}
