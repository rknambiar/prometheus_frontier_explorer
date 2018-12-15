/*
 BSD 3-Clause License

 Copyright (c) 2018, Rohitkrishna Nambiar,  Harsh Kakashaniya
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file    FrontierExplorerTest.cpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief Test for FrontierExplorer class file
 *
 *  @section DESCRIPTION
 *
 *  Map class implementation
 */
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
