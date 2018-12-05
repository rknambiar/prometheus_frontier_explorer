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
 *  @file    main.cpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief main file for running prometheus_frontier_exploration package
 *
 *  @section DESCRIPTION
 *
 *  Main file for prometheus_frontier_exploration package. This calls
 *  the FrontierExplorer class method to explore the environment.
 */

#include "FrontierExplorer.hpp"

FrontierExplorer::FrontierExplorer() {
  // Initialize publisher topic
  velPub = nh.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 100);

  // Sunscriber to map message
  mapSub = nh.subscribe("/map", 100, &FrontierExplorer::processOccupancyGrid,
                        this);

  // Set start velocity message to zero
  velMsg.linear.x = 0.0;
  velMsg.linear.y = 0.0;
  velMsg.linear.z = 0.0;
  velMsg.angular.x = 0.0;
  velMsg.angular.y = 0.0;
  velMsg.angular.z = 0.0;

  // Publish the velocity
  velPub.publish(velMsg);

  // Visualization markers for frontier and centroids
  frontierMarkerPub = nh.advertise < visualization_msgs::MarkerArray
      > ("/frontier_marker_array", 1);

  allFrontierPub = nh.advertise < visualization_msgs::MarkerArray
      > ("/all_frontier_marker_array", 1);

  frontierClusterPub = nh.advertise < visualization_msgs::MarkerArray
      > ("/frontier_clustor_array", 1);

  ROS_INFO("New frontier exploration turtle bot created.");
}

FrontierExplorer::~FrontierExplorer() {
  // Set velocity to zero on exit
  velMsg.linear.x = 0.0;
  velMsg.linear.y = 0.0;
  velMsg.linear.z = 0.0;
  velMsg.angular.x = 0.0;
  velMsg.angular.y = 0.0;
  velMsg.angular.z = 0.0;

  // Publish the velocity
  velPub.publish(velMsg);
}

void FrontierExplorer::rotate360() {
  velMsg.linear.x = 0.0;
  velMsg.linear.y = 0.0;
  velMsg.linear.z = 0.0;
  velMsg.angular.x = 0.0;
  velMsg.angular.y = 0.0;
  velMsg.angular.z = 0.8;
  ros::Time begin = ros::Time::now();
  ros::Duration waitTime = ros::Duration(7.5);
  ros::Time end = begin + waitTime;
  std::cout << "[ Start time:]" << begin << std::endl;
  while (ros::Time::now() < end && ros::ok()) {
    // Publish the velocity
    velPub.publish(velMsg);
  }
  std::cout << "[ End time:]" << ros::Time::now() << std::endl;
}

void FrontierExplorer::processOccupancyGrid(const nav_msgs::OccupancyGrid
                                            ::ConstPtr& gridMsg) {
  // TODO(harshkakashaniya) process occupancy grid
}

int FrontierExplorer::getFrontiers() {
  // TODO(harshkakashaniya) get frontier with gmapping library
  return 1;
}

void FrontierExplorer::getClusters() {
  // TODO(harshkakashaniya) get centroid with the help of clusters.
}


void FrontierExplorer::visualizeClusterCenters(std::vector<std::pair<double,
                                                double>> centers) {
    // TODO(harshkakashaniya) use markers for cluster center.
}

void FrontierExplorer::visualizeClusterFrontiers() {
    // TODO(harshkakashaniya) visualize cluster frontiers.
}
void FrontierExplorer::publishFrontierPoints(int count) {
  // TODO(harshkakashaniya) publish frontier points
}


void FrontierExplorer::explore() {
  // TODO(harshkakashaniya) explore around for different frontier.
    }
