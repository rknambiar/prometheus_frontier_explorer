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
  std::cout << "\n\n";
  ROS_INFO("Occupancy message post-process called..");
  int currwidth = gridMsg->info.width;  // x
  int currheight = gridMsg->info.height;  // y
  double currreso = gridMsg->info.resolution;
  geometry_msgs::Point currcenter = gridMsg->info.origin.position;
  std::cout << "[MAP INFO] Width: " << currwidth << ", Height: " << currheight
            << ", Resolution: " << currreso;
  std::cout << ", Origin: " << gridMsg->info.origin.position.x << ","
      << gridMsg->info.origin.position.y << std::endl;

  slamMap.updateMap(currwidth, currheight, currreso, currcenter, gridMsg);
}

int FrontierExplorer::getFrontiers() {
  int width = slamMap.getmapWidth();
  int height = slamMap.getmapHeight();
  int frontierCount = 0;
  //  ROS_INFO("Getting frontiers..");
  ROS_INFO("Getting frontiers for map with w:%d, h:%d", width, height);
  std::vector<std::vector<MapNode>>& map = slamMap.getMap();
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      if (map[i][j].getProbability() == 0) {

        bool frontierFlag = false;
        // Check if neighbor is unexplored node
        for (int k = i - 1; k <= i + 1; k++) {
          for (int l = j - 1; l <= j + 1; l++) {
            if (k >= 0 && l >= 0 && k <= height && l <= width
                && map[k][l].getProbability() == -1) {
              // Set flag to true
              frontierFlag = true;
            }
          }
        }
        // Update in the map
        map[i][j].setisFrontier(frontierFlag);
        if (frontierFlag) {
          frontierCount++;
        }
      }
      // We also set its frontier id to -1
      map[i][j].setFrontierIndex(-1);
    }
  }

  ROS_INFO("Updated frontier flag and index with frontier count:%d",
           frontierCount);
  return frontierCount;
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
  //  ROS_INFO("Publishing frontier markers... ");
  int width = slamMap.getmapWidth();
  int height = slamMap.getmapHeight();

  std::vector<std::vector<MapNode>>& map = slamMap.getMap();

  visualization_msgs::MarkerArray markerArray;

  int markerCount = 0;
  int markerLimit = count;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      if (map[i][j].getisFrontier()) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "Frontier";
        marker.id = markerCount;
        marker.type = visualization_msgs::Marker::SPHERE;

        marker.action = visualization_msgs::Marker::ADD;

        // Define the scale (meter scale)
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;

        // Set the color
        marker.color.r = 0.0f;
        marker.color.g = 0.41f;
        marker.color.b = 0.70f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker.pose.position.x = map[i][j].getX();
        marker.pose.position.y = map[i][j].getY();
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;

        markerCount++;
        if (markerCount < markerLimit) {
          markerArray.markers.push_back(marker);
          //          std::cout << map[i][j].getX() << "," << map[i][j].getY() << std::endl;
        }
      }
    }
  }

  allFrontierPub.publish(markerArray);
  ROS_INFO("Total %d markers published.", markerCount);
}


void FrontierExplorer::explore() {
  // Set loop frequency
  ros::Rate loop_rate(1);
  bool shouldRotate = true;
  loop_rate.sleep();
  ros::spinOnce();
  while (ros::ok()) {
    std::cout << "\n\n\n";
    ROS_INFO_STREAM("#################################");

    if (shouldRotate) {
      //rotate360();
      shouldRotate = false;
      ROS_INFO("Finished rotating turtlebot");
    }

    // Check frontiers
    int count = getFrontiers();

    // Visualize all frontiers
    publishFrontierPoints(count);

    // Spin once to check for callbacks
    ros::spinOnce();

    // Sleep for desired frequency
    loop_rate.sleep();
  }
}
