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
  int width = slamMap.getmapWidth();
  int height = slamMap.getmapHeight();

  std::vector<std::vector<MapNode>>& map = slamMap.getMap();

  // Declare structure to store clusters
  std::vector<std::vector<std::pair<int, int>>> clusters;

  // Left half algorithm
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      if (!map[i][j].getisFrontier()) {
        continue;
      } else if (i - 1 >= 0 && j - 1 >= 0
          && map[i - 1][j - 1].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i - 1][j - 1].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      } else if ((i - 1 >= 0 && j - 1 >= 0
          && map[i - 1][j].getFrontierIndex() == -1
          && map[i][j - 1].getFrontierIndex() == -1)
          || (i - 1 < 0 && j - 1 >= 0 && map[i][j - 1].getFrontierIndex() == -1)
          || (i - 1 >= 0 && j - 1 < 0 && map[i - 1][j].getFrontierIndex() == -1)) {
        map[i][j].setFrontierIndex(clusters.size());
        std::vector<std::pair<int, int>> coordinates;
        coordinates.push_back(std::make_pair(i, j));
        clusters.push_back(coordinates);
      } else if (i - 1 >= 0 && map[i - 1][j].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i - 1][j].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      } else if (j - 1 >= 0 && map[i][j - 1].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i][j - 1].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      }
    }
  }

  // Right bruno algorithm

  for (int i = 0; i < height; i++) {
   for (int j = 0; j < width; j++) {
      if (!map[i][j].getisFrontier()) {
        continue;
      } else if (i - 1 >= 0 && j < width - 1
          && map[i - 1][j + 1].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i - 1][j + 1].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      }
    }
   }


  // Filtering out smaller clusters
  frontierCluster.clear();
  for (auto row : clusters) {
    if (row.size() > 20) {
      frontierCluster.push_back(row);
    }
  }

  ROS_INFO_STREAM("Number of clusters: " << frontierCluster.size());
}

std::vector<std::pair<double, double>> FrontierExplorer::getClusterCentroids() {
  std::vector<std::vector<MapNode>>& map = slamMap.getMap();
  std::vector<std::pair<double, double>> centroids;
  for (auto row : frontierCluster) {
    int i = 0, j = 0;
    double sumX = 0, sumY = 0;
    for (auto point : row) {
      i = point.first;
      j = point.second;
      sumX = sumX + map[i][j].getX();
      sumY = sumY + map[i][j].getY();
    }
    sumX = sumX / row.size();
    sumY = sumY / row.size();
    std::cout << "Centroid x: " << sumX << ", y: " << sumY << std::endl;
    centroids.push_back(std::make_pair(sumX, sumY));
  }
  return centroids;
}

int FrontierExplorer::getNearestCluster(
    std::vector<std::pair<double, double>> centers) {
  tf::StampedTransform transform;
  turtleFrameListener.lookupTransform("/map", "/base_link", ros::Time(0),
                                      transform);

  double turtleX = transform.getOrigin().x();
  double turtleY = transform.getOrigin().y();

  ROS_INFO_STREAM("Current turtle location x:" << turtleX << ", y:" << turtleY);

  // Index and distance to store the closest frontier
  int closestFrontierIndex = -1;
  double distance = -1;

  // Loop through all the clusters
  int loopIndex = 0;
  for (auto center : centers) {
    double currDistance = std::hypot(center.first - turtleX,
                                     center.second - turtleY);
    if (distance == -1 || (currDistance < distance && currDistance > 1.5)) {
      distance = currDistance;
      closestFrontierIndex = loopIndex;
    }
    loopIndex++;
  }

  ROS_INFO_STREAM("Found closest cluster at number: " << closestFrontierIndex);

  return closestFrontierIndex;
}

void FrontierExplorer::moveTurtle(
    std::vector<std::pair<double, double>> centers, int id) {
  double goalPointX = centers[id].first;
  double goalPointY = centers[id].second;

  // Create move base goal with params
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goalPointX;
  goal.target_pose.pose.position.y = goalPointY;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO_STREAM(
      "Navigating to point x:" << goalPointX << ", y:" << goalPointY
          << " on the map");

  actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction
      > ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("Move base action server online..");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Cluster centroid reached.");
  else
    ROS_INFO_STREAM("Could not reach cluster centroid.");

}

void FrontierExplorer::visualizeClusterCenters(std::vector<std::pair<double,
                                                double>> centers, int id) {
  visualization_msgs::MarkerArray clusterMarkerArray;
  int clusterIndex = 0;
  for (auto center : centers) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "cluster_center";
    marker.id = clusterIndex;
    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    if (clusterIndex == id) {
      // Define the scale (meter scale)
      marker.scale.x = 0.08;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;

      // Set the color
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.6;
    } else {
      // Define the scale (meter scale)
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      // Set the color
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.8;
    }
    marker.lifetime = ros::Duration();

    marker.pose.position.x = center.first;
    marker.pose.position.y = center.second;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;

    clusterIndex++;
    clusterMarkerArray.markers.push_back(marker);
  }

  frontierClusterPub.publish(clusterMarkerArray);
  ROS_INFO("Total %d markers published.", clusterIndex);
}

void FrontierExplorer::visualizeClusterFrontiers() {
  std::vector<std::vector<MapNode>>& map = slamMap.getMap();
  // 8 different colors(r,g,b). Loop after we get there
  int colorSize = 8;
  std::vector<std::tuple<double, double, double>> colors;
  colors.push_back(std::make_tuple(0.5, 0.0, 0.0));
  colors.push_back(std::make_tuple(0.0, 0.0, 1.0));
  colors.push_back(std::make_tuple(0.0, 1.0, 0.0));
  colors.push_back(std::make_tuple(0.0, 1.0, 1.0));
  colors.push_back(std::make_tuple(1.0, 0.0, 0.0));
  colors.push_back(std::make_tuple(1.0, 0.0, 1.0));
  colors.push_back(std::make_tuple(1.0, 1.0, 0.0));
  colors.push_back(std::make_tuple(1.0, 1.0, 1.0));

  int colorCounter = 1;
  int frontierIndex = 0;
  int frontierNodeIndex = 0;
  // Loop through the clusters
  for (auto row : frontierCluster) {
    visualization_msgs::MarkerArray clusterCenterMarkerArray;
    for (auto point : row) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time();
      marker.ns = "cluster_marker_frontier";
      marker.id = frontierNodeIndex;
      marker.type = visualization_msgs::Marker::CUBE;

      marker.action = visualization_msgs::Marker::ADD;

      // Define the scale (meter scale)
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;

      // Set the color
      marker.color.r = std::get < 0 > (colors[colorCounter - 1]);
      marker.color.g = std::get < 1 > (colors[colorCounter - 1]);
      marker.color.b = std::get < 2 > (colors[colorCounter - 1]);
      //      marker.color.r = 1.0;
      //      marker.color.g = 0.0;
      //      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration();

      marker.pose.position.x = map[point.first][point.second].getX();
      marker.pose.position.y = map[point.first][point.second].getY();
      marker.pose.position.z = 0;
      marker.pose.orientation.w = 1.0;

      clusterCenterMarkerArray.markers.push_back(marker);
      frontierNodeIndex++;
    }
    frontierIndex++;
    // Adjust for color
    if (colorCounter % 8 == 0) {
      colorCounter = 1;
    } else {
      colorCounter++;
    }
    frontierMarkerPub.publish(clusterCenterMarkerArray);
    ROS_INFO_STREAM(
        "Published cluster no: " << frontierIndex << ", with color index: "
            << colorCounter);
  }
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

    getClusters();

    // Get cluster centroids
    std::vector<std::pair<double, double>> clusterCenters =
        getClusterCentroids();

    // Get nearest cluster index
    int id = getNearestCluster(clusterCenters);

    // Visualize cluster centroids
    visualizeClusterCenters(clusterCenters, id);

    // Visualize cluster frontiers
    visualizeClusterFrontiers();

    // Visualize all frontiers
    publishFrontierPoints(count);

    // Move to the cluster center
    moveTurtle(clusterCenters, id);

    // Spin once to check for callbacks
    ros::spinOnce();

    // Sleep for desired frequency
    loop_rate.sleep();
  }
}
