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
 *  @file    FrontierExplorer.hpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief FrontierExplorer header file
 *
 *  @section DESCRIPTION
 *
 * FrontierExplorer class header declaration
 */

#ifndef INCLUDE_FRONTIEREXPLORER_HPP_
#define INCLUDE_FRONTIEREXPLORER_HPP_

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// C++ header files
#include <iostream>
#include <utility>
#include <vector>
// ROS header files
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "Map.hpp"

/**
 * @brief FrontierExploration Class
 *
 * Class for frontier exploration.
 */
class FrontierExplorer {
 public:
  /**
   *  @brief Default constructor for FrontierExplorer class
   *
   *  @param none
   *
   *  @return none
   */
  FrontierExplorer();

  /**
   *  @brief Destructor for FrontierExplorer class
   *
   *  @param none
   *
   *  @return none
   */
  ~FrontierExplorer();

  /**
   *  @brief Callback function for processing Occupancy Grid
   *
   *  @param gridMsg Occupancy grid nav_msgs
   *
   *  @return void
   */
  void processOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg);

  /**
   *  @brief Wrapper function to start frontier exploration
   *
   *  @param none
   *
   *  @return void
   */
  void explore();

  /**
   *  @brief Function to rotate turtle bot 360 degrees
   *
   *  @param none
   *
   *  @return void
   */
  void rotate360();

  /**
   *  @brief Function to get frontiers from occupancy grid message
   *
   *  @param none
   *
   *  @return int count of frontiers
   */
  int getFrontiers();

  /**
   *  @brief Function to make clusters from array of frontier location
   *
   *  @param none
   *
   *  @return void
   */
  void getClusters();

  /**
   *  @brief Function to calculate cluster centroids
   *
   *  @param none
   *
   *  @return std::vector<std::pair<double, double>> cluster centroid locations
   */
  std::vector<std::pair<double, double>> getClusterCentroids();

  /**
   *  @brief Function to visualize all frontier points in Rviz
   *
   *  @param int count of clusters
   *
   *  @return void
   */
  void publishFrontierPoints(int count);

  /**
   *  @brief Function to visualize all cluster centers in Rviz
   *
   *  @param std::vector<std::pair<double, double>> center locations
   *
   *  @return void
   */
  void visualizeClusterCenters(std::vector<std::pair<double, double>> centers);

  /**
   *  @brief Function to visualize frontiers segregated based on clusters in Rviz
   *
   *  @param none
   *
   *  @return void
   */
  void visualizeClusterFrontiers();

 private:
  // ROS node handle
  ros::NodeHandle nh;

  // Sunscriber to Occupancy-map topic
  ros::Subscriber mapSub;

  // Variable to store turtle-bot velocity
  geometry_msgs::Twist velMsg;

  // Geometry message twist publisher
  ros::Publisher velPub;

  // Map object
  Map slamMap;

  // Publisher to show frontier based on clusters
  ros::Publisher frontierMarkerPub;

  // Publisher to show all frontiers
  ros::Publisher allFrontierPub;

  // Publisher to show frontier clusters
  ros::Publisher frontierClusterPub;

  // Structure to hold frontier clusters
  std::vector<std::vector<std::pair<int, int>>> frontierCluster;
};

#endif  // INCLUDE_FRONTIEREXPLORER_HPP_
