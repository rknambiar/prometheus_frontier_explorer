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
 *  @file    Map.cpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief Map class implementation file
 *
 *  @section DESCRIPTION
 *
 *  Map class implementation
 */
#include "Map.hpp"

Map::Map() {
  // Set Map parameters on object creation
  mapSet = false;
  mapHeight = 0;
  mapWidth = 0;
  mapReso = 0.05;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
}

Map::~Map() {
}

void Map::setMapSet(bool value) {
  mapSet = value;
}

bool Map::getMapSet() {
  return mapSet;
}

int Map::getmapHeight() {
  return mapHeight;
}

int Map::getmapWidth() {
  return mapWidth;
}

double Map::getmapReso() {
  return mapReso;
}

void Map::setmapHeight(int value) {
  mapHeight = value;
}

void Map::setmapWidth(int value) {
  mapWidth = value;
}

void Map::setmapReso(double value) {
  mapReso = value;
}

void Map::setOrigin(geometry_msgs::Point point) {
  origin = point;
}

geometry_msgs::Point Map::getOrigin() {
  return origin;
}

std::vector<std::vector<MapNode>>& Map::getMap() {
  //  std::cout << "Map sent with width: " << map.size() << ", Height: "
  //            << map[0].size() << std::endl;
  return map;
}

bool Map::updateMapParams(int currentWidth, int currentHeight,
                          double currentReso,
                          geometry_msgs::Point currCenter) {
  bool updateFlag = false;
  if (currentWidth != mapWidth) {
    ROS_INFO_STREAM(
        "Map width updated from " << mapWidth << " to " << currentWidth);
    mapWidth = currentWidth;
    updateFlag = true;
  }

  if (currentHeight != mapHeight) {
    ROS_INFO_STREAM(
        "Map height updated from " << mapHeight << " to " << currentHeight);
    mapHeight = currentHeight;
    updateFlag = true;
  }

  if (currentReso != mapReso) {
    ROS_INFO_STREAM(
        "Map resolution updated from " << mapReso << " to " << currentReso);
    mapReso = currentReso;
    updateFlag = true;
  }

  if (currCenter.x != origin.x || currCenter.y != origin.y) {
    origin.x = currCenter.x;
    origin.y = currCenter.y;
    origin.z = currCenter.z;
    ROS_INFO_STREAM(
        "Map origin updated at x:" << origin.x << ", y:" << origin.y);
  }
  return updateFlag;
}

void Map::updateMap(int currentWidth, int currentHeight, double currentReso,
                    geometry_msgs::Point mapCenter,
                    const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {
  //  ROS_INFO("Update Map function received with width:%d, height:%d",
  //           currentWidth, currentHeight);
  /* Check is map has been updated. If yes, set mapSet flag to false to reset
   the map */
  if (updateMapParams(currentWidth, currentHeight, currentReso, mapCenter)) {
    mapSet = false;
    ROS_INFO(
        "Map reset as one of the parameter has been updated. Will initialize again..");
  }

  int mapIter = 0;
  if (!mapSet) {
    mapSet = true;
    // Clearing out the vector incase re-initialized
    map.clear();

    // Loop to fill the map. We go from every width for each height
    for (int i = 0; i < currentHeight; i++) {
      std::vector<MapNode> rowNodes;
      for (int j = 0; j < currentWidth; j++) {
        MapNode currentNode;

        // Calculate x and y
        float x = j * mapReso + origin.x;
        float y = i * mapReso + origin.y;

        // Update in each node of map
        currentNode.setX(x);
        currentNode.setY(y);
        currentNode.setProbability(gridMsg->data[mapIter]);
        rowNodes.push_back(currentNode);
        mapIter++;
      }
      map.push_back(rowNodes);
    }
    ROS_INFO_STREAM(
        "Map Initialized. Current w:" << map[0].size() << ", h:" << map.size()
            << ", resolution: " << mapReso << ", origin x: " << origin.x
            << ", y: " << origin.y);
  } else {
    for (int i = 0; i < currentHeight; i++) {
      for (int j = 0; j < currentWidth; j++) {
        map[i][j].setProbability(gridMsg->data[mapIter]);
        mapIter++;
      }
    }
    ROS_INFO_STREAM(
        "Map Updated. Current w:" << map[0].size() << " ,h:" << map.size()
            << ", resolution: " << mapReso << ", origin x: " << origin.x
            << ", y: " << origin.y);
  }
}
