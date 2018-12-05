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
//TODO initialize constructor
}

Map::~Map() {
//TODO initialize destructor
}

void Map::setMapSet(bool value) {
  //TODO set map
}

bool Map::getMapSet() {
  //TODO get map
  return true;
}

int Map::getmapHeight() {
  //TODO get map height
return 1;
}

int Map::getmapWidth() {
  //TODO get map width
return 1;
}

double Map::getmapReso() {
  //TODO get map resolution
return 1.0;
}

void Map::setmapHeight(int value) {
  //TODO set map Height
}

void Map::setmapWidth(int value) {
  //TODO set map width
}

void Map::setmapReso(double value) {
  //TODO set map resolution
}

void Map::setOrigin(geometry_msgs::Point point) {
  //TODO set origin
}

geometry_msgs::Point Map::getOrigin() {
  //TODO get origin
}

std::vector<std::vector<MapNode>>& Map::getMap() {
  //TODO get map
}

bool Map::updateMapParams(int currentWidth, int currentHeight,
                          double currentReso,
                          geometry_msgs::Point currCenter) {
  //TODO update map parameters
return true;
}

void Map::updateMap(int currentWidth, int currentHeight, double currentReso,
                    geometry_msgs::Point mapCenter,
                    const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {
  //TODO update map
}
