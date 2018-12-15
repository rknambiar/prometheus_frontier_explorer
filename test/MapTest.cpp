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
 *  @brief Test for Map class file
 *
 *  @section DESCRIPTION
 *
 *  Map class implementation
 */

#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "FrontierExplorer.hpp"
#include <boost/shared_ptr.hpp>



class MapTest : public ::testing::Test {
 public:
  // Map objects
  nav_msgs::MapMetaData info;
  // Create object of map for testing utility functions
  Map testMap;

  // Map object for testing set methods
  Map setTestMap;

  // Map object for testing get methods
  Map getTestMap;

  // Variables for set test
  bool mapFlag;
  int widthTest;
  int heightTest;
  float resoTest;
  geometry_msgs::Point currcentertest;

  /**
   *  @brief Setup Function used during tests
   *
   *  @param none
   *
   *  @return void
   */
  void SetUp() {
    nav_msgs::OccupancyGridPtr testGrid(new nav_msgs::OccupancyGrid);
    // Build the info
    info.origin.position.x = 0;
    info.origin.position.y = 0;
    info.origin.orientation.w = 1;
    info.resolution = 1.0;
    info.width = 5;
    info.height = 5;

    testGrid->info = info;
    testGrid->data = {0, 0, 0, 0, 0,
      0, 0, 100, 0, 0,
      -1, 0, 100, 0, -1,
      -1, 0, 100, 0, -1,
      0, 0, 0, 0, 0
    };
    geometry_msgs::Point currcenter;
    currcenter.x = 0;
    currcenter.y = 0;
    testMap.updateMap(5, 5, 1.0, currcenter, testGrid);

    // Set params for get set
    mapFlag = true;
    widthTest = 5;
    heightTest = 5;
    resoTest = 1.0;
    currcentertest.x = 0.0;
    currcentertest.y = 0.0;
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

TEST_F(MapTest, testGridToMap) {
  std::vector<std::vector<MapNode>> map = testMap.getMap();
  auto prob = map[1][2].getProbability();
  int8_t testVal = 100;
  ASSERT_EQ(prob, testVal);
}


TEST_F(MapTest, testGetFrontiers) {
  int count = testMap.getFrontiers();
  ASSERT_EQ(count, 12);
}


TEST_F(MapTest, testGetClusters) {
  int count = testMap.getFrontiers();
  int clusters = testMap.getClusters(1);
  ASSERT_EQ(clusters, 2);
}


TEST_F(MapTest, testGetClusterCentroids) {
  int count = testMap.getFrontiers();
  int clusters = testMap.getClusters(1);
  std::vector<std::vector<MapNode>> map = testMap.getMap();
  std::vector<std::pair<double, double>> centroid =
      testMap.getClusterCentroids();
  ASSERT_NEAR(centroid[0].first, 0.666, 0.1);
  ASSERT_NEAR(centroid[0].second, 2.5, 0.1);
  ASSERT_NEAR(centroid[1].first, 3.333, 0.1);
  ASSERT_NEAR(centroid[1].second, 2.5, 0.1);
}


TEST_F(MapTest, testSetMapSet) {
  setTestMap.setMapSet(mapFlag);
  ASSERT_EQ(setTestMap.getMapSet(), mapFlag);
}

TEST_F(MapTest, testSetMapHeight) {
  setTestMap.setmapHeight(heightTest);
  ASSERT_EQ(setTestMap.getmapHeight(), heightTest);
}

TEST_F(MapTest, testSetMapWidth) {
  setTestMap.setmapWidth(widthTest);
  ASSERT_EQ(setTestMap.getmapWidth(), widthTest);
}

TEST_F(MapTest, testSetMapReso) {
  setTestMap.setmapReso(resoTest);
  ASSERT_EQ(setTestMap.getmapReso(), resoTest);
}

TEST_F(MapTest, testSetOrigin) {
  setTestMap.setOrigin(currcentertest);
  geometry_msgs::Point centerOut = setTestMap.getOrigin();
  ASSERT_EQ(centerOut.x, 0.0);
  ASSERT_EQ(centerOut.y, 0.0);
}

TEST_F(MapTest, testGetMapSet) {
  ASSERT_EQ(getTestMap.getMapSet(), false);
}

TEST_F(MapTest, testGetMapHeight) {
  ASSERT_EQ(getTestMap.getmapHeight(), 0);
}

TEST_F(MapTest, testGetMapWidth) {
  ASSERT_EQ(getTestMap.getmapWidth(), 0);
}

TEST_F(MapTest, testGetMapReso) {
  ASSERT_NEAR(getTestMap.getmapReso(), 0.05, 0.1);
}

TEST_F(MapTest, testGetOrigin) {
  geometry_msgs::Point centerOut = setTestMap.getOrigin();
  ASSERT_EQ(centerOut.x, 0.0);
  ASSERT_EQ(centerOut.y, 0.0);
}
