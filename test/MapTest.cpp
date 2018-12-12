#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "FrontierExplorer.hpp"
#include <boost/shared_ptr.hpp>

TEST(MapTest, testGridToMap) {
  ros::NodeHandle nh;
  nav_msgs::OccupancyGridPtr testGrid(new nav_msgs::OccupancyGrid);
  nav_msgs::MapMetaData info;

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

  // Create object of map
  Map testMap;
  geometry_msgs::Point currcenter;
  currcenter.x = 0;
  currcenter.y = 0;
  testMap.updateMap(5, 5, 1.0, currcenter, testGrid);

  std::vector<std::vector<MapNode>> map = testMap.getMap();
  auto prob = map[1][2].getProbability();
  int8_t dummy = 100;
  ASSERT_EQ(prob, dummy);
}

