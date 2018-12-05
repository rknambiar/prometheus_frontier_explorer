#include <gtest/gtest.h>
#include "MapNode.hpp"

TEST(MapNodeTest, testGetterSetter) {
  MapNode testNode;
  float x = 2.0;
  float y = 3.0;
  int8_t prob = 5;
  bool frontierFlag = true;
  int frontierIndex = 6;

  // Set value of x and y in node
  testNode.setX(x);
  testNode.setY(y);
  testNode.setProbability(prob);
  testNode.setisFrontier(frontierFlag);
  testNode.setFrontierIndex(frontierIndex);

  EXPECT_EQ(testNode.getX(), x);
  EXPECT_EQ(testNode.getY(), y);
  EXPECT_EQ(testNode.getProbability(), prob);
  EXPECT_TRUE(testNode.getisFrontier());
  EXPECT_EQ(testNode.getFrontierIndex(), frontierIndex);
}
