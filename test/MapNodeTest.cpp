#include <gtest/gtest.h>
#include "MapNode.hpp"

class MapNodeTest : public ::testing::Test {
 public:
  // MapNode objects
  MapNode testNodeSet;
  MapNode testNodeGet;
  float x;
  float y;
  int8_t prob;
  bool frontierFlag;
  int frontierIndex;

  /**
   *  @brief Setup Function used during tests
   *
   *  @param none
   *
   *  @return void
   */
  void SetUp() {
    x = 2.0;
    y = 3.0;
    prob = 5;
    frontierFlag = true;
    frontierIndex = 6;
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

TEST_F(MapNodeTest, testSetX) {
  testNodeSet.setX(x);
  ASSERT_EQ(testNodeSet.getX(), x);
}

TEST_F(MapNodeTest, testSetY) {
  testNodeSet.setY(y);
  ASSERT_EQ(testNodeSet.getY(), y);
}

TEST_F(MapNodeTest, testSetProbability) {
  testNodeSet.setProbability(prob);
  ASSERT_EQ(testNodeSet.getProbability(), prob);
}

TEST_F(MapNodeTest, testSetIsFrontier) {
  testNodeSet.setisFrontier(frontierFlag);
  ASSERT_EQ(testNodeSet.getisFrontier(), frontierFlag);
}

TEST_F(MapNodeTest, testSetFrontierIndex) {
  testNodeSet.setFrontierIndex(frontierIndex);
  ASSERT_EQ(testNodeSet.getFrontierIndex(), frontierIndex);
}

TEST_F(MapNodeTest, testGetX) {
  ASSERT_EQ(testNodeGet.getX(), -1);
}

TEST_F(MapNodeTest, testGetY) {
  ASSERT_EQ(testNodeGet.getY(), -1);
}

TEST_F(MapNodeTest, testGetProbability) {
  ASSERT_EQ(testNodeGet.getProbability(), -1);
}

TEST_F(MapNodeTest, testGetIsFrontier) {
  ASSERT_EQ(testNodeGet.getisFrontier(), false);
}

TEST_F(MapNodeTest, testGetFrontierIndex) {
  ASSERT_EQ(testNodeGet.getFrontierIndex(), -1);
}
