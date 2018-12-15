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
 *  @file    MapNodeTest.cpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief Test for MapNode class file
 *
 *  @section DESCRIPTION
 *
 *  Test implementation for Mapnode class
 */
#include <gtest/gtest.h>
#include "MapNode.hpp"

/**
 * @brief MapNodeTest Class
 *
 * Test framework class for MapNodeTest.
 */
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

/**
 *@brief Test set node x method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testSetX) {
  testNodeSet.setX(x);
  ASSERT_EQ(testNodeSet.getX(), x);
}

/**
 *@brief Test set node y method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testSetY) {
  testNodeSet.setY(y);
  ASSERT_EQ(testNodeSet.getY(), y);
}

/**
 *@brief Test set node probability method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testSetProbability) {
  testNodeSet.setProbability(prob);
  ASSERT_EQ(testNodeSet.getProbability(), prob);
}

/**
 *@brief Test set node is frontier method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testSetIsFrontier) {
  testNodeSet.setisFrontier(frontierFlag);
  ASSERT_EQ(testNodeSet.getisFrontier(), frontierFlag);
}

/**
 *@brief Test set node frontierIndex method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testSetFrontierIndex) {
  testNodeSet.setFrontierIndex(frontierIndex);
  ASSERT_EQ(testNodeSet.getFrontierIndex(), frontierIndex);
}

/**
 *@brief Test get node x method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testGetX) {
  ASSERT_EQ(testNodeGet.getX(), -1);
}

/**
 *@brief Test get node y method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testGetY) {
  ASSERT_EQ(testNodeGet.getY(), -1);
}

/**
 *@brief Test get node probability method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testGetProbability) {
  ASSERT_EQ(testNodeGet.getProbability(), -1);
}

/**
 *@brief Test get node is frontier method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testGetIsFrontier) {
  ASSERT_EQ(testNodeGet.getisFrontier(), false);
}

/**
 *@brief Test get node frontier index method
 *
 *@param none
 *
 *@return none
 */
TEST_F(MapNodeTest, testGetFrontierIndex) {
  ASSERT_EQ(testNodeGet.getFrontierIndex(), -1);
}
