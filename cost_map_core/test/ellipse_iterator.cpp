/**
 * @file test/ellipse_iterator.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/cost_map_core/common.hpp"
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>
#include "../include/cost_map_core/cost_map.hpp"
#include "../include/cost_map_core/iterators/ellipse_iterator.hpp"

using namespace std;
using namespace Eigen;

TEST(EllipseIterator, OneCellWideEllipse)
{
  cost_map::CostMap map( { "types" });
  map.setGeometry(cost_map::Length(8.0, 5.0), 1.0, cost_map::Position(0.0, 0.0));

  cost_map::EllipseIterator iterator(map, cost_map::Position(0.0, 0.0), cost_map::Length(8.0, 1.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  ++iterator;
  ++iterator;
  ++iterator;
  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}

