/*
 * EllipseIteratorTest.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "../include/cost_map_core/common.hpp"
#include "../include/cost_map_core/iterators/EllipseIterator.hpp"

#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>
#include "../include/cost_map_core/CostMap.hpp"

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
