/**
 * @file test/polygon_iterator.cpp
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
#include <grid_map_core/Polygon.hpp>
#include "../include/cost_map_core/iterators/polygon_iterator.hpp"

using namespace std;
using namespace Eigen;

TEST(PolygonIterator, FullCover)
{
  vector<string> types;
  types.push_back("type");
  cost_map::CostMap map(types);
  map.setGeometry(cost_map::Length(8.0, 5.0), 1.0, cost_map::Position(0.0, 0.0)); // bufferSize(8, 5)

  cost_map::Polygon polygon;
  polygon.addVertex(cost_map::Position(-100.0, 100.0));
  polygon.addVertex(cost_map::Position(100.0, 100.0));
  polygon.addVertex(cost_map::Position(100.0, -100.0));
  polygon.addVertex(cost_map::Position(-100.0, -100.0));

  cost_map::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 37; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Outside)
{
  cost_map::CostMap map({"types"});
  map.setGeometry(cost_map::Length(8.0, 5.0), 1.0, cost_map::Position(0.0, 0.0)); // bufferSize(8, 5)

  cost_map::Polygon polygon;
  polygon.addVertex(cost_map::Position(99.0, 101.0));
  polygon.addVertex(cost_map::Position(101.0, 101.0));
  polygon.addVertex(cost_map::Position(101.0, 99.0));
  polygon.addVertex(cost_map::Position(99.0, 99.0));

  cost_map::PolygonIterator iterator(map, polygon);

  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Square)
{
  cost_map::CostMap map({"types"});
  map.setGeometry(cost_map::Length(8.0, 5.0), 1.0, cost_map::Position(0.0, 0.0)); // bufferSize(8, 5)

  cost_map::Polygon polygon;
  polygon.addVertex(cost_map::Position(-1.0, 1.5));
  polygon.addVertex(cost_map::Position(1.0, 1.5));
  polygon.addVertex(cost_map::Position(1.0, -1.5));
  polygon.addVertex(cost_map::Position(-1.0, -1.5));

  cost_map::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, TopLeftTriangle)
{
  cost_map::CostMap map({"types"});
  map.setGeometry(cost_map::Length(8.0, 5.0), 1.0, cost_map::Position(0.0, 0.0)); // bufferSize(8, 5)

  cost_map::Polygon polygon;
  polygon.addVertex(cost_map::Position(-40.1, 20.6));
  polygon.addVertex(cost_map::Position(40.1, 20.4));
  polygon.addVertex(cost_map::Position(-40.1, -20.6));

  cost_map::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  // TODO Extend.
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}

