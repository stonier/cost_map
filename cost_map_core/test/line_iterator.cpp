/**
 * @file /cost_map_core/test/line_iterator.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/cost_map_core/iterators/line_iterator.hpp"
#include "../include/cost_map_core/cost_map.hpp"

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace cost_map;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(LineIterator, StartOutsideMap)
{
  CostMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(2.0, 2.0), Position(0.0, 0.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, EndOutsideMap)
{
  CostMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(0.0, 0.0), Position(9.0, 6.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMap)
{
  CostMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(-7.0, -9.0), Position(8.0, 8.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  ++iterator;
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMapWithoutIntersectingMap)
{
  CostMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(-8.0, 8.0), Position(8.0, 8.0));

  EXPECT_TRUE(iterator.isPastEnd());
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}

