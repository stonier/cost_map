/**
 * @file test/submap_iterator.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/cost_map_core/common.hpp"
#include "../include/cost_map_core/iterators/submap_iterator.hpp"

#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>
#include "../include/cost_map_core/cost_map.hpp"

using namespace std;
using namespace Eigen;

TEST(SubmapIterator, Simple)
{
  Eigen::Array2i submapTopLeftIndex(3, 1);
  Eigen::Array2i submapBufferSize(3, 2);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  cost_map::CostMap map(types);
  map.setGeometry(Array2d(8.1, 5.1), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)

  cost_map::SubmapIterator iterator(map, submapTopLeftIndex, submapBufferSize);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(submapTopLeftIndex(0), (*iterator)(0));
  EXPECT_EQ(submapTopLeftIndex(1), (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));
}

TEST(SubmapIterator, CircularBuffer)
{
  Eigen::Array2i submapTopLeftIndex(6, 3);
  Eigen::Array2i submapBufferSize(2, 4);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  cost_map::CostMap map(types);
  map.setGeometry(Array2d(8.1, 5.1), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)
  map.move(Vector2d(-3.0, -2.0)); // bufferStartIndex(3, 2)

  cost_map::SubmapIterator iterator(map, submapTopLeftIndex, submapBufferSize);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(submapTopLeftIndex(0), (*iterator)(0));
  EXPECT_EQ(submapTopLeftIndex(1), (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(2, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(2, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
