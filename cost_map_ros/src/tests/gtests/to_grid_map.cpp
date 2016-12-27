/**
 * @file src/tests/gtests/to_grid_map.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <grid_map_core/GridMap.hpp>
#include "../../../include/cost_map_ros/converter.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(ToGridMap, creation)
{
  cost_map::CostMap cost_map;
  cost_map.setGeometry(cost_map::Length(3.0, 4.0), 1.0, cost_map::Position(0.0, 0.0));
  cost_map.add("free_space", cost_map::FREE_SPACE);
  cost_map.add("no_information", cost_map::NO_INFORMATION);
  grid_map::GridMap grid_map;
  cost_map::toGridMap(cost_map, grid_map);
  EXPECT_EQ(grid_map.getSize().x(), cost_map.getSize().x());
  EXPECT_EQ(grid_map.getSize().y(), cost_map.getSize().y());
  EXPECT_EQ(2, grid_map.getLayers().size());
  EXPECT_EQ(grid_map.at("free_space", grid_map::Index(0,0)), 0.0);
  EXPECT_EQ(grid_map.at("no_information", grid_map::Index(0,0)), 100.0);
}


/*****************************************************************************
** Main
*****************************************************************************/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
