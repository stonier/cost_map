/**
 * @file src/tests/gtests/from_occupancy_grid.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../../include/cost_map_ros/converter.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(FromOccupancyGrid, creation)
{
  const unsigned int size_x(3);
  const unsigned int size_y(4);
  const double resolution(1.0);
  const double origin_x(0.0);
  const double origin_y(0.0);
  std::string frame_id("map");
  
  nav_msgs::OccupancyGrid occupancy_grid;
  occupancy_grid.header.frame_id = frame_id;
  occupancy_grid.info.resolution = resolution;
  occupancy_grid.info.width = size_x;
  occupancy_grid.info.height = size_y;
  occupancy_grid.info.origin.position.x = origin_x;
  occupancy_grid.info.origin.position.y = origin_y;
  occupancy_grid.info.origin.orientation.w = 1.0;
  occupancy_grid.data.resize(size_x * size_y);

  cost_map::CostMap cost_map;
  bool done = cost_map::fromOccupancyGrid(occupancy_grid, "occupancy_grid", cost_map);

  EXPECT_EQ(true, done);
  EXPECT_EQ(frame_id, cost_map.getFrameId());
  EXPECT_EQ(size_x, cost_map.getSize().x());
  EXPECT_EQ(size_y, cost_map.getSize().y());
  EXPECT_EQ(resolution, cost_map.getResolution());
  // Different conventions of center of map.
  const double length_x(resolution * size_x);
  const double length_y(resolution * size_y);
  cost_map::Length length(length_x, length_y);
  cost_map::Position position(origin_x, origin_y);
  position += 0.5 * length.matrix();
  EXPECT_EQ(position, cost_map.getPosition());
  EXPECT_EQ(1, cost_map.getLayers().size());
}

TEST(FromOccupancyGrid, quaterneryConversion)
{
  const unsigned int size_x(2);
  const unsigned int size_y(2);

  nav_msgs::OccupancyGrid occupancy_grid;
  occupancy_grid.info.resolution = 1.0;
  occupancy_grid.info.width = size_x;
  occupancy_grid.info.height = size_y;
  occupancy_grid.info.origin.orientation.w = 1.0;
  occupancy_grid.data.resize(size_x * size_y);
  occupancy_grid.data[0] = -1; // UNKNOWN
  occupancy_grid.data[1] = 0; // FREE_SPACE
  occupancy_grid.data[2] = 100; // LETHAL_OBSTACLE
  occupancy_grid.data[3] = 99; // INSCRIBED_OBSTACLE

  cost_map::CostMap cost_map;
  bool done = cost_map::fromOccupancyGrid(occupancy_grid, "occupancy_grid", cost_map);

  EXPECT_EQ(true, done);
  // Different coordinate systems in occupancy_grid and cost_map
  // [(0,0), (0, 1), (1, 0), (1, 1)] -> [(1, 1), (0, 1), (1, 0), (0, 0)]
  EXPECT_EQ(cost_map::INSCRIBED_OBSTACLE, cost_map.at("occupancy_grid", cost_map::Index(0, 0)));
  EXPECT_EQ(cost_map::FREE_SPACE , cost_map.at("occupancy_grid", cost_map::Index(0, 1)));
  EXPECT_EQ(cost_map::LETHAL_OBSTACLE, cost_map.at("occupancy_grid", cost_map::Index(1, 0)));
  EXPECT_EQ(cost_map::NO_INFORMATION, cost_map.at("occupancy_grid", cost_map::Index(1, 1)));
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
