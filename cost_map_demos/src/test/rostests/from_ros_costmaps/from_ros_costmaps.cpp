/**
 * @file /cost_map_demos/tests/from_ros_costmaps.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <iostream>

#include "../../../../include/cost_map_demos/from_ros_costmaps.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(CostMap2DROS, full_window) {
  std::cout << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << "                 Copy Full Window" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << std::endl;
}

TEST(CostMap2DROS, sub_window) {
  std::cout << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << "                 Copy Sub Window" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << std::endl;
}


/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_from_ros_costmaps");

  cost_map_demos::TransformBroadcaster broadcaster;
  cost_map_demos::broadcastCostmap2DROSTestSuiteTransforms(broadcaster);

  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}


