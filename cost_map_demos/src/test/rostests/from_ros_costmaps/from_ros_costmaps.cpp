/**
 * @file /cost_map_demos/tests/from_ros_costmaps.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_ros/cost_map_ros.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <string>

#include "../../../../include/cost_map_demos/from_ros_costmaps.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(CostMap2DROS, full_window) {
  std::cout << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << "                 Copy Full Window" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  // MAKE SURE THIS STAY IN SYNC WITH src/applications/from_ros_costmaps.cpp!
  // preparation
  std::string layer_name =  "obstacle_costs";
  cost_map_demos::ROSCostmapServer ros_costmap_5x5("five_by_five", "base_link_5x5", cost_map::Position(0.0, 0.0), 5.0, 5.0);
  cost_map::CostMap cost_map_5x5, cost_map_4x4;
  cost_map::fromCostMap2DROS(*(ros_costmap_5x5.getROSCostmap()), layer_name, cost_map_5x5);
  // assert map properties
  ASSERT_EQ(cost_map_5x5.getFrameId(), ros_costmap_5x5.getROSCostmap()->getGlobalFrameID());
  ASSERT_EQ(cost_map_5x5.getLength().x(),
            ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsX()*ros_costmap_5x5.getROSCostmap()->getCostmap()->getResolution()
            );
  ASSERT_EQ(cost_map_5x5.getLength().y(),
            ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsY()*ros_costmap_5x5.getROSCostmap()->getCostmap()->getResolution()
            );
  ASSERT_EQ(cost_map_5x5.getSize()[0], ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsX());
  ASSERT_EQ(cost_map_5x5.getSize()[1], ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsY());
  cost_map::Length position = cost_map_5x5.getPosition() - 0.5 * cost_map_5x5.getLength().matrix();
  ASSERT_EQ(position.x(), ros_costmap_5x5.getROSCostmap()->getCostmap()->getOriginX());
  ASSERT_EQ(position.y(), ros_costmap_5x5.getROSCostmap()->getCostmap()->getOriginY());
  // assert map data
//  for ( unsigned int i = 0; i < 5; ++i ) {
//    for ( unsigned int j = 0; j < 5; ++j ) {
//      std::cout << static_cast<int>(ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(i,j)) << " ";
//    }
//    std::cout << std::endl;
//  }
//  for ( unsigned int i = 0; i < 5; ++i ) {
//    for ( unsigned int j = 0; j < 5; ++j ) {
//      std::cout << static_cast<int>(cost_map_5x5.at(layer_name, cost_map::Index(i,j))) << " ";
//    }
//    std::cout << std::endl;
//  }
  // TODO a function which does the index conversion
  ASSERT_EQ(cost_map_5x5.at(layer_name, cost_map::Index(1,4)), ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(3,0));
  std::cout << std::endl;
}

TEST(CostMap2DROS, sub_window) {
  std::cout << std::endl;
  std::cout << "***********************************************************" << std::endl;
  std::cout << "                 Copy Sub Window" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  cost_map::CostMap cost_map_5x5, cost_map_4x4;
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


