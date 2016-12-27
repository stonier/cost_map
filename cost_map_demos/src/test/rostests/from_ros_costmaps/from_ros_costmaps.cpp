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

TEST(Costmap2DROS, full_window) {
  std::cout << std::endl;
  ROS_INFO("***********************************************************");
  ROS_INFO("                 Copy Full Window");
  ROS_INFO("***********************************************************");
  // MAKE SURE THIS STAY IN SYNC WITH src/applications/from_ros_costmaps.cpp!
  // preparation
  std::string layer_name =  "obstacle_costs";
  cost_map_demos::ROSCostmapServer ros_costmap_5x5("five_by_five", "base_link_5x5", cost_map::Position(0.0, 0.0), 5.0, 5.0);
  cost_map::CostMap cost_map_5x5, cost_map_4x4;
  cost_map::fromCostmap2DROS(*(ros_costmap_5x5.getROSCostmap()), layer_name, cost_map_5x5);
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
  ASSERT_EQ(cost_map_5x5.at(layer_name, cost_map::Index(1,9)), ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(8,0));
  std::cout << std::endl;
}

TEST(Costmap2DROS, cost_map_centres) {
  std::cout << std::endl;
  ROS_INFO("***********************************************************");
  ROS_INFO("                 Check Subwindow Centres");
  ROS_INFO("***********************************************************");
  ROS_INFO("Subwindows are centred as closely as possible to the robot");
  ROS_INFO("pose, though not exactly. They still need to align with");
  ROS_INFO("the underlying ros costmap so that they don't introduce a");
  ROS_INFO("new kind of error. As a result, the centre is shifted from");
  ROS_INFO("the robot pose to the nearest appropriate point which aligns");
  ROS_INFO("the new cost map exactly on top of the original ros costmap.");
  std::cout << std::endl;
  std::string layer_name =  "obstacle_costs";
  cost_map_demos::ROSCostmapServer ros_costmap_5x5_3x3_offset("five_by_five_three_by_three_offset", "base_link_5x5_3x3_offset", cost_map::Position(-6.0, 0.0), 5.0, 5.0);
  cost_map_demos::ROSCostmapServer ros_costmap_5x5_3x3_centre("five_by_five_three_by_three_centre", "base_link_5x5_3x3_centre", cost_map::Position(-6.0, -6.0), 5.0, 5.0);
  cost_map_demos::ROSCostmapServer ros_costmap_5x5_2_5x2_5_offset("five_by_five_twohalf_by_twohalf_offset", "base_link_5x5_2_5x2_5_offset", cost_map::Position(-12.0, 0.0), 5.0, 5.0);
  cost_map::CostMap cost_map_5x5_3x3_offset, cost_map_5x5_3x3_centre, cost_map_5x5_2_5x2_5_offset;
  cost_map::Length geometry_3x3(3.0, 3.0);
  cost_map::fromCostmap2DROSAtRobotPose(*(ros_costmap_5x5_3x3_offset.getROSCostmap()), geometry_3x3, layer_name, cost_map_5x5_3x3_offset);
  cost_map::fromCostmap2DROSAtRobotPose(*(ros_costmap_5x5_3x3_centre.getROSCostmap()), geometry_3x3, layer_name, cost_map_5x5_3x3_centre);
  cost_map::Length geometry_2_5x2_5(2.5, 2.5);
  cost_map::fromCostmap2DROSAtRobotPose(*(ros_costmap_5x5_2_5x2_5_offset.getROSCostmap()), geometry_2_5x2_5, layer_name, cost_map_5x5_2_5x2_5_offset);
  ROS_INFO_STREAM("  cost_map_5x5_3x3_offset : " << cost_map_5x5_3x3_offset.getPosition().transpose());
  ROS_INFO_STREAM("  cost_map_5x5_3x3_offset : " << cost_map_5x5_3x3_centre.getPosition().transpose());
  ROS_INFO_STREAM("  cost_map_5x5_3x3_offset : " << cost_map_5x5_2_5x2_5_offset.getPosition().transpose());
  ASSERT_EQ(-3.5, cost_map_5x5_3x3_offset.getPosition().x());
  ASSERT_EQ(2.5, cost_map_5x5_3x3_offset.getPosition().y());
  ASSERT_EQ(-3.5, cost_map_5x5_3x3_centre.getPosition().x());
  ASSERT_EQ(-3.5, cost_map_5x5_3x3_centre.getPosition().y());
  ASSERT_EQ(-9.75, cost_map_5x5_2_5x2_5_offset.getPosition().x());
  ASSERT_EQ(2.25, cost_map_5x5_2_5x2_5_offset.getPosition().y());
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
  int result = RUN_ALL_TESTS();
  broadcaster.shutdown();
  return result;
}


