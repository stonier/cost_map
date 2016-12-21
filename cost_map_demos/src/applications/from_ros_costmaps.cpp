/**
 * @file src/applications/alignment_test.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <atomic>
#include <ros/common.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cost_map_ros/converter.hpp>

#include "../../include/cost_map_demos/from_ros_costmaps.hpp"

/*****************************************************************************
** Interfaces
*****************************************************************************/



int main(int argc, char** argv)
{
  ros::init(argc, argv, "converter"); // , ros::init_options::AnonymousName);

  /********************
  ** Transforms
  ********************/
  cost_map_demos::TransformBroadcaster broadcaster;
  cost_map_demos::broadcastCostmap2DROSTestSuiteTransforms(broadcaster);

  /********************
  ** ROS Costmaps
  ********************/
  // MAKE SURE THIS STAY IN SYNC WITH src/tests/rostests/from_ros_costmaps/from_ros_costmaps.cpp!
  // The following should result in costmaps with strips of increasing cost in the x-direction,
  // but also 1-4 cells cleared by the footprint (see rviz)
  cost_map_demos::ROSCostmapServer ros_costmap_5x5("five_by_five", "base_link_5x5", cost_map::Position(0.0, 0.0), 5.0, 5.0);
  cost_map_demos::ROSCostmapServer ros_costmap_4x4("four_by_four", "base_link_4x4", cost_map::Position(0.0, -5.0), 4.0, 4.0);
  cost_map_demos::ROSCostmapServer ros_costmap_5x5_3x3_offset("five_by_five_three_by_three_offset", "base_link_5x5_3x3_offset", cost_map::Position(-6.0, 0.0), 5.0, 5.0);
  cost_map_demos::ROSCostmapServer ros_costmap_5x5_3x3_centre("five_by_five_three_by_three_centre", "base_link_5x5_3x3_centre", cost_map::Position(-6.0, -6.0), 5.0, 5.0);
  cost_map_demos::ROSCostmapServer ros_costmap_5x5_2_5x2_5_offset("five_by_five_twohalf_by_twohalf_offset", "base_link_5x5_2_5x2_5_offset", cost_map::Position(-12.0, 0.0), 5.0, 5.0);

  /********************
  ** New Costmaps
  ********************/
  // MAKE SURE THIS STAY IN SYNC WITH src/tests/rostests/from_ros_costmaps/from_ros_costmaps.cpp!
  cost_map::CostMap cost_map_5x5, cost_map_4x4, cost_map_5x5_3x3_offset, cost_map_5x5_3x3_centre, cost_map_5x5_2_5x2_5_offset;
  cost_map::fromCostMap2DROS(*(ros_costmap_5x5.getROSCostmap()), "obstacle_costs", cost_map_5x5);
  cost_map::fromCostMap2DROS(*(ros_costmap_4x4.getROSCostmap()), "obstacle_costs", cost_map_4x4);
  cost_map::Length geometry_3x3(3.0, 3.0);
  cost_map::fromCostMap2DROS(*(ros_costmap_5x5_3x3_offset.getROSCostmap()), geometry_3x3, "obstacle_costs", cost_map_5x5_3x3_offset);
  cost_map::fromCostMap2DROS(*(ros_costmap_5x5_3x3_centre.getROSCostmap()), geometry_3x3, "obstacle_costs", cost_map_5x5_3x3_centre);
  cost_map::Length geometry_2_5x2_5(2.5, 2.5);
  cost_map::fromCostMap2DROS(*(ros_costmap_5x5_2_5x2_5_offset.getROSCostmap()), geometry_2_5x2_5, "obstacle_costs", cost_map_5x5_2_5x2_5_offset);

  /********************
  ** Publishers
  ********************/
  ros::NodeHandle node_handle;
  ros::Publisher pub_5x5 = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_5x5", 1, true);
  ros::Publisher pub_4x4 = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_4x4", 1, true);
  ros::Publisher pub_5x5_3x3_offset = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_5x5_3x3_offset", 1, true);
  ros::Publisher pub_5x5_3x3_centre = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_5x5_3x3_centre", 1, true);
  ros::Publisher pub_5x5_2_5x2_5_offset = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_5x5_2_5x2_5_offset", 1, true);

  nav_msgs::OccupancyGrid occupancy_msg;
  cost_map::toOccupancyGrid(cost_map_5x5, cost_map_5x5.getLayers()[0], occupancy_msg);
  pub_5x5.publish(occupancy_msg);
  cost_map::toOccupancyGrid(cost_map_4x4, cost_map_4x4.getLayers()[0], occupancy_msg);
  pub_4x4.publish(occupancy_msg);
  cost_map::toOccupancyGrid(cost_map_5x5_3x3_offset, cost_map_5x5_3x3_offset.getLayers()[0], occupancy_msg);
  pub_5x5_3x3_offset.publish(occupancy_msg);
  cost_map::toOccupancyGrid(cost_map_5x5_3x3_centre, cost_map_5x5_3x3_centre.getLayers()[0], occupancy_msg);
  pub_5x5_3x3_centre.publish(occupancy_msg);
  cost_map::toOccupancyGrid(cost_map_5x5_2_5x2_5_offset, cost_map_5x5_2_5x2_5_offset.getLayers()[0], occupancy_msg);
  pub_5x5_2_5x2_5_offset.publish(occupancy_msg);

  /********************
  ** Spin & Shutdown
  ********************/
  ros::spin();

  return 0;
}
