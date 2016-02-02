/**
 * @file /cost_map_visualisations/src/lib/node.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <ros/ros.h>
#include "../../include/cost_map_visualisations/occupancy_grid.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  ros::init(argc, argv, "cost_map_visualisations");
  ros::NodeHandle nodehandle("~");
  cost_map::OccupancyGrid occupancy_grid(nodehandle);
  ros::spin();
  return EXIT_SUCCESS;
}
