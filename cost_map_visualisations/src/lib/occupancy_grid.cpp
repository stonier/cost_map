/**
 * @file /cost_map_visualisations/src/lib/occupancy_grid.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core/cost_map.hpp>
#include <cost_map_ros/converter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "../../include/cost_map_visualisations/occupancy_grid.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

OccupancyGrid::OccupancyGrid()
{
  ros::NodeHandle nodehandle("~");

  subscriber_ = nodehandle.subscribe("cost_map", 10, &OccupancyGrid::_costMapCallback, this);
}

void OccupancyGrid::_costMapCallback(const cost_map_msgs::CostMap::ConstPtr& msg) {
  for (const auto& layer : msg->layers) {
    if ( publishers_.count(layer) == 0 ) {
      ros::NodeHandle nodehandle("~");
      auto result = publishers_.insert(std::pair<std::string, ros::Publisher>(layer, ros::Publisher(nodehandle.advertise<nav_msgs::OccupancyGrid>(layer, 1, true))));
    }
    // TODO check if layers disappeared and remove the publishers
    ros::Publisher& publisher = publishers_[layer];
    if (publisher.getNumSubscribers() >= 0) {
      cost_map::CostMap cost_map;
      cost_map::fromMessage(*msg, cost_map);
      nav_msgs::OccupancyGrid occupancy_grid_msg;
      cost_map::toOccupancyGrid(cost_map, layer, occupancy_grid_msg);
      publisher.publish(occupancy_grid_msg);
    }
  }
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
