/**
 * @file /cost_map_visualisations/include/cost_map_visualisations/occupancy_grid.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_visualisations_OCCUPANCY_GRID_HPP_
#define cost_map_visualisations_OCCUPANCY_GRID_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_msgs/CostMap.h>
#include <ros/ros.h>
#include <map>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class OccupancyGrid {
public:
  OccupancyGrid(ros::NodeHandle& nodehandle);


private:
  void _costMapCallback(const cost_map_msgs::CostMap::ConstPtr& msg);

  ros::NodeHandle& nodehandle_;
  ros::Subscriber subscriber_;
  std::map<std::string, ros::Publisher> publishers_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_visualisations_OCCUPANCY_GRID_HPP_ */
