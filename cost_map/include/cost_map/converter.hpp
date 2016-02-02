/**
 * @file /cost_map/include/cost_map/converter.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_CONVERTER_HPP_
#define cost_map_CONVERTER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <grid_map/grid_map.hpp>
#include <cost_map_core/cost_map_core.hpp>
#include <cost_map_msgs/CostMap.h>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Interfaces
*****************************************************************************/

CostMap loadFromImageFile(const std::string& filename);

bool addLayerFromROSImage(const sensor_msgs::Image& image,
                          const std::string& layer,
                          cost_map::CostMap& cost_map
                          );

/*!
 * Converts all layers of a grid map object to a ROS grid map message.
 * @param[in] gridMap the grid map object.
 * @param[out] message the grid map message to be populated.
 */
void toMessage(const cost_map::CostMap& cost_map, cost_map_msgs::CostMap& message);

grid_map::GridMap toGridMap(const cost_map::CostMap cost_map);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_CONVERTER_HPP_ */
