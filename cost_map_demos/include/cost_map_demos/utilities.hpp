/**
 * @file /cost_map_demos/include/cost_map_demos/utilities.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_demos_UTILITIES_HPP_
#define cost_map_demos_UTILITIES_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map_demos {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Pretty print a ros cost map in ascii format to stdout.
 * @param[in] ros_costmap : won't let us use a const here, but treat it as such
 */
void pretty_print(costmap_2d::Costmap2DROS &ros_costmap);

/**
 * @brief Pretty print a ros cost map in ascii format to stdout.
 *
 * @param[in] stream : the incoming object into which to insert stuff
 * @param[in] ros_costmap : won't let us use a const here, but treat it as such
 * @returns std::ostream
 */
std::ostream& operator <<(std::ostream& stream, costmap_2d::Costmap2DROS& ros_costmap);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map_demos

#endif /* cost_map_demos_UTILITIES_HPP_ */
