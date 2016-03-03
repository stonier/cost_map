/**
 * @file /grid_map_extras/include/grid_map_extras/normalise.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef grid_map_extras_INCLUDE_GRID_MAP_EXTRAS_NORMALISE_HPP_
#define grid_map_extras_INCLUDE_GRID_MAP_EXTRAS_NORMALISE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <grid_map/grid_map.hpp>
#include <limits>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace grid_map_extras {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Normalise and handle infinity for a grid map.
 *
 * This is useful for publishing to a point cloud.
 *
 * @param boundary_value : maximum bound of the normalisation (typically 1.0)
 * @param infinity_value
 */
void normalise(grid_map::GridMap& grid_map,
               const std::string& from_layer,
               const std::string& to_layer,
               const double& boundary_value = 1.0,
               const double& infinity_value = std::numeric_limits<double>::infinity());

}
/*****************************************************************************
** Trailers
*****************************************************************************/

#endif /* grid_map_extras_INCLUDE_GRID_MAP_EXTRAS_NORMALISE_HPP_ */
