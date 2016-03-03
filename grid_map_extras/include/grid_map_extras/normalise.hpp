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

#include <grid_map_core/grid_map_core.hpp>
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
 * This is useful for publishing text representations of the cloud where we'd
 * like to limit the digits used.
 *
 * @param boundary_value : maximum bound of the normalisation (typically 1.0)
 * @param infinity_value
 */
void normalise(grid_map::GridMap& grid_map,
               const std::string& from_layer,
               const std::string& to_layer,
               const double& boundary_value = 1.0);
}

/*****************************************************************************
** Trailers
*****************************************************************************/

#endif /* grid_map_extras_INCLUDE_GRID_MAP_EXTRAS_NORMALISE_HPP_ */
