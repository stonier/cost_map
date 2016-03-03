/**
 * @file /grid_map_extras/src/lib/normalise.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <limits>
#include "../../include/grid_map_extras/normalise.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace grid_map_extras {

/*****************************************************************************
** Implementation
*****************************************************************************/

void normalise(grid_map::GridMap& grid_map,
               const std::string& from_layer,
               const std::string& to_layer,
               const double& boundary_value)
{
  grid_map::Index costmap_index;
  grid_map.add(to_layer, 0.0);
  grid_map::Matrix& data_normalised = grid_map.get(to_layer);
  grid_map::Matrix& data_original = grid_map.get(from_layer);
  double max_magnitude = 0.0001;
  for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator) {
    int i = (*iterator)(0);
    int j = (*iterator)(1);
    double value = data_original(i, j);
    if ( ( std::abs(value) > max_magnitude ) && (value != std::numeric_limits<double>::infinity()) ) {
      max_magnitude = value;
    }
  }
  for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator) {
    int i = (*iterator)(0);
    int j = (*iterator)(1);
    double value = data_original(i, j);
    if (value == std::numeric_limits<double>::infinity()) {
      data_normalised(i, j) = value;
    } else {
      data_normalised(i, j) = boundary_value*data_original(i,j)/max_magnitude;
    }
  }

}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace grid_map_extras
