/**
 * @file /cost_map_demos/src/lib/utilities.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/cost_map_demos/utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map_demos {

/*****************************************************************************
** Implementation
*****************************************************************************/

void pretty_print(costmap_2d::Costmap2DROS &ros_costmap) {
  for ( unsigned int i = 0; i < ros_costmap.getCostmap()->getSizeInCellsX(); ++i ) {
    for ( unsigned int j = 0; j < ros_costmap.getCostmap()->getSizeInCellsY(); ++j ) {
      std::cout << static_cast<int>(ros_costmap.getCostmap()->getCost(i,j)) << " ";
    }
    std::cout << std::endl;
  }
}

std::ostream& operator <<(std::ostream& stream, costmap_2d::Costmap2DROS& ros_costmap) {
  for ( unsigned int i = 0; i < ros_costmap.getCostmap()->getSizeInCellsX(); ++i ) {
    for ( unsigned int j = 0; j < ros_costmap.getCostmap()->getSizeInCellsY(); ++j ) {
      stream << static_cast<int>(ros_costmap.getCostmap()->getCost(i,j)) << " ";
    }
    stream << std::endl;
  }
  return stream;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map_demos
