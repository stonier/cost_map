/**
 * @file /cost_map_core/src/lib/common.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/cost_map_core/common.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

// designed to match those in costmap_2d/cost_values.h
const unsigned char NO_INFORMATION = 255;
const unsigned char LETHAL_OBSTACLE = 254;
const unsigned char INSCRIBED_OBSTACLE = 253;
const unsigned char FREE_SPACE = 0;

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map_core
