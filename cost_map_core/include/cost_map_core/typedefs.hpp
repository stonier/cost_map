/**
 * @file /include/cost_map_core/cost_map-core.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_core_TYPEDEFS_HPP_
#define cost_map_core_TYPEDEFS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// primarily to bring in the grid_map eigen plugins
#include <grid_map_core/TypeDefs.hpp>

namespace cost_map {

  typedef Eigen::MatrixXf Matrix;
  typedef Eigen::Vector2d Position;
  typedef Eigen::Vector2d Vector;
  typedef Eigen::Vector3d Position3;
  typedef Eigen::Vector3d Vector3;
  typedef Eigen::Array2i Index;
  typedef Eigen::Array2i Size;
  typedef Eigen::Array2d Length;
  typedef uint64_t Time;

} /* namespace */

/*****************************************************************************
** Trailers
*****************************************************************************/

#endif /* cost_map_core_TYPEDEFS_HPP_ */
