/**
 * @file /include/cost_map_core/common.hpp
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
#include <grid_map_core/BufferRegion.hpp>
#include <grid_map_core/Polygon.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Typedefs
*****************************************************************************/
  // the biggie - this one is different
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> Matrix;
  typedef unsigned char DataType;
  extern const unsigned char NO_INFORMATION;
  extern const unsigned char LETHAL_OBSTACLE;
  extern const unsigned char INSCRIBED_OBSTACLE;
  extern const unsigned char FREE_SPACE;

  typedef grid_map::Matrix DataMatrix;
  //typedef grid_map::Matrix Matrix;

  // these are in grid_map_core/TypeDefs.hpp, just bring here for convenience
  typedef grid_map::Position Position;
  typedef grid_map::Vector Vector;
  typedef grid_map::Position3 Position3;
  typedef grid_map::Vector3 Vector3;
  typedef grid_map::Index Index;
  typedef grid_map::Size Size;
  typedef grid_map::Length Length;
  typedef grid_map::Time Time;

  // common classes
  typedef grid_map::BufferRegion BufferRegion;
  typedef grid_map::Polygon Polygon;

/*****************************************************************************
** Trailers
*****************************************************************************/

} /* namespace cost_map */

#endif /* cost_map_core_TYPEDEFS_HPP_ */
