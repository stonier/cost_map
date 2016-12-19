/**
 * @file /cost_map_demos/include/cost_map_demos/from_ros_costmaps.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_demos_FROM_ROS_COSTMAPS_HPP_
#define cost_map_demos_FROM_ROS_COSTMAPS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core/cost_map_core.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <memory>
#include <string>

#include "utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map_demos {

/*****************************************************************************
** Costmap2DROS Demo/Test Classes
*****************************************************************************/
/**
 * Some partial customation of various ros costmaps for use with
 * converter demos and tests.
 *
 * Characteristics/Constraints:
 *
 * - fills with stripes, each with an increasing cost value
 * - second last stripe is filled with LETHAL_OBSTACLE cost
 * - last stripe is filled with NO_INFORMATION cost
 */
class ROSCostmapServer {
public:
  typedef costmap_2d::Costmap2DROS ROSCostmap;
  typedef std::shared_ptr<ROSCostmap> ROSCostmapPtr;

  ROSCostmapServer(const std::string& name,
                   const std::string& base_link_transform_name,
                   const cost_map::Position& origin,
                   const double& width,
                   const double& height
  );

  ROSCostmapPtr getROSCostmap() { return costmap; };

private:
  ROSCostmapPtr costmap;
  tf::TransformListener transform_listener;
};


/*****************************************************************************
** Costmap2DROS Demo/Test Suite Helpers
*****************************************************************************/

/**
 * Broadcast a set of transform useful for the suite of Costmap2DROS
 * converter demos and tests
 *
 * @param[in] broadcaster : uninitialised broadcaster object
 */
void broadcastCostmap2DROSTestSuiteTransforms(TransformBroadcaster& broadcaster);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map_demos

#endif /* cost_map_demos_FROM_ROS_COSTMAPS_HPP_ */
