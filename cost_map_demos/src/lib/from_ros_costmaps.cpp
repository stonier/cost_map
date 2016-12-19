/**
 * @file /cost_map_demos/src/lib/from_ros_costmaps.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/cost_map_demos/from_ros_costmaps.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map_demos {

/*****************************************************************************
** Costmap2DROS Test Suite Helpers
*****************************************************************************/

void broadcastCostmap2DROSTestSuiteTransforms(TransformBroadcaster& broadcaster) {
  broadcaster.add("base_link_5x5", tf::Vector3(1.0,  1.0, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_4x4", tf::Vector3(1.0, -3.0, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_5x5_3x3_offset", tf::Vector3(-3.7, 2.4, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_5x5_3x3_centre", tf::Vector3(-3.5, -3.5, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_5x5_2_5x2_5_offset", tf::Vector3(-9.7, 2.4, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.startBroadCastingThread();
}

/*****************************************************************************
** ROS Costmap Server
*****************************************************************************/

ROSCostmapServer::ROSCostmapServer(const std::string& name,
                                   const std::string& base_link_transform_name,
                                   const cost_map::Position& origin,
                                   const double& width,
                                   const double& height
)
: transform_listener(ros::Duration(1.0))
{
  ros::NodeHandle private_node_handle("~");
  // lots of parameters here affect the construction ( e.g. rolling window)
  // if you don't have any parameters set, then this
  //  - alot of defaults which get dumped on the ros param server
  //  - fires up an obstacle layer and an inflation layer
  //  - creates a publisher for an occupancy grid
  private_node_handle.setParam(name + "/robot_base_frame", base_link_transform_name);
  private_node_handle.setParam(name + "/origin_x", origin.x());
  private_node_handle.setParam(name + "/origin_y", origin.y());
  private_node_handle.setParam(name + "/width", width);
  private_node_handle.setParam(name + "/height", height);
  private_node_handle.setParam(name + "/plugins", std::vector<std::string>());
  private_node_handle.setParam(name + "/resolution", 0.5);
  private_node_handle.setParam(name + "/robot_radius", 0.03); // clears 1 cell if inside, up to 4 cells on a vertex
  costmap = std::make_shared<ROSCostmap>(name, transform_listener);

  for ( unsigned int index = 0; index < costmap->getCostmap()->getSizeInCellsY(); ++index ) {
    unsigned int dimension = costmap->getCostmap()->getSizeInCellsX();
    // @todo assert dimension > 1
    // set the first row to costmap_2d::FREE_SPACE? but it shows up invisibly in rviz, so don't bother
    for ( unsigned int fill_index = 0; fill_index < dimension - 2; ++fill_index )
    {
      double fraction = static_cast<double>(fill_index + 1) / static_cast<double>(costmap->getCostmap()->getSizeInCellsX());
      costmap->getCostmap()->setCost(fill_index, index, fraction*costmap_2d::INSCRIBED_INFLATED_OBSTACLE );
    }
    costmap->getCostmap()->setCost(dimension - 2, index, costmap_2d::LETHAL_OBSTACLE);
    costmap->getCostmap()->setCost(dimension - 1, index, costmap_2d::NO_INFORMATION);
  }
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map_demos
