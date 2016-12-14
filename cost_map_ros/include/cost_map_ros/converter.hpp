/**
 * @file /cost_map_ros/include/cost_map/converter.hpp
 *
 * Methods in this file should in general, adopt the following signatures:
 *
 * - initialiseFromXYZ(const auto& in, cost_map::CostMap& cost_map) : delete all layers, set the geeomtry, frame id
 * - addLayerFromXYZ(const auto& in, cost_map::CostMap& cost_map)   : assert cost map properties, then add data layer
 * - toXYZ(const auto& in, cost_map::CostMap& cost_map)             : convert cost_map to the specified object
 * - fromXYZ(const auto& in, cost_map::CostMap& cost_map)           : convert specified object to cost_map (i.e. completely instantiate)
 *
 * Note : important to pass in cost_map objects as a reference. This provides
 * the flexibility to use this api with either objects or smart pointers.
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_ros_CONVERTER_HPP_
#define cost_map_ros_CONVERTER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// grid maps
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cost_map_core/cost_map_core.hpp>
#include <cost_map_msgs/CostMap.h>
#include <cost_map_msgs/GetCostMap.h>

// ros
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>

// general
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Image Bundles
*****************************************************************************/

/**
 * @brief Initialises a adds a single layer from a yaml/image resource pair.
 *
 * The image resource pair
 * @warning this will change the geometry of the provided costmap and delete all layers!
 * @todo extend this so it can load multiple layers
 *
 * @param[in] filename : yaml file
 * @param[out] cost_map :
 */
void fromImageBundle(const std::string& filename, cost_map::CostMap& cost_map);
/**
 * @brief Dump a cost map to an image bundle set of files.
 *
 * This creates the specified yaml file with image bundle meta information
 * and a set of png images alongside, one for each layer in the cost map.
 *
 * @param[in] filename : name of the yaml file to write
 * @param[in] cost_map : cost map to dump
 */
void toImageBundle(const std::string& filename, cost_map::CostMap& cost_map);

// this might be an interesting api to have too
// void addLayerFromImageFile()

/*****************************************************************************
** CostMap and GridMap
*****************************************************************************/

/**
 * @todo should be a void function with ref argument so people
 * can use smart pointers or objects with this function
 * @param cost_map
 * @return
 */
grid_map::GridMap toGridMap(const cost_map::CostMap cost_map);

/*****************************************************************************
** ROS Messages
*****************************************************************************/

/*!
 * Converts all layers of a grid map object to a ROS grid map message.
 * @param[in] gridMap the grid map object.
 * @param[out] message the grid map message to be populated.
 */
void toMessage(const cost_map::CostMap& cost_map, cost_map_msgs::CostMap& message);

/*!
 * Converts a ROS cost map message to a cost map object.
 * @param[in] message the cost map message.
 * @param[out] cost_map the cost map object to be initialized.
 * @return true if successful, false otherwise.
 */
bool fromMessage(const cost_map_msgs::CostMap& message, cost_map::CostMap& cost_map);

/*****************************************************************************
** ROS Images
*****************************************************************************/

bool addLayerFromROSImage(const sensor_msgs::Image& image,
                          const std::string& layer,
                          cost_map::CostMap& cost_map
                          );

/*****************************************************************************
** Ros CostMap2D & Occupancy Grids
*****************************************************************************/
/*
 * There are various ways we can pull from ros costmaps. Build up a suite
 * of functions as we need to.
 */

/**
 * @brief Converts a ROS costmap around the robot to a costmap object.
 *
 * This automatically affixes the cost map grid to the location of the robot
 * in the ros costmap. Resolution is also carried across. The only configuration
 * necessary is to specify how large the cost map should be. Take care that this
 * subwindow does not go off the edge of the underlying ros costmap!
 *
 * @param ros_costmap : a traditional ros costmap object (input).
 * @param geometry : size of the subwindow (mxm), use 0x0 to get the whole costmap.
 * @return shared pointer to the cost map object
 *
 * @note We should, but cannot use a const for the ros costmap since it hasn't been very
 * well designed. Treat it as such and do not change the internals inside.
 */
CostMapPtr fromROSCostMap2D(costmap_2d::Costmap2DROS& ros_costmap,
                            const cost_map::Length& geometry=cost_map::Length::Zero());

/**
 * @brief Copies all data from copied_cost_map to target_cost_map (have to be same size)
 *
 * The data will be put in a new layer called "obstacle_costs"
 *
 * @param copied_cost_map : a traditional ros Costmap2D object.
 * @param target_cost_map : Ptr to target cost_map.
 *
 * @note We should, but cannot use a const for the ros Costmap2D since it hasn't been very
 * well designed. Treat it as such and do not change the internals inside.
 */
void copyCostmap2DData(costmap_2d::Costmap2D& copied_cost_map, const CostMapPtr& target_cost_map);

void toOccupancyGrid(const cost_map::CostMap& cost_map, const std::string& layer, nav_msgs::OccupancyGrid& msg);

/**
 * @brief Provide cost_map::fromROSCostMap2D() as a ros service.
 */
class ROSCostMap2DServiceProvider {
public:
  ROSCostMap2DServiceProvider(costmap_2d::Costmap2DROS* ros_costmap,
                              const std::string& service_name="get_cost_map");

  bool callback(cost_map_msgs::GetCostMap::Request  &req,
                cost_map_msgs::GetCostMap::Response &res);
private:
  costmap_2d::Costmap2DROS* ros_costmap;
  ros::ServiceServer service;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_ros_CONVERTER_HPP_ */
