/**
 * @file /cost_map/include/cost_map/converter.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_CONVERTER_HPP_
#define cost_map_CONVERTER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// grid maps
#include <grid_map/grid_map.hpp>
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
** Images
*****************************************************************************/

/**
 * @brief Loads from a cost_map image yaml representative file.
 *
 * The specified format for this file is a yaml, which holds some specifications
 * for the cost_map (e.g. resolution) and points to an actual image file which
 * contains the cost_map data.
 *
 * @param filename : yaml file
 * @return shared pointer to the cost map object
 */
CostMapPtr fromImageResource(const std::string& filename);
/**
 * @brief Save to cost_map image yaml representative file(s).
 *
 * The specified format for this file is a yaml, which holds some specifications
 * for the cost_map (e.g. resolution) and points to an actual image file which
 * contains the cost_map data.
 *
 * One yaml and its associated image is saved for each layer in the cost map.
 *
 * @param cost_map : the cost map to save
 */
void toImageResource(const cost_map::CostMap& cost_map);

bool addLayerFromROSImage(const sensor_msgs::Image& image,
                          const std::string& layer,
                          cost_map::CostMap& cost_map
                          );

/*****************************************************************************
** CostMap and GridMap
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

grid_map::GridMap toGridMap(const cost_map::CostMap cost_map);

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
 * @param geometry : size of the subwindow (metres x metres).
 * @return shared pointer to the cost map object
 *
 * @note We should, but cannot use a const for the ros costmap since it hasn't been very
 * well designed. Treat it as such and do not change the internals inside.
 */
CostMapPtr fromROSCostMap2D(costmap_2d::Costmap2DROS& ros_costmap, cost_map::Length& geometry);

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
** MultiArray Message Helpers
*****************************************************************************/
/*
 * Note: These are template version of the specific Float32MultiArray
 * version in grid_map. Could be back-ported to grid_map easily.
 */
/*!
 * Checks if message data is stored in row-major format.
 *
 * @param[in] messageData the message data.
 * @return true if is in row-major format, false if is in column-major format.
 */
template<typename MultiArrayMessageType_>
bool isRowMajor(const MultiArrayMessageType_& messageData) {
  if (messageData.layout.dim[0].label == grid_map::storageIndexNames[grid_map::StorageIndices::Column]) return false;
  else if (messageData.layout.dim[0].label == grid_map::storageIndexNames[grid_map::StorageIndices::Row]) return true;
  //ROS_ERROR("isRowMajor() failed because layout label is not set correctly.");
  return false;
}

template<typename MultiArrayMessageType_>
unsigned int getCols(const MultiArrayMessageType_& messageData)
{
  if (isRowMajor(messageData)) return messageData.layout.dim.at(1).size;
  return messageData.layout.dim.at(0).size;
}

template<typename MultiArrayMessageType_>
unsigned int getRows(const MultiArrayMessageType_& messageData)
{
  if (isRowMajor(messageData)) return messageData.layout.dim.at(0).size;
  return messageData.layout.dim.at(1).size;
}

/**
 * Template version of the specific Float32MultiArray version in grid_map.
 *
 * @param[in] m the ROS message to which the data will be copied.
 * @param[out] e the Eigen matrix to be converted.
 * @return true if successful
 * @return
 */
template<typename EigenType_, typename MessageType_>
bool multiArrayMessageCopyToMatrixEigen(const MessageType_& m, EigenType_& e)
{
  if (e.IsRowMajor != isRowMajor(m))
  {
    //ROS_ERROR("multiArrayMessageToMatrixEigen() failed because the storage order is not compatible.");
    return false;
  }

  EigenType_ tempE(getRows(m), getCols(m));
  tempE = Eigen::Map<const EigenType_>(m.data.data(), getRows(m), getCols(m));
  e = tempE;
  return true;
}

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_CONVERTER_HPP_ */
