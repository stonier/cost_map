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

#include <grid_map/grid_map.hpp>
#include <cost_map_core.hpp>
#include <cost_map_msgs/CostMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Interfaces
*****************************************************************************/

CostMapPtr loadFromImageFile(const std::string& filename);

bool addLayerFromROSImage(const sensor_msgs::Image& image,
                          const std::string& layer,
                          cost_map::CostMap& cost_map
                          );

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

void toOccupancyGrid(const cost_map::CostMap& cost_map, const std::string& layer, nav_msgs::OccupancyGrid& msg);

grid_map::GridMap toGridMap(const cost_map::CostMap cost_map);

/*****************************************************************************
** MultiArray Message Helpers
*****************************************************************************/
/*
 * Note: These are template version of the specific Float32MultiArray
 * version in grid_map. Could be back-ported back easily.
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
