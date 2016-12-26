/**
 * @file /cost_map_ros/src/lib/converter.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <boost/thread/lock_guard.hpp>
#include <cmath>
#include <cost_map_core/cost_map_core.hpp>
#include <cost_map_msgs/CostMap.h>
#include <ecl/console.hpp>
#include <limits>
#include <map>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8MultiArray.h>
#include <stdexcept>
#include <string>
#include <tf/tf.h>
#include "../../include/cost_map_ros/converter.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** ROS Images
*****************************************************************************/

//
// almost a carbon copy of grid_map::addLayerFromImage
//
bool addLayerFromROSImage(const sensor_msgs::Image& image,
                          const std::string& layer_name,
                          cost_map::CostMap& cost_map
                          )
{
  cv_bridge::CvImagePtr cvPtrAlpha, cvPtrMono;

  // If alpha channel exist, read it.
  if (image.encoding == sensor_msgs::image_encodings::BGRA8
      || image.encoding == sensor_msgs::image_encodings::BGRA16) {
    try {
      cvPtrAlpha = cv_bridge::toCvCopy(image, image.encoding);
    } catch (cv_bridge::Exception& e) {
      std::cout << ecl::red << "[ERROR] cv_bridge exception: " << e.what() << ecl::reset << std::endl;
      return false;
    }
  }

  unsigned int depth;  // Convert color image to grayscale.
  try {
    if (image.encoding == sensor_msgs::image_encodings::BGRA8
        || image.encoding == sensor_msgs::image_encodings::BGR8
        || image.encoding == sensor_msgs::image_encodings::MONO8) {
      cvPtrMono = cv_bridge::toCvCopy(image,
                                      sensor_msgs::image_encodings::MONO8);
      depth = std::pow(2, 8);
      // std::cout << "Color image converted to mono8" << std::endl;
    } else if (image.encoding == sensor_msgs::image_encodings::BGRA16
        || image.encoding == sensor_msgs::image_encodings::BGR16
        || image.encoding == sensor_msgs::image_encodings::MONO16) {
      cvPtrMono = cv_bridge::toCvCopy(image,
                                      sensor_msgs::image_encodings::MONO16);
      depth = std::pow(2, 16);
      // std::cout << "Color image converted to mono16" << std::endl;
    } else {
      std::cout << ecl::red << "[ERROR] expected BGR, BGRA, or MONO image encoding." << ecl::reset << std::endl;
      return false;
    }
  } catch (cv_bridge::Exception& e) {
    std::cout << ecl::red << "[ERROR] cv_bridge exception: " << e.what() << ecl::reset << std::endl;
    return false;
  }

  cost_map.add(layer_name);

  if (cost_map.getSize()(0) != image.height
      || cost_map.getSize()(1) != image.width) {
    std::cout << ecl::red << "[ERROR] Image size does not correspond to grid map size!" << ecl::reset << std::endl;
    return false;
  }

  for (CostMapIterator iterator(cost_map); !iterator.isPastEnd(); ++iterator) {
    // skip transparent values (they will typically be set to the default (cv_map::NO_INFORMATION/255)
    if (image.encoding == sensor_msgs::image_encodings::BGRA8) {
      const auto& cvAlpha = cvPtrAlpha->image.at<cv::Vec4b>((*iterator)(0),
                                                            (*iterator)(1));
      unsigned int alpha = cvAlpha[3];
      if (cvAlpha[3] < depth / 2)
        continue;
    }
    if (image.encoding == sensor_msgs::image_encodings::BGRA16) {
      const auto& cvAlpha = cvPtrAlpha->image.at<cv::Vec<uchar, 8>>(
          (*iterator)(0), (*iterator)(1));
      int alpha = (cvAlpha[6] << 8) + cvAlpha[7];
      if (alpha < depth / 2)
        continue;
    }

    // Compute non-transparent values.
    unsigned int grayValue;
    if (depth == std::pow(2, 8)) {
      uchar cvGrayscale = cvPtrMono->image.at<uchar>((*iterator)(0),
                                                     (*iterator)(1));
      grayValue = cvGrayscale;
    }
    if (depth == std::pow(2, 16)) {
      const auto& cvGrayscale = cvPtrMono->image.at<cv::Vec2b>((*iterator)(0),
                                                               (*iterator)(1));
      grayValue = (cvGrayscale[0] << 8) + cvGrayscale[1];
    }
    // RULE 1 : scale only from 0-254 (remember 255 is reserved for NO_INFORMATION)
    // RULE 2 : invert the value as black on an image (grayscale: 0) typically represents an obstacle (cost: 254)
    cost_map::DataType value = static_cast<cost_map::DataType>(254.0 * ( 1.0 - (static_cast<double>(grayValue) / static_cast<double>(depth))));
    cost_map.at(layer_name, *iterator) = value;
  }
  return true;
}

/*****************************************************************************
** CostMap and GridMap
*****************************************************************************/


grid_map::GridMap toGridMap(const cost_map::CostMap cost_map)
{
  grid_map::GridMap grid_map;
  grid_map.setGeometry(cost_map.getLength(), cost_map.getResolution(), cost_map.getPosition());
  grid_map.setFrameId(cost_map.getFrameId());
  grid_map.setTimestamp(cost_map.getTimestamp());
  for (const std::string& layer_name : cost_map.getLayers()) {
    const cost_map::Matrix& cost_map_data = cost_map[layer_name];
    grid_map::Matrix grid_map_data(cost_map_data.rows(), cost_map_data.cols());
    const cost_map::DataType *cost_map_ptr = cost_map_data.data();
    float *grid_map_ptr = grid_map_data.data();
    for (unsigned int i = 0; i < cost_map_data.size(); ++i ) {
      *grid_map_ptr = 100.0 * static_cast<double>(*cost_map_ptr) / static_cast<double>(cost_map::NO_INFORMATION);
      ++cost_map_ptr; ++grid_map_ptr;
    }
    grid_map.add(layer_name, grid_map_data);
  }
  return grid_map;
}

/*****************************************************************************
** Messages
*****************************************************************************/
//
// almost a carbon copy of grid_map::toMessage
//
void toMessage(const cost_map::CostMap& cost_map, cost_map_msgs::CostMap& message)
{
  std::vector<std::string> layers = cost_map.getLayers();

  message.info.header.stamp.fromNSec(cost_map.getTimestamp());
  message.info.header.frame_id = cost_map.getFrameId();
  message.info.resolution = cost_map.getResolution();
  message.info.length_x = cost_map.getLength().x();
  message.info.length_y = cost_map.getLength().y();
  message.info.pose.position.x = cost_map.getPosition().x();
  message.info.pose.position.y = cost_map.getPosition().y();
  message.info.pose.position.z = 0.0;
  message.info.pose.orientation.x = 0.0;
  message.info.pose.orientation.y = 0.0;
  message.info.pose.orientation.z = 0.0;
  message.info.pose.orientation.w = 1.0;

  message.layers = layers;
  message.basic_layers = cost_map.getBasicLayers();

  message.data.clear();
  for (const auto& layer : layers) {
    const cost_map::Matrix& data = cost_map.get(layer);
    std_msgs::UInt8MultiArray data_array;
    grid_map::matrixEigenCopyToMultiArrayMessage(cost_map.get(layer), data_array);
    message.data.push_back(data_array);
  }

  message.outer_start_index = cost_map.getStartIndex()(0);
  message.inner_start_index = cost_map.getStartIndex()(1);
}

bool fromMessage(const cost_map_msgs::CostMap& message, cost_map::CostMap& cost_map)
{
  cost_map.setTimestamp(message.info.header.stamp.toNSec());
  cost_map.setFrameId(message.info.header.frame_id);
  cost_map.setGeometry(Length(message.info.length_x, message.info.length_y), message.info.resolution,
                      Position(message.info.pose.position.x, message.info.pose.position.y));

  if (message.layers.size() != message.data.size()) {
    // ROS_ERROR("Different number of layers and data in grid map message.");
    return false;
  }

  for (unsigned int i = 0; i < message.layers.size(); i++) {
    Matrix data;
    // this is not a template function
    // grid_map::multiArrayMessageCopyToMatrixEigen(message.data[i], data); // TODO Could we use the data mapping (instead of copying) method here?
    grid_map::multiArrayMessageCopyToMatrixEigen(message.data[i], data);
    // TODO Check if size is good.   size_ << getRows(message.data[0]), getCols(message.data[0]);
    cost_map.add(message.layers[i], data);
  }

  cost_map.setBasicLayers(message.basic_layers);
  cost_map.setStartIndex(Index(message.outer_start_index, message.inner_start_index));
  return true;
}

/*****************************************************************************
** CostMap2DROS and Occupancy Grids
*****************************************************************************/

void fromCostMap2DROS(costmap_2d::Costmap2DROS& ros_costmap,
                      const std::string& layer_name,
                      cost_map::CostMap& cost_map) {
  // preparation
  double resolution = ros_costmap.getCostmap()->getResolution();
  double original_size_x = ros_costmap.getCostmap()->getSizeInCellsX() * resolution;
  double original_size_y = ros_costmap.getCostmap()->getSizeInCellsY() * resolution;
  cost_map::Length geometry(original_size_x, original_size_y);
  cost_map::Position ros_map_origin(ros_costmap.getCostmap()->getOriginX(), ros_costmap.getCostmap()->getOriginY());
  cost_map::Position new_cost_map_origin(ros_map_origin.x() + original_size_x/2, ros_map_origin.y() + original_size_y/2);

  // cost map meta information
  cost_map.setFrameId(ros_costmap.getGlobalFrameID());
  cost_map.setTimestamp(ros::Time::now().toNSec());
  cost_map.setGeometry(geometry, resolution, new_cost_map_origin);

  // data
  boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(ros_costmap.getCostmap()->getMutex()));
  addLayerFromCostMap2D(*(ros_costmap.getCostmap()), layer_name, cost_map);
}

void fromCostMap2DROS(costmap_2d::Costmap2DROS& ros_costmap,
                      const cost_map::Length& geometry,
                      const std::string& layer_name,
                      cost_map::CostMap& cost_map)
{
  /****************************************
  ** Properties
  ****************************************/

  double resolution = ros_costmap.getCostmap()->getResolution();
  // Note: don't use getSizeInMeters() here - it doesn't give the actual grid size...for
  // some reason they designed it so that it cuts it short at the centre of the last cell
  //            e.g. 10 cells, resolution 1.0 -> getSizeInMeters() == 9.5
  double original_size_x = ros_costmap.getCostmap()->getSizeInCellsX() * resolution;
  double original_size_y = ros_costmap.getCostmap()->getSizeInCellsY() * resolution;

  /****************************************
  ** Redirect if actually a full window
  ****************************************/
  if (geometry.x() == 0.0 || geometry.y() == 0.0 ||
      ( geometry.x() == original_size_x && geometry.y() == original_size_y )
      ) {
    return fromCostMap2DROS(ros_costmap, layer_name, cost_map);
  }

  /****************************************
  ** Get the Robot Pose Transform
  ****************************************/
  tf::Stamped<tf::Pose> tf_pose;
  if(!ros_costmap.getRobotPose(tf_pose))
  {
    std::ostringstream error_message;
    error_message << "Could not get robot pose, not published?";
    throw std::runtime_error(error_message.str());
  }

  /****************************************
  ** Asserts
  ****************************************/
  // TODO check the robot pose is in the window

  /****************************************
  ** Where is the New Costmap Origin?
  ****************************************/
  cost_map::Position robot_position(tf_pose.getOrigin().x() , tf_pose.getOrigin().y());
  cost_map::Position ros_map_origin(ros_costmap.getCostmap()->getOriginX(), ros_costmap.getCostmap()->getOriginY());
  cost_map::Position new_cost_map_origin;

  // Note:
  //   You cannot directly use the robot pose as the new 'costmap centre'
  //   since the underlying grid is not necessarily exactly aligned with
  //   that (two cases to consider, rolling window and globally fixed).
  //
  // Relevant diagrams:
  //  - https://github.com/ethz-asl/grid_map

  // Have to do the exact same as the ros costmap updateOrigin to end up with same position
  // Don't use original_size_x here. getSizeInMeters is actually broken but the
  // ros costmap uses it to set the origin. So we also have to do it to get the
  // same behaviour. (See also updateOrigin call in LayeredCostmap)

  // float versions of the cell co-ordinates, use static_cast<int> to get the indices
  cost_map::Position robot_cell_position = (robot_position - ros_map_origin)/resolution;

  // if there is an odd number of cells
  //   centre of the new grid map in the centre of the current cell
  // if there is an even number of cells
  //   centre of the new grid map at the closest vertex between cells
  // of the current cell
  int number_of_cells_x = geometry.x()/resolution;
  int number_of_cells_y = geometry.y()/resolution;
  if ( number_of_cells_x % 2 ) { // odd
    new_cost_map_origin(0) = std::floor(robot_cell_position.x())*resolution + resolution/2.0 + ros_map_origin.x();
  } else {
    new_cost_map_origin(0) = std::round(robot_cell_position.x())*resolution + ros_map_origin.x();
  }
  if ( number_of_cells_y % 2 ) { // odd
    new_cost_map_origin(1) = std::floor(robot_cell_position.y())*resolution + resolution/2.0 + ros_map_origin.y();
  } else {
    new_cost_map_origin(1) = std::round(robot_cell_position.y())*resolution + ros_map_origin.y();
  }

  /****************************************
  ** Initialise the CostMap
  ****************************************/
  cost_map.setFrameId(ros_costmap.getGlobalFrameID());
  cost_map.setTimestamp(ros::Time::now().toNSec());
  cost_map.setGeometry(geometry, resolution, new_cost_map_origin);

  /****************************************
  ** Copy Data
  ****************************************/
  bool is_valid_window = false;
  costmap_2d::Costmap2D costmap_subwindow;
  {
    boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(ros_costmap.getCostmap()->getMutex()));

    is_valid_window = costmap_subwindow.copyCostmapWindow(
                            *(ros_costmap.getCostmap()),
                            new_cost_map_origin.x() - geometry.x() / 2.0, // subwindow_bottom_left_x
                            new_cost_map_origin.y() - geometry.y() / 2.0, // subwindow_bottom_left_y
                            geometry.x(),
                            geometry.y());
  }

  if ( !is_valid_window ) {
    // handle differently - e.g. copy the internal part only and lethal elsewhere, but other parts would have to handle being outside too
    std::ostringstream error_message;
    error_message << "Subwindow landed outside the costmap (max size: " << original_size_x << "x" << original_size_y
                  << "), aborting (you should ensure the robot travels inside the costmap bounds).";
    throw std::out_of_range(error_message.str());
  }

  addLayerFromCostMap2D(costmap_subwindow, layer_name, cost_map);
}

void addLayerFromCostMap2D(costmap_2d::Costmap2D& costmap_2d,
                           const std::string& layer_name,
                           CostMap& cost_map)
{
  if (costmap_2d.getSizeInCellsX() != cost_map.getSize().x()
      || costmap_2d.getSizeInCellsY() != cost_map.getSize().y()) {

    std::ostringstream error_message;
    error_message << "Tried to copy Costmap2D data (" << costmap_2d.getSizeInCellsX() << "x" << costmap_2d.getSizeInCellsY()
                  << ") to a differently sized cost_map (" << cost_map.getSize().x() << "x" << cost_map.getSize().y() << ")";
    throw std::invalid_argument(error_message.str());
  }

  unsigned char* subwindow_costs = costmap_2d.getCharMap();

  // remember there is a different convention for indexing.
  //  - costmap_2d::CostMap starts from the bottom left
  //  - cost_map::CostMap   starts from the top left
  cost_map::Matrix data(cost_map.getSize().x(), cost_map.getSize().y());

  unsigned int size = cost_map.getSize().x() * cost_map.getSize().y();
  for ( int i=0, index = size-1; index >= 0; --index, ++i) {
    data(i) = subwindow_costs[index];
  }
  cost_map.add(layer_name, data);
}

void toOccupancyGrid(const cost_map::CostMap& cost_map, const std::string& layer, nav_msgs::OccupancyGrid& msg) {
  msg.header.frame_id = cost_map.getFrameId();
  msg.header.stamp.fromNSec(cost_map.getTimestamp());
  msg.info.map_load_time = msg.header.stamp;  // Same as header stamp as we do not load the map.
  msg.info.resolution = cost_map.getResolution();
  msg.info.width = cost_map.getSize()(0);
  msg.info.height = cost_map.getSize()(1);

  // adjust for difference in center
  Position positionOfOrigin = cost_map.getPosition() - 0.5 * cost_map.getLength().matrix();
  msg.info.origin.position.x = positionOfOrigin.x();
  msg.info.origin.position.y = positionOfOrigin.y();
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(msg.info.width * msg.info.height);

  size_t nCells = cost_map.getSize().prod();
  msg.data.resize(nCells);

  // Occupancy probabilities are in the range [0,100].  Unknown is -1.
  const float cellMin = 0;
  const float cellMax = 98;
  const float cellRange = cellMax - cellMin;

  const float data_minimum = 0;
  const float data_maximum = 252;
  for (CostMapIterator iterator(cost_map); !iterator.isPastEnd(); ++iterator)
  {
    float value;
    if (cost_map.at(layer, *iterator) == cost_map::NO_INFORMATION) // 255
    {
      value = -1;
    }
    else if (cost_map.at(layer, *iterator) == cost_map::LETHAL_OBSTACLE) // 254
    {
      value = 100;
    }
    else if (cost_map.at(layer, *iterator) == cost_map::INSCRIBED_OBSTACLE) // 253
    {
      value = 99;
    }
    else
    {
      value = (cost_map.at(layer, *iterator) - data_minimum) / (data_maximum - data_minimum);
      value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
    }
    unsigned int index = grid_map::getLinearIndexFromIndex(*iterator, cost_map.getSize(), false);
    msg.data[nCells - index - 1] = value;
  }
}

ROSCostMap2DServiceProvider::ROSCostMap2DServiceProvider(costmap_2d::Costmap2DROS* ros_costmap,
                                                         const std::string& service_name)
: ros_costmap(ros_costmap)
{
  ros::NodeHandle private_nodehandle("~");
  service = private_nodehandle.advertiseService(service_name, &ROSCostMap2DServiceProvider::callback, this);
}

bool ROSCostMap2DServiceProvider::callback(
    cost_map_msgs::GetCostMap::Request  &request,
    cost_map_msgs::GetCostMap::Response &response)
{
  try {
    CostMap cost_map;
    cost_map::Length geometry;
    geometry << request.length_x, request.length_y;
    fromCostMap2DROS(*ros_costmap, geometry, "obstacle_costs", cost_map);
    toMessage(cost_map, response.map);
  } catch ( std::runtime_error &e) {
    ROS_ERROR_STREAM("CostMap Service : " << e.what());
    return false;
  } catch ( std::out_of_range &e) {
    ROS_ERROR_STREAM("CostMap Service : " << e.what());
    return false;
  }
  return true;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
