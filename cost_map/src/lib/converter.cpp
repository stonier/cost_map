/**
 * @file /cost_map/src/lib/converter.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <boost/thread/lock_guard.hpp>
#include <cost_map_core/cost_map_core.hpp>
#include <cost_map_msgs/CostMap.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <ecl/console.hpp>
#include <ecl/exceptions.hpp>
#include <fstream>
#include <limits>
#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8MultiArray.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include <tf/tf.h>
#include "../../include/cost_map/converter.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Images
*****************************************************************************/

void fromImageBundle(const std::string& filename, cost_map::CostMap& cost_map)
{
  /****************************************
  ** Load Meta Information from Yaml
  ****************************************/
  std::string frame_id;
  cost_map::Position centre;
  float resolution;
  unsigned int number_of_cells_x, number_of_cells_y;
  std::map<std::string, std::string> layers;
  try {
    YAML::Node config = YAML::LoadFile(filename);
    for (const std::string& property : {
        "frame_id", "centre_x", "centre_y",
        "resolution", "number_of_cells_x", "number_of_cells_y"} ) {
      if (!config[property]) {
        throw ecl::StandardException(LOC, ecl::ConfigurationError, "missing required value '" + property + "'");
      }
    }
    frame_id = config["frame_id"].as<std::string>();
    resolution = config["resolution"].as<float>();
    number_of_cells_x = config["number_of_cells_x"].as<unsigned int>();
    number_of_cells_y = config["number_of_cells_y"].as<unsigned int>();
    centre << config["centre_x"].as<double>(), config["centre_y"].as<double>();
    if (config["layers"]) {
      for ( unsigned int index = 0; index < config["layers"].size(); ++index ) {
        const YAML::Node& layer = config["layers"][index];
        if (!layer["layer_name"]) {
          throw ecl::StandardException(LOC, ecl::ConfigurationError, "missing required value 'layer_name'");
        }
        if (!layer["layer_data"]) {
          throw ecl::StandardException(LOC, ecl::ConfigurationError, "missing required value 'layer_data'");
        }
        layers.insert(std::pair<std::string, std::string>(
            layer["layer_name"].as<std::string>(),
            layer["layer_data"].as<std::string>()
            ));
      }
    }
  } catch(const YAML::ParserException& e ) {
    throw ecl::StandardException(LOC, ecl::InvalidObjectError, "failed to parse (bad) yaml '" + filename + "'");
  }
  /****************************************
  ** Initialise the Cost Map
  ****************************************/
  Length length(resolution*number_of_cells_x, resolution*number_of_cells_y);
  cost_map.setGeometry(length, resolution, centre);
  cost_map.setFrameId(frame_id);
  cost_map.resetTimestamp();
  /****************************************
  ** Add Layers from Image Data
  ****************************************/
  for ( const auto& p : layers ) {
    // until c++11 filesystem stuff is more convenient.
    boost::filesystem::path parent_path = boost::filesystem::path(filename).parent_path();
    const std::string& layer_name = p.first;
    const std::string& image_relative_filename = p.second;
    boost::filesystem::path image_absolute_filename = parent_path / image_relative_filename;
    /********************
    ** Load OpenCV Image
    ********************/
    cv::Mat image = cv::imread(image_absolute_filename.string(), cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED, cv::IMREAD_COLOR, cv::IMREAD_GRAYSCALE, CV_LOAD_IMAGE_COLOR, CV_LOAD_IMAGE_GRAYSCALE
    // TODO check image.depth() for number of bits
    std::string encoding;
    switch( image.channels() ) {
      case 3  : encoding = "bgr8"; break;
      case 4  : encoding = "bgra8"; break;
      default : encoding = "mono8"; break;
    }

    /********************
    ** To Ros Image
    ********************/
    // TODO figure out how to skip this step and convert directly
    sensor_msgs::ImagePtr ros_image_msg = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
    // TODO optionally set a frame id from the yaml
    ros_image_msg->header.frame_id = frame_id;
    //std::cout << "Ros Image Message: " << *ros_image_msg << std::endl;

    /********************
    ** To Cost Map
    ********************/
    // this converts to a grayscale value immediately
    addLayerFromROSImage(*ros_image_msg, layer_name, cost_map);

    /********************
    ** Debugging
    ********************/
    //
    //  std::cout << "Filename   : " << filename << std::endl;
    //  std::cout << "Resolution : " << resolution << std::endl;
    //  std::cout << "Image (Rel): " << image_relative_filename << std::endl;
    //  std::cout << "Image (Abs): " << image_absolute_filename.string() << std::endl;
    //
    // cv::namedWindow(image_relative_filename, cv::WINDOW_AUTOSIZE);
    // cv::imshow(image_relative_filename, image);
    // cv::waitKey(0);

  }
}

void toImageBundle(const std::string& filename, const cost_map::CostMap& cost_map) {
  /********************
  ** Yaml
  ********************/
  YAML::Node node;
  node["frame_id"] = cost_map.getFrameId();
  node["centre_x"] = cost_map.getPosition().x();
  node["centre_y"] = cost_map.getPosition().x();
  node["resolution"] = cost_map.getResolution();
  node["number_of_cells_x"] = cost_map.getSize()(0);
  node["number_of_cells_y"] = cost_map.getSize()(1);
  YAML::Node layers;
  for (const std::string& layer_name : cost_map.getLayers()) {
    YAML::Node layer;
    layer["layer_name"] = layer_name;
    layer["layer_data"] = layer_name + ".png";
    layers.push_back(layer);
  }
  node["layers"] = layers;
  std::ofstream ofs(filename, std::ofstream::out);
  ofs << node;
  ofs.close();
  /********************
  ** Layers
  ********************/
  for (const std::string& layer : cost_map.getLayers()) {
    // can't take a const Matrix& here, since opencv will complain that it doesn't have control of
    // the memory (i.e. const void* cannot convert to void*)
    const cost_map::Matrix& cost_map_storage = cost_map.get(layer);
    // cv::Mat image(cost_map_storage.rows(), cost_map_storage.cols(), CV_8U, cost_map_storage.data());
    cv::Mat image(cost_map.getSize().x(), cost_map.getSize().y(), CV_8UC4);
    for (int i = 0; i < cost_map_storage.rows(); ++i) {
      for (int j = 0; j < cost_map_storage.cols(); ++j) {
        cv::Vec4b& rgba = image.at<cv::Vec4b>(i, j);
        cost_map::DataType value = cost_map_storage(i,j);
        // RULE 1 : scale only from 0-254 (remember 255 is reserved for NO_INFORMATION)
        // RULE 2 : invert the value as black on an image (grayscale: 0) typically represents an obstacle (cost: 254)
        cost_map::DataType flipped_value = static_cast<cost_map::DataType>(std::numeric_limits<cost_map::DataType>::max() * (1.0 - static_cast<double>(value) / static_cast<double>(cost_map::NO_INFORMATION)));
        rgba[0] = flipped_value;
        rgba[1] = flipped_value;
        rgba[2] = flipped_value;
        rgba[3] = (value == cost_map::NO_INFORMATION) ? 0.0 : std::numeric_limits<cost_map::DataType>::max();
      }
    }
    boost::filesystem::path parent_path = boost::filesystem::path(filename).parent_path();
    boost::filesystem::path image_absolute_filename = parent_path / (layer + std::string(".png"));
    cv::imwrite(image_absolute_filename.string(), image);
  }
}

//
// almost a carbon copy of grid_map::addLayerFromImage
//
bool addLayerFromROSImage(const sensor_msgs::Image& image,
                          const std::string& layer,
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

  cost_map.add(layer);

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
    cost_map.at(layer, *iterator) = value;
  }
  return true;
}

/*****************************************************************************
** CostMap and GridMap
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

grid_map::GridMap toGridMap(const cost_map::CostMap cost_map)
{
  grid_map::GridMap grid_map;
  grid_map.setGeometry(cost_map.getLength(), cost_map.getResolution(), cost_map.getPosition());
  grid_map.setFrameId(cost_map.getFrameId());
  grid_map.setTimestamp(cost_map.getTimestamp());
  // TODO fill in the data fields
  return grid_map;
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

CostMapPtr fromROSCostMap2D(costmap_2d::Costmap2DROS& ros_costmap, const cost_map::Length& geometry) {
  CostMapPtr cost_map = std::make_shared<CostMap>();
  costmap_2d::Costmap2D costmap_subwindow;
  cost_map::Length geometry_ = geometry;

  double resolution = ros_costmap.getCostmap()->getResolution();
  double original_size_x = ros_costmap.getCostmap()->getSizeInCellsX() * resolution;
  double original_size_y = ros_costmap.getCostmap()->getSizeInCellsY() * resolution;

  tf::Stamped<tf::Pose> tf_pose;
  if(!ros_costmap.getRobotPose(tf_pose))
  {
    std::ostringstream error_message;
    error_message << "Could not get robot pose, not published?";
    throw ecl::StandardException(LOC, ecl::OutOfRangeError, error_message.str());
  }

  cost_map::Position robot_position(tf_pose.getOrigin().x() , tf_pose.getOrigin().y());
  cost_map::Position ros_map_origin(ros_costmap.getCostmap()->getOriginX(), ros_costmap.getCostmap()->getOriginY());

  if (geometry_.x() == 0.0 || geometry_.y() == 0.0 ) {
    geometry_ << original_size_x, original_size_y;
  }
  bool full_size_requested = geometry_.x() == original_size_x && geometry_.y() == original_size_y;

  /****************************************
  ** Where is the New Costmap Origin?
  ****************************************/
  cost_map::Position new_cost_map_origin;

  if ( full_size_requested ) {
    new_cost_map_origin <<
        ros_map_origin.x() + original_size_x/2,
        ros_map_origin.y() + original_size_y/2;
  } else {
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
    double fake_origin_x = robot_position.x() - ros_costmap.getCostmap()->getSizeInMetersX() / 2;
    double fake_origin_y = robot_position.y() - ros_costmap.getCostmap()->getSizeInMetersY() / 2;

    int fake_origin_cell_x, fake_origin_cell_y;
    fake_origin_cell_x = int((fake_origin_x - ros_map_origin.x()) / resolution);
    fake_origin_cell_y = int((fake_origin_y - ros_map_origin.y()) / resolution);

    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double fake_origin_aligned_x, fake_origin_aligned_y;
    fake_origin_aligned_x = ros_map_origin.x() + fake_origin_cell_x * resolution;
    fake_origin_aligned_y = ros_map_origin.y() + fake_origin_cell_y * resolution;

    new_cost_map_origin <<
        fake_origin_aligned_x + original_size_x / 2,
        fake_origin_aligned_y + original_size_y / 2;
  }

  /****************************************
  ** Initialise the CostMap
  ****************************************/
  cost_map->setFrameId(ros_costmap.getGlobalFrameID());
  cost_map->setTimestamp(ros::Time::now().toNSec());
  cost_map->setGeometry(geometry_, resolution, new_cost_map_origin);

  /****************************************
  ** Copy Data
  ****************************************/
  if( full_size_requested ) {
    boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(ros_costmap.getCostmap()->getMutex()));
    copyCostmap2DData(*(ros_costmap.getCostmap()), cost_map);
    return cost_map;
  }

  double subwindow_bottom_left_x = new_cost_map_origin.x() - geometry_.x() / 2.0;
  double subwindow_bottom_left_y = new_cost_map_origin.y() - geometry_.y() / 2.0;

  double resolution_offset_x = std::abs(std::fmod(subwindow_bottom_left_x, resolution));
  double resolution_offset_y = std::abs(std::fmod(subwindow_bottom_left_y, resolution));

  // The way the conversion of world to map coordinate is done in costmap_2d is:
  //    mx = (int)((wx - origin_x_) / resolution_);
  // Because of numeric inaccuracy with the division we can end up with something too low
  // So we add a buffer. The buffer has have the same sign because of the used int cast
  // which is not rounding, but just cutting off
  double numeric_inaccuracy_fix = 0.5 * resolution;
  subwindow_bottom_left_x += std::copysign(numeric_inaccuracy_fix - resolution_offset_x, subwindow_bottom_left_x);
  subwindow_bottom_left_y += std::copysign(numeric_inaccuracy_fix - resolution_offset_y, subwindow_bottom_left_y);

  //debug
//  if ((robot_aligned - robot_position).norm() > 3 * resolution)
//  {
//    //something funny happened
//    std::cout << "[cost_map]: Got an apparently wrong position out of the cost_map conversion" << std::endl;
//    std::cout << "  Resolution        : " << resolution << std::endl;
//    std::cout << "  original size     : " << original_size_x << "x" << original_size_y << std::endl;
//    std::cout << "  Size              : " << geometry.x() << "x" << geometry.y() << std::endl;
//    std::cout << "  Robot Pose        : " << robot_position.x() << "," << robot_position.y() << std::endl;
//    std::cout << "  fake_origin       : " << fake_origin_x << "x" << fake_origin_y << std::endl;
//    std::cout << "  fake_origin_cell  : " << fake_origin_cell_x << "x" << fake_origin_cell_y << std::endl;
//    std::cout << "  fake_ aligned     : " << fake_origin_aligned_x << "x" << fake_origin_aligned_y << std::endl;
//    std::cout << "  robot_aligned     : " << robot_aligned_x << "x" << robot_aligned_y << std::endl;
//    std::cout << "  resolution_offset : " << resolution_offset_x << "x" << resolution_offset_y << std::endl;
//    std::cout << "  subwindow before  : " << new_cost_map_origin.x() - geometry.x() / 2.0 << "x" << new_cost_map_origin.y() - geometry.y() / 2.0 << std::endl;
//    std::cout << "  subwindow after   : " << subwindow_bottom_left_x << "x" << subwindow_bottom_left_y << std::endl;
//    std::cout << "  ros_map_origin    : " << ros_map_origin.x() << "x" << ros_map_origin.y() << std::endl;
//  }

  bool is_valid_window = false;
  // why do we need to lock - why don't they lock for us?
  {
    boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(ros_costmap.getCostmap()->getMutex()));

    is_valid_window = costmap_subwindow.copyCostmapWindow(
                            *(ros_costmap.getCostmap()),
                            subwindow_bottom_left_x, subwindow_bottom_left_y,
                            geometry_.x(),
                            geometry_.y());
  }

  if ( !is_valid_window ) {
    // handle differently - e.g. copy the internal part only and lethal elsewhere, but other parts would have to handle being outside too
    std::ostringstream error_message;
    error_message << "Subwindow landed outside the costmap (max size: " << original_size_x << "x" << original_size_y
                  << "), aborting (you should ensure the robot travels inside the costmap bounds).";
    throw ecl::StandardException(LOC, ecl::OutOfRangeError, error_message.str());
  }

  copyCostmap2DData(costmap_subwindow, cost_map);
  return cost_map;
}

void copyCostmap2DData(costmap_2d::Costmap2D& copied_cost_map, const CostMapPtr& target_cost_map)
{
  if (copied_cost_map.getSizeInCellsX() != target_cost_map->getSize().x()
      || copied_cost_map.getSizeInCellsY() != target_cost_map->getSize().y()) {

    std::ostringstream error_message;
    error_message << "Tried to copy Costmap2D data (" << copied_cost_map.getSizeInCellsX() << "x" << copied_cost_map.getSizeInCellsY()
                  << ") to a differently sized cost_map (" << target_cost_map->getSize().x() << "x" << target_cost_map->getSize().y() << ")";
    throw ecl::StandardException(LOC, ecl::OutOfRangeError, error_message.str());
  }

  unsigned char* subwindow_costs = copied_cost_map.getCharMap();

  // remember there is a different convention for indexing.
  //  - costmap_2d::CostMap starts from the bottom left
  //  - cost_map::CostMap   starts from the top left
  cost_map::Matrix data(target_cost_map->getSize().x(), target_cost_map->getSize().y());

  unsigned int size = target_cost_map->getSize().x() * target_cost_map->getSize().y();
  for ( int i=0, index = size-1; index >= 0; --index, ++i) {
    data(i) = subwindow_costs[index];
  }
  target_cost_map->add("obstacle_costs", data);
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
  CostMapPtr cost_map;
  try {
    cost_map::Length geometry;
    geometry << request.length_x, request.length_y;
    cost_map = fromROSCostMap2D(*ros_costmap, geometry);
    toMessage(*cost_map, response.map);
  } catch ( ecl::StandardException &e) {
    ROS_ERROR_STREAM("CostMap Service : " << e.what());
    return false;
  }
  return true;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
