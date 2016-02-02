/**
 * @file /cost_map/src/lib/converter.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core/cost_map_core.hpp>
#include <cost_map_msgs/CostMap.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <ecl/console.hpp>
#include <ecl/exceptions.hpp>
#include <limits>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8MultiArray.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include "../../include/cost_map/converter.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Private Implementation (to this file)
*****************************************************************************/

/*!
 * Initializes a cost map from a ros image message. This changes the geometry
 * of the map and deletes all contents of the layers!
 * @param[in] image the image.
 * @param[in] resolution the desired resolution of the grid map [m/cell].
 * @param[out] gridMap the grid map to be initialized.
 * @return true if successful, false otherwise.
 */
static bool initializeFromROSImage(
    const sensor_msgs::Image& image,
    const double resolution,
    cost_map::CostMap& cost_map,
    const cost_map::Position& position
)
{
  const double lengthX = resolution * image.height;
  const double lengthY = resolution * image.width;
  Length length(lengthX, lengthY);
  cost_map.setGeometry(length, resolution, position);
  cost_map.setFrameId(image.header.frame_id);
  cost_map.setTimestamp(image.header.stamp.toNSec());
  return true;
}

/*****************************************************************************
** Implementation
*****************************************************************************/

CostMap loadFromImageFile(const std::string& filename)
{
  /********************
  ** Load Yaml
  ********************/
  float resolution;
  std::string image_relative_filename;
  try {
    YAML::Node config = YAML::LoadFile(filename);
    if (!config["resolution"]) {
      throw ecl::StandardException(LOC, ecl::ConfigurationError, "missing required value 'resolution'");
    }
    if (!config["filename"]) {
      throw ecl::StandardException(LOC, ecl::ConfigurationError, "missing required value 'filename'");
    }
    resolution = config["resolution"].as<float>();
    image_relative_filename = config["filename"].as<std::string>();
  } catch(const YAML::ParserException& e ) {
    std::cout << "Error parsing the yaml: " << e.what() << std::endl;
  }
  // until c++11 filesystem stuff is more convenient.
  boost::filesystem::path p = boost::filesystem::path(filename).parent_path();
  p /= image_relative_filename;

  /********************
  ** Load OpenCV Image
  ********************/
  std::cout << "Filename   : " << filename << std::endl;
  std::cout << "Resolution : " << resolution << std::endl;
  std::cout << "Image (Rel): " << image_relative_filename << std::endl;
  std::cout << "Image (Abs): " << p << std::endl;

  cv::Mat image = cv::imread(p.string(), cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED, cv::IMREAD_COLOR, cv::IMREAD_GRAYSCALE, CV_LOAD_IMAGE_COLOR, CV_LOAD_IMAGE_GRAYSCALE

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
  ros_image_msg->header.frame_id = "map";
  //std::cout << "Ros Image Message: " << *ros_image_msg << std::endl;

  /********************
  ** To Cost Map
  ********************/
  CostMap cost_map;
  initializeFromROSImage(*ros_image_msg, resolution, cost_map, cost_map::Position::Zero());

  // grid_map::GridMapRosConverter::initializeFromImage(*ros_image_msg, 0.025, grid_map);

  // this converts to a grayscale value immediately
  addLayerFromROSImage(*ros_image_msg, "obstacle_cost", cost_map);
  //grid_map::GridMapRosConverter::addLayerFromImage(*ros_image_msg, "obstacle_cost", grid_map, min_height, max_height); // heights were 0.0, 1.0

  // debugging
  // cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Display window", image);
  // cv::waitKey(0);

  return cost_map;
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
    // Set transparent values.
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

    // Compute height.
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

    cost_map::DataType value = static_cast<cost_map::DataType>(255.0 * (static_cast<double>(grayValue) / static_cast<double>(depth)));
    cost_map.at(layer, *iterator) = value;
  }
  return true;
}

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
    for ( unsigned int i = 0; i < data.rows(); ++i ) {
      for ( unsigned int j = 0; j < data.cols(); ++j ) {
        std::cout << "  " << i << "," << j << ": " << static_cast<int>(data(i,j)) << std::endl;
      }
    }
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


/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
