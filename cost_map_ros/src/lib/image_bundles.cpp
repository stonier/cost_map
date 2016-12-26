/**
 * @file /cost_map_ros/src/lib/loader.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <boost/filesystem.hpp>
#include <cost_map_msgs/CostMap.h>
#include <cv_bridge/cv_bridge.h>
#include <ecl/console.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "../../include/cost_map_ros/converter.hpp"
#include "../../include/cost_map_ros/image_bundles.hpp"
#include "../../include/cost_map_ros/utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** ImageBundle Converters
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
        throw std::logic_error("missing required value '" + property + "'");
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
          throw std::logic_error("missing required value 'layer_name'");
        }
        if (!layer["layer_data"]) {
          throw std::logic_error("missing required value 'layer_data'");
        }
        layers.insert(std::pair<std::string, std::string>(
            layer["layer_name"].as<std::string>(),
            layer["layer_data"].as<std::string>()
            ));
      }
    }
  } catch(const YAML::ParserException& e ) {
    throw std::logic_error("failed to parse (bad) yaml '" + filename + "'");
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

/*****************************************************************************
** Class Implementations
*****************************************************************************/

LoadImageBundle::LoadImageBundle(
    const std::string& image_bundle_location,
    const std::string& topic_name)
{
  ros::NodeHandle nodehandle("~");
  publisher = nodehandle.advertise<cost_map_msgs::CostMap>(topic_name, 1, true);

  std::string yaml_filename;
  if ( boost::filesystem::exists(image_bundle_location)) {
    yaml_filename = image_bundle_location;
  } else {
    // fallback to check if it's a ros resource
    yaml_filename = cost_map::resolveResourceName(image_bundle_location);
  }
  cost_map = std::make_shared<CostMap>();
  cost_map::fromImageBundle(yaml_filename, *cost_map);
  publish();
  // for debugging, verify this function returns what we loaded.
  // toImageBundle("debug.yaml", *cost_map);
}

void LoadImageBundle::publish() {
  cost_map_msgs::CostMap map_message;
  cost_map::toMessage(*cost_map, map_message);
  publisher.publish(map_message);
}

SaveImageBundle::SaveImageBundle(
    const std::string& topic_name,
    const std::string& yaml_filename)
: finished(false)
, yaml_filename(yaml_filename)
{
  ros::NodeHandle nodehandle("~");
  // TODO : check that a publisher exists and warn if not available?
  subscriber_ = nodehandle.subscribe(topic_name, 1, &SaveImageBundle::_costmapCallback, this);
}

void SaveImageBundle::_costmapCallback(const cost_map_msgs::CostMap& msg) {
  std::lock_guard<std::mutex> guard(mutex_);
  if ( !finished ) {
    cost_map::CostMap cost_map;
    if ( !fromMessage(msg, cost_map) ) {
      ROS_ERROR_STREAM("SaveImageBundle : failed to convert cost map msg -> cost map class");
      return;
    }
    toImageBundle(yaml_filename, cost_map);
    ROS_INFO_STREAM("SaveImageBundle : successfully saved to '" << yaml_filename << "'");
    finished = true;
  }
}



/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
