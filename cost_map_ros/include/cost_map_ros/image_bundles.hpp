/**
 * @file /cost_map_ros/include/cost_map_ros/loader.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_ros_LOADER_HPP_
#define cost_map_ros_LOADER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core/cost_map_core.hpp>
#include <cost_map_msgs/CostMap.h>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Methods
*****************************************************************************/

// to/from converters are in converters.hpp/converters.cpp

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Helper for loading and publishing image bundles.
 *
 * Used by the save_image_bundle command line utility.
 */
class LoadImageBundle {
public:
  /**
   * @brief Load and publish from an image bundle.
   *
   * @param[in] image_bundle_location : package_name/yaml resource pair  (e.g. cost_map_visualisations/example.yaml), or simply yaml filename
   * @param[in] topic_name : where to publish the costmap (default: 'cost_map')
   */
  LoadImageBundle(const std::string& image_bundle_location,
                  const std::string& topic_name="cost_map");

  void publish();

  cost_map::CostMapPtr cost_map;
  ros::Publisher publisher;
};

/**
 * @brief Helper for saving an image bundle from a cost map topic.
 *
 * Used by the save_image_bundle command line utility.
 */
class SaveImageBundle {
public:
  /**
   * @brief Load and publish from an image bundle.
   *
   * @param[in] topic_name : topic to listen to for incoming cost maps
   * @param[in] yaml_filename : name of the image bundle meta yaml file (abs or relative path).
   */
  SaveImageBundle(const std::string& topic_name, const std::string& yaml_filename="foo.yaml");
  void costmapCallback(const cost_map_msgs::CostMap& msg);

  std::string yaml_filename;
  bool finished;

private:
  ros::Subscriber subscriber_;
  std::mutex mutex_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_ros_LOADER_HPP_ */
