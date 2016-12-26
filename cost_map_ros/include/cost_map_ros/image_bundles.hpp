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

/**
 * @brief Initialises a adds a single layer from a yaml/image resource pair.
 *
 * @warning this will change the geometry of the provided costmap and delete all layers!
 * @todo bool result/exception handling for when things go wrong
 *
 * @param[in] filename : yaml file
 * @param[out] cost_map :
 *
 *@throw std::logic_error if the yaml couldn't be read, or the required yaml was not valid
 */
void fromImageBundle(const std::string& filename, cost_map::CostMap& cost_map);
/**
 * @brief Dump a cost map to an image bundle set of files.
 *
 * This creates the specified yaml file with image bundle meta information
 * and a set of png images alongside, one for each layer in the cost map.
 *
 * @todo bool result/exception handling for when things go wrong
 *
 * @param[in] filename : name of the yaml file to write
 * @param[in] cost_map : cost map to dump
 */
void toImageBundle(const std::string& filename, const cost_map::CostMap& cost_map);

// this might be an interesting api to have too
// void addLayerFromImageFile()
// and this one, which is similar to api in grid_map and inputs opencv types directly
// void addLayerFromImage()

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
  virtual ~LoadImageBundle() {}

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
  virtual ~SaveImageBundle() {}


  std::string yaml_filename;
  bool finished;

private:
  void _costmapCallback(const cost_map_msgs::CostMap& msg);

  ros::Subscriber subscriber_;
  std::mutex mutex_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_ros_LOADER_HPP_ */
