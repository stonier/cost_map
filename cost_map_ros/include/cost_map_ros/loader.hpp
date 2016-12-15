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
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class Loader {
public:
  /**
   * @brief Load and publish from an image bundle.
   *
   * @param[in] image_bundle_location : package_name/yaml resource pair  (e.g. cost_map_visualisations/example.yaml), or simply yaml filename
   * @param[in] topic_name : where to publish the costmap (default: 'cost_map')
   */
  Loader(const std::string& image_bundle_location,
         const std::string& topic_name="cost_map");

  void publish();

  cost_map::CostMapPtr cost_map;
  ros::Publisher publisher;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_ros_LOADER_HPP_ */
