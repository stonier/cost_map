/**
 * @file /cost_map/include/cost_map/loader.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_LOADER_HPP_
#define cost_map_LOADER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core/cost_map_core.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

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
   * @brief Load from an image resource name (package_name/image_yaml_config).
   *
   * Not currently supporting any other method (e.g. loading directly from
   * a filename). If we do, split the loader up into multiple static methods.
   *
   * @param image_resource_name
   */
  Loader(const std::string& image_resource_name="cost_map_visualisations/example.yaml",
         const std::string& frame_id="map"
         );

  void imageResourceNameCallback(const std_msgs::String& msg);

  void publish();

  cost_map::CostMapPtr cost_map;
  ros::Publisher publisher;
  ros::Subscriber subscriber;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_LOADER_HPP_ */
