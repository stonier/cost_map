/**
 * @file /cost_map_ros/src/lib/loader.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_msgs/CostMap.h>
#include "../../include/cost_map_ros/converter.hpp"
#include "../../include/cost_map_ros/loader.hpp"
#include "../../include/cost_map_ros/utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

Loader::Loader(const std::string& image_bundle_location,
               const std::string& topic_name)
{
  ros::NodeHandle nodehandle("~");
  publisher = nodehandle.advertise<cost_map_msgs::CostMap>(topic_name, 1, true);

  std::string yaml_filename = cost_map::resolveResourceName(image_bundle_location);
  cost_map = std::make_shared<CostMap>();
  cost_map::fromImageBundle(yaml_filename, *cost_map);
  publish();
  // for debugging, verify this function returns what we loaded.
  // toImageBundle("debug.yaml", *cost_map);
}

void Loader::publish() {
  cost_map_msgs::CostMap map_message;
  cost_map::toMessage(*cost_map, map_message);
  publisher.publish(map_message);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
