/**
 * @file /cost_map/src/lib/loader.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_msgs/CostMap.h>
#include "../../include/cost_map/converter.hpp"
#include "../../include/cost_map/loader.hpp"
#include "../../include/cost_map/utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Implementation
*****************************************************************************/


Loader::Loader(const std::string& image_bundle_resource_name)
{
  ros::NodeHandle nodehandle("~");

  publisher = nodehandle.advertise<cost_map_msgs::CostMap>("cost_map", 1, true);
  subscriber = nodehandle.subscribe("resource_name", 1, &Loader::imageResourceNameCallback, this);

  std::string yaml_filename = cost_map::resolveResourceName(image_bundle_resource_name);
  cost_map = std::make_shared<CostMap>();
  cost_map::fromImageBundle(yaml_filename, *cost_map);
  publish();
  // for debugging, verify this function returns what we loaded.
  // toImageBundle("debug.yaml", *cost_map);
}

void Loader::imageResourceNameCallback(const std_msgs::String& msg) {
  //std::string filename = resolveResourceName(msg.data);
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
