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


Loader::Loader(ros::NodeHandle& nodehandle)
: publisher(nodehandle.advertise<cost_map_msgs::CostMap>("cost_map", 1, true))
, subscriber(nodehandle.subscribe("resource_name", 1, &Loader::imageResourceNameCallback, this))
{
  std::string default_resource_name;
  nodehandle.param<std::string>("resource_name", default_resource_name, "cost_map_visualisations/example.yaml");
  std::string yaml_filename = cost_map::resolveResourceName(default_resource_name);

  cost_map = cost_map::loadFromImageFile(yaml_filename);
  publish();
}

void Loader::imageResourceNameCallback(const std_msgs::String& msg) {
  //std::string filename = resolveResourceName(msg.data);
}

void Loader::publish() {
  cost_map_msgs::CostMap map_message;
  cost_map::toMessage(cost_map, map_message);
  publisher.publish(map_message);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
