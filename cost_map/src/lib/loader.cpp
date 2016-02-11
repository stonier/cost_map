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


Loader::Loader(const std::string& image_resource_name,
               const std::string& frame_id)
{
  ros::NodeHandle nodehandle("~");

  publisher = nodehandle.advertise<cost_map_msgs::CostMap>("cost_map", 1, true);
  subscriber = nodehandle.subscribe("resource_name", 1, &Loader::imageResourceNameCallback, this);

  std::string yaml_filename = cost_map::resolveResourceName(image_resource_name);
  cost_map = cost_map::loadFromImageFile(yaml_filename);
  cost_map->setFrameId(frame_id);
  publish();
  // for debugging, verify this function returns what we loaded.
  // saveToImageFile(*cost_map);
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
