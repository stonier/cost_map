/**
 * @file /cost_map_ros/src/lib/loader.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_msgs/CostMap.h>
#include "../../include/cost_map_ros/converter.hpp"
#include "../../include/cost_map_ros/image_bundles.hpp"
#include "../../include/cost_map_ros/utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

LoadImageBundle::LoadImageBundle(
    const std::string& image_bundle_location,
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
  subscriber_ = nodehandle.subscribe(topic_name, 1, &SaveImageBundle::costmapCallback, this);
}

void SaveImageBundle::costmapCallback(const cost_map_msgs::CostMap& msg) {
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
