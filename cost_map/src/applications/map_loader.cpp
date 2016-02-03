/**
 * @file src/applications/map_loader.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/console.hpp>
#include <ecl/exceptions.hpp>
#include <cost_map/cost_map.hpp>
#include <cost_map_msgs/CostMap.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include "../../include/cost_map/utilities.hpp"

/*****************************************************************************
** Methods
*****************************************************************************/

class Loader {
public:
  Loader(ros::NodeHandle& nodehandle)
  : publisher(nodehandle.advertise<cost_map_msgs::CostMap>("cost_map", 1, true))
  , subscriber(nodehandle.subscribe("resource_name", 1, &Loader::imageResourceNameCallback, this))
  {
    std::string default_resource_name;
    nodehandle.param<std::string>("resource_name", default_resource_name, "cost_map_visualisations/example.yaml");
    std::string yaml_filename = cost_map::resolveResourceName(default_resource_name);

    cost_map = cost_map::loadFromImageFile(yaml_filename);
    publish();
  }

  void imageResourceNameCallback(const std_msgs::String& msg) {
    //std::string filename = resolveResourceName(msg.data);
  }

  void publish() {
    cost_map_msgs::CostMap map_message;
    cost_map::toMessage(cost_map, map_message);
    publisher.publish(map_message);
  }

  cost_map::CostMap cost_map;
  ros::Publisher publisher;
  ros::Subscriber subscriber;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  ros::init(argc, argv, "gopher_map_publisher");
  ros::NodeHandle nodehandle("~");
  try {
    Loader loader(nodehandle);
    ros::spin();
  } catch (const ecl::StandardException& e) {
    std::cout << ecl::red << "[ERROR] " << e.what() << ecl::reset << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
