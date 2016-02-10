/**
 * @file src/applications/map_loader.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/console.hpp>
#include <ecl/exceptions.hpp>

#include <ros/ros.h>
#include <string>

#include "../../include/cost_map/loader.hpp"
#include "../../include/cost_map/utilities.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_loader");
  ros::NodeHandle nodehandle("~");
  std::string image_resource_name;
  nodehandle.param<std::string>("resource_name", image_resource_name, "cost_map_visualisations/example.yaml");
  try {
    cost_map::Loader loader(image_resource_name);
    ros::spin();
  } catch (const ecl::StandardException& e) {
    std::cout << ecl::red << "[ERROR] " << e.what() << ecl::reset << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
