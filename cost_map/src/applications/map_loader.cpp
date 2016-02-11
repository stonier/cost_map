/**
 * @file src/applications/map_loader.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map.hpp>
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
    if ( 0 ) {
      cost_map::Matrix data = loader.cost_map->get("obstacle_cost");
      for (int i = 0, number_of_rows = data.rows(), number_of_columns = data.cols(); i < number_of_rows; ++i) {
        for (int j = 0; j < number_of_columns; ++j) {
          std::cout << static_cast<int>(data(i,j)) << " ";
        }
        std::cout << std::endl;
      }
    }
    ros::spin();
  } catch (const ecl::StandardException& e) {
    std::cout << ecl::red << "[ERROR] " << e.what() << ecl::reset << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
