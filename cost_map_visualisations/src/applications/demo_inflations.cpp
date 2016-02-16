/**
 * @file /cost_map_visualisations/src/applications/demo_inflations.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map/cost_map.hpp>
#include <ecl/console.hpp>
#include <ecl/exceptions.hpp>
#include <memory>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  /****************************************
  ** Ros
  ****************************************/
  ros::init(argc, argv, "demo_inflations");
  ros::NodeHandle nodehandle("~");

  /****************************************
  ** Load
  ****************************************/
  std::string image_resource_name = "cost_map_visualisations/example.yaml";
  std::shared_ptr<cost_map::Loader> loader;
  try {
    loader = std::make_shared<cost_map::Loader>(image_resource_name);
    if ( 0 ) {
      cost_map::Matrix data = loader->cost_map->get("obstacle_costs");
      for (int i = 0, number_of_rows = data.rows(), number_of_columns = data.cols(); i < number_of_rows; ++i) {
        for (int j = 0; j < number_of_columns; ++j) {
          std::cout << static_cast<int>(data(i,j)) << " ";
        }
        std::cout << std::endl;
      }
    }
    /****************************************
    ** Inflate & Deflate
    ****************************************/
    float inscribed_radius = 0.3;
    float inflation_radius = 0.8;
    float inflation_exponential_rate = 5.0;
    cost_map::Inflate inflate;
    cost_map::ROSInflationComputer inflation_computer(inscribed_radius, inflation_exponential_rate);
    inflate("obstacle_costs", "inflated_costs",
            inflation_radius,
            inflation_computer,
            *(loader->cost_map)
            );
    cost_map::Deflate deflate;
    deflate("inflated_costs", "deflated_costs", *(loader->cost_map));
  } catch (const ecl::StandardException& e) {
    std::cout << ecl::red << "[ERROR] " << e.what() << ecl::reset << std::endl;
    return EXIT_FAILURE;
  }
  /****************************************
  ** Publish
  ****************************************/
  bool latched = true;
  ros::Publisher obstacle_publisher = nodehandle.advertise<nav_msgs::OccupancyGrid>("obstacle_layer", 1, latched);
  ros::Publisher inflation_publisher = nodehandle.advertise<nav_msgs::OccupancyGrid>("inflation_layer", 1, latched);
  ros::Publisher deflation_publisher = nodehandle.advertise<nav_msgs::OccupancyGrid>("deflation_layer", 1, latched);

  nav_msgs::OccupancyGrid obstacle_occupancy_grid_msg, inflated_occupancy_grid_msg, deflated_occupancy_grid_msg;
  cost_map::toOccupancyGrid(*(loader->cost_map), "obstacle_costs", obstacle_occupancy_grid_msg);
  cost_map::toOccupancyGrid(*(loader->cost_map), "inflated_costs", inflated_occupancy_grid_msg);
  cost_map::toOccupancyGrid(*(loader->cost_map), "deflated_costs", deflated_occupancy_grid_msg);
  obstacle_publisher.publish(obstacle_occupancy_grid_msg);
  inflation_publisher.publish(inflated_occupancy_grid_msg);
  deflation_publisher.publish(deflated_occupancy_grid_msg);

  /****************************************
  ** Spin
  ****************************************/
  ros::spin();
  return EXIT_SUCCESS;
}

