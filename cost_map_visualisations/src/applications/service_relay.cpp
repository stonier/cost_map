/**
 * @file /cost_map_visualisations/src/applications/service_relay.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <ecl/command_line.hpp>
#include <ros/ros.h>
#include <cost_map_ros/cost_map_ros.hpp>
#include <cost_map_msgs/GetCostMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>

/*****************************************************************************
** Main
*****************************************************************************/

/**
 * Tunes into a cost map service and relays it as an Occupancy Grid.
 *
 * Mostly for demonstration/debugging purposes as you are usually
 * viewing a whole global costmap in rviz anyway.
 */
int main(int argc, char **argv) {
  /****************************************
  ** Parsing
  ****************************************/
  ecl::CmdLine cmd("Relays service provided cost maps to an occupancy grid publisher.");
  ecl::ValueArg<float> lengthArg("l","length","Length of a side of the costmap subwindow.", false, 2, "float", cmd);
  ecl::ValueArg<std::string> serviceNameArg("s","service_name","Service name to call.", false, "get_cost_map", "string", cmd);
  ecl::ValueArg<float> rateArg("r","rate","Rate at which to relay (hz).", false, 1, "float", cmd);
  ecl::ValueArg<std::string> layerNameArg("n","layer_name","Layer name to convert.", false, "obstacle_costs", "string", cmd);
  cmd.parse(argc, argv);

  /****************************************
  ** Ros
  ****************************************/
  ros::init(argc, argv, "cost_map_client");
  ros::NodeHandle nodehandle("~");
  bool persistent = true, latched = true;
  std::cout << "Looking up service: " << serviceNameArg.getValue() << std::endl;
  ros::ServiceClient get_cost_map = nodehandle.serviceClient<cost_map_msgs::GetCostMap>(serviceNameArg.getValue(), persistent);
  if ( !get_cost_map.waitForExistence(ros::Duration(10.0)) ) {
    ROS_ERROR_STREAM("Cost Map Service Relay : failed to find the GetCostMap service on " << nodehandle.resolveName(serviceNameArg.getValue(), true));
    return EXIT_FAILURE;
  }
  ros::Publisher occupancy_grid_publisher = nodehandle.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 5, latched);

  /****************************************
  ** Service Call/Relay Loop
  ****************************************/
  cost_map_msgs::GetCostMap srv;
  srv.request.length_x = lengthArg.getValue();
  srv.request.length_y = lengthArg.getValue();
  std::string layer_name = layerNameArg.getValue();
  ros::Rate rate(rateArg.getValue());
  while ( ros::ok() ) {
    if (!get_cost_map.call(srv))
    {
      // could be just that someone closed down the navigation source and will restart
      // so quietly ignore, but do check proceed to check again for ros::ok()
      continue;
    }
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    cost_map::CostMap cost_map;
    cost_map::fromMessage(srv.response.map, cost_map);
    if ( cost_map.getLayers().size() == 1 ) {
      layer_name = cost_map.getLayers()[0];
    }
    cost_map::toOccupancyGrid(cost_map, layer_name, occupancy_grid_msg);
    occupancy_grid_publisher.publish(occupancy_grid_msg);
    rate.sleep();
    ros::spinOnce();
  }
  return EXIT_SUCCESS;
}

