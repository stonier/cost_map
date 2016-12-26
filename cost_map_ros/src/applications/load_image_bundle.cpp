/**
 * @file src/applications/load_image_bundle.cpp
 *
 * Grabs an image bundle, loads it into a costmap and publishes it.
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_ros/image_bundles.hpp>
#include <ecl/command_line.hpp>
#include <ecl/console.hpp>
#include <exception>

#include <ros/init.h>
#include <ros/ros.h>
#include <string>

#include "../../include/cost_map_ros/utilities.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  ecl::CmdLine cmd("Load an image bundle into a costmap and publish");
  ecl::ValueArg<std::string> topicNameArg(
      "t",
      "topic_name",
      "Where to publish [default: ~cost_map]",
      false,
      "cost_map",
      "string");
  ecl::UnlabeledValueArg<std::string> imageBundleArg(
      "image_bundle_location",
      "A yaml file with meta-data about the costmap to be built",
      true,
      "cost_map_ros/example",
      "string (package/name OR yaml filename)");
  cmd.add(topicNameArg);
  cmd.add(imageBundleArg);
  std::vector<std::string> myargs;
  ros::removeROSArgs(argc, argv, myargs);
  char** myargv = new char*[myargs.size()]; //array to emulate argv.
  for(int i = 0, ie = myargs.size(); i < ie; ++i)
  {
    myargv[i] = const_cast<char*>(myargs[i].data());
  }
  cmd.parse(myargs.size(), myargv);


  ros::init(argc, argv, "image_bundle");
  try {
    cost_map::LoadImageBundle loader(imageBundleArg.getValue(), topicNameArg.getValue());
//    if ( broken_layer_probably_isnt_obstacle_costs ) {
//      cost_map::Matrix data = loader.cost_map->get("obstacle_costs");
//      for (int i = 0, number_of_rows = data.rows(), number_of_columns = data.cols(); i < number_of_rows; ++i) {
//        for (int j = 0; j < number_of_columns; ++j) {
//          std::cout << static_cast<int>(data(i,j)) << " ";
//        }
//        std::cout << std::endl;
//      }
//    }
    ros::spin();
  } catch (const std::exception& e) {
    std::cout << ecl::red << "[ERROR] " << e.what() << ecl::reset << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
