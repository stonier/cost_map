/**
 * @file src/applications/save_image_bundle.cpp
 *
 * Listens to a costmap topic and on receipt of a cost map, it
 * converts and saves it to an image bundle on the filesystem.
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
  ecl::CmdLine cmd("Save a published cost map to an image bundle");
  ecl::UnlabeledValueArg<std::string> topicNameArg(
      "topic_name",
      "Topic name to listen to for costmaps (e.g. /image_bundle/cost_map)",
      true,
      "/image_bundle/cost_map",
      "string");
  ecl::UnlabeledValueArg<std::string> filenameArg(
      "filename",
      "Filename of the bundle's meta yaml file.",
      true,
      "foo.yaml",
      "string");
  cmd.add(topicNameArg);
  cmd.add(filenameArg);

  std::vector<std::string> myargs;
  ros::removeROSArgs(argc, argv, myargs);
  char** myargv = new char*[myargs.size()]; //array to emulate argv.
  for(int i = 0, ie = myargs.size(); i < ie; ++i)
  {
    myargv[i] = const_cast<char*>(myargs[i].data());
  }
  cmd.parse(myargs.size(), myargv);

  ros::init(argc, argv, "save_image_bundle");
  try {
    cost_map::SaveImageBundle dumper(topicNameArg.getValue(), filenameArg.getValue());
//    if ( broken_layer_probably_isnt_obstacle_costs ) {
//      cost_map::Matrix data = loader.cost_map->get("obstacle_costs");
//      for (int i = 0, number_of_rows = data.rows(), number_of_columns = data.cols(); i < number_of_rows; ++i) {
//        for (int j = 0; j < number_of_columns; ++j) {
//          std::cout << static_cast<int>(data(i,j)) << " ";
//        }
//        std::cout << std::endl;
//      }
//    }
    ros::Rate rate(10);
    while ( ros::ok() ) {
      ros::spinOnce();
      if ( dumper.finished ) {
        break;
      } else {
        ROS_WARN_STREAM_DELAYED_THROTTLE(5, "SaveImageBundle : still waiting for a costmap to arrive [zzzz....]");
        rate.sleep();
      }
    }
  } catch (const std::exception& e) {
    std::cout << ecl::red << "[ERROR] " << e.what() << ecl::reset << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
