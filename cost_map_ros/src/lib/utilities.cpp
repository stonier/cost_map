/**
 * @file /src/lib/utilities.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/exceptions.hpp>
#include <iostream>
#include <ros/package.h>
#include <string>
#include "../../include/cost_map_ros/utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

std::string resolveResourceName(const std::string& resource_name) {

  /********************
  ** Resource Name
  ********************/
  std::string delimiter = "/";
  int index = resource_name.find(delimiter);
  if ( index == std::string::npos ) {
    throw ecl::StandardException(LOC, ecl::InvalidArgError, std::string("'") + resource_name + std::string("' is not a valid resource name."));
  }
  std::string package = resource_name.substr(0, index);
  std::string name = resource_name.substr(index + delimiter.length());

  /********************
  ** Lookup
  ********************/
  std::string plugin_package = "cost_map";  // nav_core, cost_map
  std::string attribute = "image_resource"; // plugin, yaml

  // pending release of https://github.com/ros/ros/pull/103
  // std::vector<std::string> packages, costmaps;
  // ros::package::getPlugins(plugin_package, attribute, packages, costmaps);
  // for (unsigned int i = 0; i < packages.size(); ++i ) {
  //   std::cout << packages[i] << " " << costmaps[i] << std::endl;
  // }

  // grunt workaround for now - its not 100% robust
  std::vector<std::string> costmaps;
  ros::package::getPlugins(plugin_package, attribute, costmaps);
  for (const auto& filename : costmaps) {
    if ( (filename.find(package) != std::string::npos) && (filename.find(name) != std::string::npos) ) {
      return filename;
    }
  }
  // not found
  throw ecl::StandardException(LOC, ecl::NotFoundError, std::string("resource name '") + resource_name + std::string("' is not available (try 'rospack plugins --attrib=yaml cost_map')."));
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
