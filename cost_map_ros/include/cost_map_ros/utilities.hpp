/**
 * @file include/cost_map_ros/utilities.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_ros_UTILS_HPP_
#define cost_map_ros_UTILS_HPP__UTILS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Use rospack to resolve a costmap given a ros package resource name.
 *
 * Costmaps are yaml files with configuration and a pointer to an opencv
 * image to load. They are exported in the package.xml with a 'cost_map'
 * tag and 'yaml' attribute.
 *
 *   http://wiki.ros.org/Names#Package_Resource_Names
 *
 * @param package
 * @param name
 * @return absolute path to the resource
 *
 * @exception ecl::StandardException with ecl::NotFoundError if not found.
 */
std::string resolveResourceName(const std::string& resource_name);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_ros_UTILS_HPP_ */
