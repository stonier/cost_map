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
 * @param[in] resource_name : ros package resource name
 * @return absolute path to the resource
 *
 * @throw std::invalid_argument if resource_name is the incorrect syntax
 * @throw std::runtime_error if the resource could not be found
 */
std::string resolveResourceName(const std::string& resource_name);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_ros_UTILS_HPP_ */
