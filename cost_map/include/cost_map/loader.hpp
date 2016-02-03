/**
 * @file /cost_map/include/cost_map/loader.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_LOADER_HPP_
#define cost_map_LOADER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class Loader {
public:
  Loader(ros::NodeHandle& nodehandle);

  void imageResourceNameCallback(const std_msgs::String& msg);

  void publish();

  cost_map::CostMap cost_map;
  ros::Publisher publisher;
  ros::Subscriber subscriber;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map

#endif /* cost_map_LOADER_HPP_ */
