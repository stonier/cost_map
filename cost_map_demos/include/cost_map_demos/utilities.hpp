/**
 * @file /cost_map_demos/include/cost_map_demos/utilities.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_demos_UTILITIES_HPP_
#define cost_map_demos_UTILITIES_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <atomic>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map_demos {

/*****************************************************************************
** Transforms
*****************************************************************************/

/**
 * @brief Broadcast a set of transforms useful for various demos.
 */
class TransformBroadcaster {
public:
  TransformBroadcaster() : shutdown_flag(false) {}
  virtual ~TransformBroadcaster();
  void add(const std::string& name, tf::Vector3 origin, const tf::Quaternion& orientation);

  void startBroadCastingThread();
  void broadcast();
  void shutdown();

private:
  std::map<std::string, tf::Transform> transforms;
  std::thread broadcasting_thread;
  std::atomic<bool> shutdown_flag;
};

/*****************************************************************************
** Printing
*****************************************************************************/

/**
 * @brief Pretty print a ros cost map in ascii format to stdout.
 * @param[in] ros_costmap : won't let us use a const here, but treat it as such
 */
void pretty_print(costmap_2d::Costmap2DROS &ros_costmap);

/**
 * @brief Pretty print a ros cost map in ascii format to stdout.
 *
 * @param[in] stream : the incoming object into which to insert stuff
 * @param[in] ros_costmap : won't let us use a const here, but treat it as such
 * @returns std::ostream
 */
std::ostream& operator <<(std::ostream& stream, costmap_2d::Costmap2DROS& ros_costmap);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map_demos

#endif /* cost_map_demos_UTILITIES_HPP_ */
