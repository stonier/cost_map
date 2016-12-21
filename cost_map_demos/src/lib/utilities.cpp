/**
 * @file /cost_map_demos/src/lib/utilities.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/cost_map_demos/utilities.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map_demos {

/*****************************************************************************
** Implementation
*****************************************************************************/

void pretty_print(costmap_2d::Costmap2DROS &ros_costmap) {
  for ( unsigned int i = 0; i < ros_costmap.getCostmap()->getSizeInCellsX(); ++i ) {
    for ( unsigned int j = 0; j < ros_costmap.getCostmap()->getSizeInCellsY(); ++j ) {
      std::cout << static_cast<int>(ros_costmap.getCostmap()->getCost(i,j)) << " ";
    }
    std::cout << std::endl;
  }
}

std::ostream& operator <<(std::ostream& stream, costmap_2d::Costmap2DROS& ros_costmap) {
  for ( unsigned int i = 0; i < ros_costmap.getCostmap()->getSizeInCellsX(); ++i ) {
    for ( unsigned int j = 0; j < ros_costmap.getCostmap()->getSizeInCellsY(); ++j ) {
      stream << static_cast<int>(ros_costmap.getCostmap()->getCost(i,j)) << " ";
    }
    stream << std::endl;
  }
  return stream;
}

/*****************************************************************************
** TransformBroadcaster
*****************************************************************************/

TransformBroadcaster::~TransformBroadcaster() {
  broadcasting_thread.join();
}

void TransformBroadcaster::shutdown() {
  shutdown_flag = true;
}

void TransformBroadcaster::add(const std::string& name, tf::Vector3 origin, const tf::Quaternion& orientation) {
  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(orientation);
  transforms.insert(std::pair<std::string, tf::Transform>(name, transform));
}

void TransformBroadcaster::startBroadCastingThread() {
  broadcasting_thread = std::thread(&TransformBroadcaster::broadcast, this);
}

void TransformBroadcaster::broadcast() {
  tf::TransformBroadcaster tf_broadcaster;
  while(ros::ok() && !shutdown_flag )
  {
    for (std::pair<std::string, tf::Transform> p: transforms) {
      tf::StampedTransform stamped_transform(p.second, ros::Time::now(), "map", p.first);
      tf_broadcaster.sendTransform(stamped_transform);
    }
    ros::Duration(0.1).sleep();
  }
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map_demos
