/**
 * @file src/applications/alignment_test.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <atomic>
#include <thread>
#include <ros/common.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../../include/cost_map_ros/converter.hpp"

/*****************************************************************************
** Interfaces
*****************************************************************************/

void pretty_print(costmap_2d::Costmap2DROS &ros_costmap) {  // won't let us use const here, but treat it as such
  for ( unsigned int i = 0; i < ros_costmap.getCostmap()->getSizeInCellsX(); ++i ) {
    for ( unsigned int j = 0; j < ros_costmap.getCostmap()->getSizeInCellsY(); ++j ) {
      std::cout << static_cast<int>(ros_costmap.getCostmap()->getCost(i,j)) << " ";
    }
    std::cout << std::endl;
  }
}

class ROSServer {
public:
  typedef costmap_2d::Costmap2DROS ROSCostmap;
  typedef std::shared_ptr<ROSCostmap> ROSCostmapPtr;

  ROSServer(const std::string& name,
            const std::string& base_link_transform_name,
            const cost_map::Position& origin,
            const double& width,
            const double& height,
            const std::vector<unsigned char>& fill
            )
    : transform_listener(ros::Duration(1.0))
  {
    ros::NodeHandle private_node_handle("~");
    // lots of parameters here affect the construction ( e.g. rolling window)
    // if you don't have any parameters set, then this
    //  - alot of defaults which get dumped on the ros param server
    //  - fires up an obstacle layer and an inflation layer
    //  - creates a publisher for an occupancy grid
    private_node_handle.setParam(name + "/robot_base_frame", base_link_transform_name);
    private_node_handle.setParam(name + "/origin_x", origin.x());
    private_node_handle.setParam(name + "/origin_y", origin.y());
    private_node_handle.setParam(name + "/width", width);
    private_node_handle.setParam(name + "/height", height);
    private_node_handle.setParam(name + "/resolution", 1.0);
    private_node_handle.setParam(name + "/robot_radius", 0.03); // clears 1 cell if inside, up to 4 cells on a vertex
    costmap = std::make_shared<ROSCostmap>(name, transform_listener);

    for ( unsigned int index = 0; index < costmap->getCostmap()->getSizeInCellsY(); ++index ) {
      for ( unsigned int fill_index = 0;
          fill_index < std::min(static_cast<unsigned int>(fill.size()), costmap->getCostmap()->getSizeInCellsX());
          ++fill_index
          )
      {
        costmap->getCostmap()->setCost(fill_index, index, fill[fill_index]);
      }
    }
  }

  ROSCostmapPtr getROSCostmap() { return costmap; };

private:
  ROSCostmapPtr costmap;
  tf::TransformListener transform_listener;

};

class TransformBroadcaster {
public:
  TransformBroadcaster() {}
  void add(const std::string& name, tf::Vector3 origin, const tf::Quaternion& orientation) {
    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(orientation);
    transforms.insert(std::pair<std::string, tf::Transform>(name, transform));
  }
  void broadcast() {
    tf::TransformBroadcaster tf_broadcaster;
    while(ros::ok())
    {
      for (std::pair<std::string, tf::Transform> p: transforms) {
        tf::StampedTransform stamped_transform(p.second, ros::Time::now(), "map", p.first);
        tf_broadcaster.sendTransform(stamped_transform);
      }
      ros::Duration(0.1).sleep();
    }
  }
private:
  std::map<std::string, tf::Transform> transforms;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "converter"); // , ros::init_options::AnonymousName);

  /********************
  ** Transforms
  ********************/
  TransformBroadcaster broadcaster;
  broadcaster.add("base_link_5x5", tf::Vector3(1.0,  1.0, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_4x4", tf::Vector3(1.0, -3.0, 0.0), tf::Quaternion(0, 0, 0, 1));
  std::thread thread(&TransformBroadcaster::broadcast, broadcaster);

  /********************
  ** ROS Costmaps
  ********************/
  std::vector<unsigned char> fill = {costmap_2d::FREE_SPACE + costmap_2d::INSCRIBED_INFLATED_OBSTACLE/10,
                                     costmap_2d::INSCRIBED_INFLATED_OBSTACLE/2,
                                     costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
                                     costmap_2d::LETHAL_OBSTACLE,
                                     costmap_2d::NO_INFORMATION,
  };
  // The following should result in costmaps with strips of increasing cost in the x-direction,
  // but also 1-4 cells cleared by the footprint (see rviz)
  ROSServer ros_costmap_5x5("five_by_five", "base_link_5x5", cost_map::Position(0.0, 0.0), 5.0, 5.0, fill);
  ROSServer ros_costmap_4x4("four_by_four", "base_link_4x4", cost_map::Position(0.0, -5.0), 4.0, 4.0, fill);

  /********************
  ** New Costmaps
  ********************/
  // full windows
  cost_map::CostMapPtr cost_map_5x5 = cost_map::fromROSCostMap2D(*(ros_costmap_5x5.getROSCostmap()));
  cost_map::CostMapPtr cost_map_4x4 = cost_map::fromROSCostMap2D(*(ros_costmap_4x4.getROSCostmap()));
  // subwindows

//  cost_map::Length geometry(0,0);
//  cost_map::CostMapPtr cost_map = cost_map::fromROSCostMap2D(map_ros, geometry);

  ros::NodeHandle node_handle;
  ros::Publisher pub_5x5 = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_5x5", 1, true);
  ros::Publisher pub_4x4 = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_4x4", 1, true);

  nav_msgs::OccupancyGrid occupancy_msg;
  cost_map::toOccupancyGrid(*cost_map_5x5, cost_map_5x5->getLayers()[0], occupancy_msg);
  pub_5x5.publish(occupancy_msg);
  cost_map::toOccupancyGrid(*cost_map_4x4, cost_map_4x4->getLayers()[0], occupancy_msg);
  pub_4x4.publish(occupancy_msg);

  /********************
  ** Spin & Shutdown
  ********************/
  ros::spin();
  thread.join();

  return 0;
}
