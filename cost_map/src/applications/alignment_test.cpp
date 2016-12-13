#include <atomic>
#include <thread>
#include <ros/common.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/testing_helper.h>

#include "cost_map/converter.hpp"

using namespace costmap_2d;

static std::atomic<bool> running_;

void transformPublish()
{
  tf::TransformBroadcaster tf_broadcaster;
  tf::StampedTransform transform;
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));

  while(running_)
  {
    tf::StampedTransform stamped_transform(transform, ros::Time::now(), "base_link", "map");
    tf::StampedTransform stamped_transform_back(transform, ros::Time::now(), "map", "base_link");

    tf_broadcaster.sendTransform(stamped_transform);
    tf_broadcaster.sendTransform(stamped_transform_back);

    ros::Duration(0.1).sleep();
  }
}

void test(ros::NodeHandle& node_handle)
{
  tf::TransformListener transform_listener(ros::Duration(1.0));
  std::cout << "finished listening" << std::endl;

  Costmap2DROS map_ros("frame", transform_listener);
  LayeredCostmap* layers = map_ros.getLayeredCostmap();
  addStaticLayer(*layers, transform_listener);
  ObstacleLayer* olayer = addObstacleLayer(*layers, transform_listener);

  // Update will fill in the costmap with the static map
  layers->updateMap(0,0,0);

  Costmap2D* ros_cost_map = layers->getCostmap();

  // this is the wierd one - it only covers size in cells * resolution - half a cell
  cost_map::Length geometry(ros_cost_map->getSizeInMetersX(), ros_cost_map->getSizeInMetersY());
//  double resolution = ros_cost_map->getResolution()
//  cost_map::Length geometry(ros_cost_map->getSizeInCellsX()*resolution,
//                            ros_cost_map->getSizeInCellsY()*resolution);
  cost_map::CostMapPtr cost_map = cost_map::fromROSCostMap2D(map_ros, geometry);

  ros::Publisher pub = node_handle.advertise<nav_msgs::OccupancyGrid>("converted_back", 1, true);
  nav_msgs::OccupancyGrid occupancy_msg;
  cost_map::toOccupancyGrid(*cost_map, cost_map->getLayers()[0], occupancy_msg);

  pub.publish(occupancy_msg);

  ros::spin();
}

int main(int argc, char** argv)
{
  std::cout << "Run with 'roslaunch costmap_2d obstacle_tests.launch'" << std::endl;

  running_ = true;

  ros::init(argc, argv, "", ros::init_options::AnonymousName);

  ros::NodeHandle node_handle;

  std::thread publish_thread(transformPublish);

  test(node_handle);

  ros::spin();

  running_ = false;

  return 0;
}
