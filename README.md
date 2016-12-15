# Cost Map

## Overview

This is a C++ library directly analogous to ETHZ ASL's [GridMap] library,
but designed for use with costs where the data element is a byte (as opposed to grid_map's doubles).

**Author: Daniel Stonier**
**Maintainer: Daniel Stonier**

## Packages Overview

* ***cost_map*** is the meta-package for the grid map library.
* ***cost_map_core*** implements the algorithms of the cost map library. It provides the `CostMap` class and several helper classes such as the iterators. This package is implemented without [ROS] dependencies.
* ***cost_map_ros*** is the main package for [ROS] dependent projects using the cost map library. It provides the interfaces to convert cost maps from and to messages and files.
* ***cost_map_msgs*** holds the [ROS] message and service definitions around the [grid_map_msg/GridMap] message type.
* ***cost_map_visualisations*** contains a node written to convert CostMap messages to other [ROS] message types for example for  visualization in [RViz].
* ***cost_map_demos*** contains several nodes for demonstration purposes.


## Image Bundles

### About

An image bundle is comprised of two parts - 1) meta-information about the costmap stored in a yaml file and
2) layer data that is stored alongside in grayscale images. These enable a convenient means of
saving/loading cost maps to and from files on disk.

A typical yaml file for an image bundle looks something like:

```yaml
# frame in the world that this map is attached to
frame_id: map
# co-ordinates of the centre of the map relative to the frame_id
centre_x: 0
centre_y: 0
# dimensional properties of the map
resolution: 0.05
number_of_cells_x: 200
number_of_cells_y: 200
# data layers
layers:
  - layer_name: obstacle_costs
    layer_data: obstacle_costs.png
  - layer_name: static_costs
    layer_data: can_be_some_other_name.png
```

### Demo

```bash
# load an image bundle and visualise the cost map
roslaunch cost_map_demos load_image_bundle.launch --screen
```

### Command Line Utilities

There exist two command line utilities for loading and saving to and from a cost map topic:

```bash
# load and publish to '/foo/cost_map'
rosrun cost_map_ros load_image_bundle -t /foo/cost_map example.yaml
# subscribe and save from '/foo/cost_map' to foo.yaml
mkdir foo
cd foo
rosrun cost_map_ros save_image_bundle /foo/cost_map foo.yaml
```

### Library Usage

* `cost_map::toImageBundle()` : convert a `cost_map::CostMap` object to an image bundle
* `cost_map::fromImageBundle()` : load an image bundle into a `cost_map::CostMap` object

See the [LoadImageBundle/SaveImageBundle](https://github.com/stonier/cost_map/blob/devel/cost_map_ros/src/lib/image_bundles.cpp)
classes which illustrate how the command line utilities use these api.

## Inflation Computers

### Demo

```bash
# load an image bundle and visualise the cost map
roslaunch cost_map_demos inflations.launch --screen
```

Obstacle Layer | Inflated Layer | Deflated Layer
:---: | :---: | :---:
[![Obstacle Layer](cost_map_demos/doc/images/inflation/obstacle_layer_preview.png)](cost_map_demos/doc/images/inflation/obstacle_layer.gif) | [![Inflated Layer](cost_map_demos/doc/images/inflation/inflation_layer_preview.png)](cost_map_demos/doc/images/inflation/inflation_layer.png) | [![Deflated Layer](cost_map_demos/doc/images/inflation/deflation_layer_preview.png)](cost_map_demos/doc/images/inflation/obstacle_layer.png)

### Classes

* `cost_map::Inflate` : functor that executes (with the assistance of an inflation computer) the inflation process
* `cost_map::ROSInflationComputer` : emulates the ROS inflation algorithm
* `cost_map::Deflate` : functor reverses an inflation computation

See the [inflation demo program](https://github.com/stonier/cost_map/blob/devel/cost_map_demos/src/inflation.cpp)
which illustrates how to use these classes.

[GridMap]: https://github.com/ethz-asl/grid_map
[ROS]: http://www.ros.org
[RViz]: http://wiki.ros.org/rviz

