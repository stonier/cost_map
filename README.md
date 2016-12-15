# Cost Map

This is a C++ library directly analogous to ETHZ ASL's [GridMap] library,
but designed for use with costs where the data element is a byte (as opposed to grid_map's doubles).

## Packages Overview

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
roslaunch cost_map_visualisations demo_load_image_bundle.launch --screen
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


[GridMap]: https://github.com/ethz-asl/grid_map

