##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(cost_map_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    grid_map_msgs
    message_generation
    std_msgs
)

##############################################################################
# Find Packages
##############################################################################

add_message_files(
  FILES
      CostMap.msg
)

add_service_files(
  FILES
  GetCostMap.srv
)

generate_messages(
  DEPENDENCIES
      grid_map_msgs
      std_msgs
)

##############################################################################
# Catkin
##############################################################################

catkin_package(
    CATKIN_DEPENDS
        grid_map_msgs
        message_runtime
        std_msgs
)
