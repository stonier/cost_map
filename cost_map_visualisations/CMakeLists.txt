##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(cost_map_visualisations)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        ecl_build
        ecl_console
        cost_map
        cost_map_core
        cost_map_msgs
        nav_msgs
        roscpp
        roslib
    )

##############################################################################
# Catkin
##############################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        ecl_build
        ecl_console
        cost_map
        cost_map_core
        cost_map_msgs
        nav_msgs
        roscpp
        roslib
    )

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
add_definitions(${catkin_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# Project Sources
##############################################################################

add_subdirectory(src)

##############################################################################
# Installs
##############################################################################

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

