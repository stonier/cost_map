##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(cost_map_ros)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        cost_map_core
        cost_map_msgs
        costmap_2d
        ecl_build
        ecl_command_line
        ecl_console
        grid_map_core
        grid_map_ros
        grid_map_visualization
        nav_msgs
        roslib
    )

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp>=0.5)

##############################################################################
# Catkin
##############################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        cost_map_core
        cost_map_msgs
        costmap_2d
        ecl_build
        ecl_command_line
        ecl_console
        grid_map_core
        grid_map_ros
        grid_map_visualization
        nav_msgs
        roslib
    DEPENDS
        Boost
        YAML_CPP
        OpenCV
    )

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
add_definitions(
    ${catkin_DEFINITIONS}
    ${Boost_DEFINITIONS}
    ${OpenCV_DEFINITIONS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${YAML_CPP_LIBRARY_DIRS})

# *spank* opencv calls include_directories internally, which means if you
# don't call it after the other include_directories, it will automagically
# stuff its directories at the front of your include path and this can
# cause headaches.
find_package(OpenCV REQUIRED)

# not necessary to re-include, but should be here for when eventually opencv3
# fixes this for us
include_directories(${OpenCV_INCLUDE_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

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

install(DIRECTORY image_bundles DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
