##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(cost_map)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        cost_map_core
        ecl_build
        ecl_console
        grid_map
    )

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp>=0.5)
find_package(OpenCV REQUIRED)

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
        ecl_build
        ecl_console
        grid_map
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

