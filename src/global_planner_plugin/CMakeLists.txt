cmake_minimum_required(VERSION 3.0.2)
project(global_planner_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    nav_msgs
    geometry_msgs
    tf
)

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES global_planner_plugin
    CATKIN_DEPENDS roscpp
        geometry_msgs
        nav_msgs
        tf
)

# find_package(PCL REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

add_library(global_planner_plugin
    plugins/global_planner_plugin.cpp
)

