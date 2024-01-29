cmake_minimum_required(VERSION 3.0.2)
project(swarmros)

# Adding option to compile in debug, in order to determine where it crash :)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
 )

find_package(arrc)
find_package(SFML REQUIRED COMPONENTS graphics window system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swarmros
  CATKIN_DEPENDS other_catkin_pkg std_msgs geometry_msgs nav_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(Flightmare_Coverage
  src/fm_coverage.cpp)
add_dependencies(Flightmare_Coverage
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(Flightmare_Coverage
  PRIVATE
  arrc::arrc
  sfml-graphics
  sfml-window
  sfml-system

  PUBLIC
  ${catkin_LIBRARIES}
  )
