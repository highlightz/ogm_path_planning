cmake_minimum_required(VERSION 2.8.3)
project(ogmpp_prms)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ogmpp_graph
)

##################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ogmpp_prms
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    ogmpp_graph
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/ogmpp_prms
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ogmpp_prms_node 
  src/ogmpp_prms/main.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ogmpp_prms_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(ogmpp_prms_node
  ${catkin_LIBRARIES}
  ogmpp_graph_lib
)

