cmake_minimum_required(VERSION 2.8.3)
project(user_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  std_msgs
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

# Find Nite
find_path(Nite_INCLUDEDIR
          NAMES XnVNite.h
          HINTS /usr/include/ni)
find_library(Nite_LIBRARY
             NAMES XnVNite_1_3_1
             HINTS /usr/lib /usr/local/lib
             PATH_SUFFIXES lib) 

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Com.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES user_tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp std_msgs message_runtime tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenNI_INCLUDEDIR}
  ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})  
link_directories(${orocos_kdl_LIBRARY_DIRS})


## Declare a cpp library
# add_library(user_tracker
#   src/${PROJECT_NAME}/user_tracker.cpp
# )

## Declare a cpp executable
 add_executable(user_tracker src/user_tracker.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(user_tracker user_tracker_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(user_tracker
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OpenNI_LIBRARIES}
 )

