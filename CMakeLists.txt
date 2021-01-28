cmake_minimum_required(VERSION 3.0.2)
project(px4_offboard_jw)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  jw_offboard_lib
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(FILES change_pos.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   mavros_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES px4_offboard_jw
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs px4_offboard_jw
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/px4_offboard_jw.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
  # Topic
    add_executable(multi_plane_sidebearing_uav0_Node src/multi_plane_sidebearing_uav0.cpp)
    target_link_libraries(multi_plane_sidebearing_uav0_Node ${catkin_LIBRARIES})


    add_executable(multi_plane_sidebearing_uav1_Node src/multi_plane_sidebearing_uav1.cpp)
    target_link_libraries(multi_plane_sidebearing_uav1_Node ${catkin_LIBRARIES})


    add_executable(multi_plane_sidebearing_uav2_Node src/multi_plane_sidebearing_uav2.cpp)
    target_link_libraries(multi_plane_sidebearing_uav2_Node ${catkin_LIBRARIES})


    add_executable(multi_plane_sidebearing_uav3_Node src/multi_plane_sidebearing_uav3.cpp)
    target_link_libraries(multi_plane_sidebearing_uav3_Node ${catkin_LIBRARIES})


    add_executable(single_iris_sidebearing_Node src/single_iris_sidebearing.cpp)
    target_link_libraries(single_iris_sidebearing_Node ${catkin_LIBRARIES})


    # add_executable(single_vtol_sidebearing src/single_vtol_sidebearing.cpp)
    # starget_link_libraries(single_vtol_sidebearing ${catkin_LIBRARIES})


    add_executable(single_iris_hold_pos_Node src/single_iris_hold_pos.cpp)
    target_link_libraries(single_iris_hold_pos_Node ${catkin_LIBRARIES})
 
    add_executable(single_plane_hold_speed_Node src/single_plane_hold_speed.cpp)
    target_link_libraries(single_plane_hold_speed_Node ${catkin_LIBRARIES})

    
 
  # Service_Server
    # Position Changer
    #add_executable(change_pos_mc_srv_server src/change_pos_srv_server.cpp)
    #add_dependencies(change_pos_mc_srv_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) target_link_libraries(change_pos_mc_srv_server ${catkin_LIBRARIES})

    #add_executable(change_pos_mc_srv_client src/change_pos_srv_client.cpp)
    #add_dependencies(change_pos_mc_srv_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) target_link_libraries(change_pos_mc_srv_client ${catkin_LIBRARIES})


# 