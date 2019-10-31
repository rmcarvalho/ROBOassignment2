cmake_minimum_required(VERSION 2.8.3)
project(robo_assign2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    tf
    stdr_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(follow_wall_robot_node src/main.cpp src/follow_wall.cpp)

add_dependencies(follow_wall_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(follow_wall_robot_node
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

 install(TARGETS follow_wall_robot_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )