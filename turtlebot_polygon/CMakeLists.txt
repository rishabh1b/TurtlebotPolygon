cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_polygon)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(openpolygon src/open_polygon.cpp)
add_executable(proppolygon src/prop_polygon.cpp)
target_link_libraries(openpolygon ${catkin_LIBRARIES})
target_link_libraries(proppolygon ${catkin_LIBRARIES})
