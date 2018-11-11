cmake_minimum_required(VERSION 2.8.3)
project(gmm_obs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_filters
  controller_manager
  joint_state_controller
  robot_state_publisher
)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gmm_obs
  CATKIN_DEPENDS 
  geometry_msgs
  nav_msgs
  roscpp 
  rospy 
  std_msgs
  message_filters
  controller_manager
  joint_state_controller
  robot_state_publisher
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(contact_pendulum src/ContactPlugin.cpp)
target_link_libraries(contact_pendulum ${GAZEBO_libraries})
