cmake_minimum_required(VERSION 3.0.2)
project(i_am_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED eigen3) 
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES i_am_drop
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(momentum_hit src/node_hit_sim.cpp src/dynamical_system.cpp src/optimizer.cpp) 
target_link_libraries(momentum_hit ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} osqp::osqp OsqpEigen::OsqpEigen "stdc++fs")
add_dependencies(momentum_hit i_am_project_generate_messages_cpp)

set_property(TARGET momentum_hit PROPERTY CXX_STANDARD 17)
set_property(TARGET momentum_hit PROPERTY CXX_STANDARD_REQUIRED ON)

