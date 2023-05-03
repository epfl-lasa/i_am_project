cmake_minimum_required(VERSION 3.0.2)
project(i_am_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
find_package(Eigen3 REQUIRED eigen3) 
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR} "/usr/local/include")

add_executable(momentum_hit src/hit_motion_sim.cpp src/dynamical_system.cpp) # src/optimizer.cpp) 
target_link_libraries(momentum_hit ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} osqp::osqp OsqpEigen::OsqpEigen "stdc++fs" "-L/usr/local/lib")
add_dependencies(momentum_hit i_am_project_generate_messages_cpp)

add_executable(momentum_hit_real src/hit_motion.cpp src/dynamical_system.cpp) # src/optimizer.cpp) 
target_link_libraries(momentum_hit_real ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} osqp::osqp OsqpEigen::OsqpEigen "stdc++fs" "-L/usr/local/lib")
add_dependencies(momentum_hit_real i_am_project_generate_messages_cpp)

add_executable(push_release_real src/push_release_real.cpp src/dynamical_system.cpp) # src/optimizer.cpp) 
target_link_libraries(push_release_real ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} osqp::osqp OsqpEigen::OsqpEigen "stdc++fs" "-L/usr/local/lib")
add_dependencies(push_release_real i_am_project_generate_messages_cpp)

add_executable(inertia_QP src/inertia_motion_qp.cpp src/dynamical_system.cpp) 
target_link_libraries(inertia_QP ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} osqp::osqp OsqpEigen::OsqpEigen "stdc++fs" "-L/usr/local/lib")
add_dependencies(inertia_QP i_am_project_generate_messages_cpp)

set_property(TARGET momentum_hit PROPERTY CXX_STANDARD 17)
set_property(TARGET momentum_hit PROPERTY CXX_STANDARD_REQUIRED ON)

set_property(TARGET inertia_QP PROPERTY CXX_STANDARD 17)
set_property(TARGET inertia_QP PROPERTY CXX_STANDARD_REQUIRED ON)

