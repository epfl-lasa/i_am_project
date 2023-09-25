cmake_minimum_required(VERSION 3.0.2)
project(i_am_project)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  roslib
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED eigen3) 

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/workspace_params.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs roslib dynamic_reconfigure
)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(TOOL_SOURCES
                  src/tools/hittable.cpp 
                  src/tools/quattools.cpp
                  include/thirdparty/Utils.cpp)

add_executable(world_sim src/init_world_sim.cpp) 
target_link_libraries(world_sim ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(key_ctrl src/key_ctrl.cpp) 
target_link_libraries(key_ctrl ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(collect_data src/collect_data.cpp ${TOOL_SOURCES}) 
target_link_libraries(collect_data ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} "stdc++fs")

add_executable(air_hockey src/air_hockey.cpp src/move_robot.cpp ${TOOL_SOURCES}) 
target_link_libraries(air_hockey ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} "stdc++fs")
add_dependencies(air_hockey i_am_project_generate_messages_cpp)
