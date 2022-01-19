cmake_minimum_required(VERSION 3.0.2)
project(i_am_project)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  roslib
)

find_package(Eigen3 REQUIRED eigen3) 


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs roslib
)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})


set(MODE_SOURCES
                  src/modes/track.cpp 
                  src/modes/block.cpp 
                  src/modes/hit_DS.cpp 
                  src/modes/post_hit.cpp 
                  src/modes/rest.cpp)
set(TOOL_SOURCES
                  src/tools/modeselektor.cpp
                  src/tools/hittable.cpp 
                  src/tools/quattools.cpp
                  src/tools/calculate_alpha.cpp 
                  src/tools/nominal_DS_aux.cpp 
                  src/tools/nominal_DS_main.cpp 
                  src/tools/modulated_DS.cpp
                  include/thirdparty/Utils.cpp)

add_executable(AH_init src/1v1_init.cpp) 
target_link_libraries(AH_init ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(AH_trail_init src/1v1_trail_init.cpp) 
target_link_libraries(AH_trail_init ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(AH_key_ctrl src/1v1_key_ctrl.cpp) 
target_link_libraries(AH_key_ctrl ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(AH_master src/1v1_master.cpp ${MODE_SOURCES} ${TOOL_SOURCES}) 
target_link_libraries(AH_master ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} "stdc++fs")
add_dependencies(AH_master i_am_project_generate_messages_cpp)