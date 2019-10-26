cmake_minimum_required(VERSION 2.8.3)
project(startpos_setter)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
	geometry_msgs
  tf
)

add_message_files(FILES
  NDTStat.msg
  Point.msg
  PointArray.msg
  DTLaneArray.msg
  DTLane.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES startpos_setter
 CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs tf
 DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(startpos_setter
  src/startpos_setter.cpp
)
add_dependencies(startpos_setter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(startpos_setter
  ${catkin_LIBRARIES}
)