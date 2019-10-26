cmake_minimum_required(VERSION 2.8.3)
project(ndtscore_publisher)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(FILES
  NDTStat.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
 LIBRARIES ndtscore_publisher
 CATKIN_DEPENDS
  message_runtime message_generation roscpp std_msgs
 DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(ndtscore_publisher
  src/ndtscore_publisher.cpp
)
add_dependencies(ndtscore_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ndtscore_publisher
  ${catkin_LIBRARIES}
)