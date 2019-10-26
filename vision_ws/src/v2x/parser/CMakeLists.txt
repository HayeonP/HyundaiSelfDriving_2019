cmake_minimum_required(VERSION 2.8.3)
project(parser)

add_compile_options(-O -Wall -std=c++14 -fpermissive )

find_package(catkin REQUIRED COMPONENTS roscpp v2x_msgs)

catkin_package(
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/j2735 # when in git directory
)
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/)

add_executable(v2x_main
  src/decode.cpp
  src/v2x_main.cpp
)
add_dependencies(v2x_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(v2x_main
  libj2735.a
  ${catkin_LIBRARIES}
)