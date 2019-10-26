cmake_minimum_required(VERSION 2.8.3)
project(v2x_object_detect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp v2x_msgs jsk_recognition_msgs tf autoware_msgs tf2 object_manager_msgs)

find_package(GeographicLib REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}
 ${GeographicLib_INCLUDE_DIRS}
)

find_package(GeographicLib REQUIRED)
catkin_package(
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(v2x_processor_main
  include/v2x_processor/convertMap2Baselink.h
  src/convertMap2Baselink.cpp
  include/v2x_processor/tf_mat.h
  src/tim.cpp
  include/v2x_processor/tim.h
  src/bsm.cpp
  include/v2x_processor/bsm.h
  src/spat.cpp
  include/v2x_processor/spat.h
  src/v2x_processor_main.cpp
)
add_dependencies(v2x_processor_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(v2x_processor_main
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_definitions(${GeographicLib_DEFINITIONS})
