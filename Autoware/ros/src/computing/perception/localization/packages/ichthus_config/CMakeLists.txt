cmake_minimum_required(VERSION 2.8.3)
project(ichthus_config)
add_compile_options(-std=c++17)

find_package(autoware_msgs REQUIRED)
find_package(autoware_config_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
	geometry_msgs
  tf
  autoware_msgs
  autoware_config_msgs
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ichthus_config
 CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs tf autoware_msgs autoware_config_msgs
 DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(ichthus_config
  src/ichthus_config.cpp
)
add_dependencies(ichthus_config ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ichthus_config
  ${catkin_LIBRARIES}
)
