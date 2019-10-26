cmake_minimum_required(VERSION 2.8.3)
project(nodelet_lidar_driver)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation ${${PROJECT_NAME}_CATKIN_DEPS}
)
find_package(catkin REQUIRED
  message_generation
  std_msgs
  sensor_msgs
  pcl_conversions
  roscpp
  pcl_ros
  nodelet
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

#pcap
set(libpcap_LIBRARIES -lpcap)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${Boost_INCLUDE_DIR}
)
#yaml
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

#message
add_message_files(FILES
  OSI64.msg
  VLP16.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime message_generation std_msgs sensor_msgs pcl_conversions roscpp pcl_ros nodelet
  DEPENDS system_lib
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(nodelet_lidar_driver
  src/nodelet_midend.cpp
  src/calibration.cpp
  src/lidar.cpp
  src/packet.cpp
  src/nodelet_frontend.cpp
  src/nodelet_backend.cpp
)
target_link_libraries(nodelet_lidar_driver ${catkin_LIBRARIES} ${libpcap_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(nodelet_lidar_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_lidar_driver ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS nodelet_lidar_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES 
  nodelet_lidar_driver.xml 
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
