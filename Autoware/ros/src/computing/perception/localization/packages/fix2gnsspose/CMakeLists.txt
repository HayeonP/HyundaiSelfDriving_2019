cmake_minimum_required(VERSION 2.8.3)
project(fix2gnsspose)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	tf
	geometry_msgs
)
find_package(GeographicLib REQUIRED)

catkin_package(
 LIBRARIES fix2gnsspose
 CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs
 DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )

include_directories(
 ${GeographicLib_INCLUDE_DIRS}
 )

add_executable(fix2gnsspose src/fix2gnsspose.cpp)

target_link_libraries(fix2gnsspose ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${GeographicLib_DEFINITIONS})
