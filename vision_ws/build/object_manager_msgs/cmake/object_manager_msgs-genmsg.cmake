# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "object_manager_msgs: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iobject_manager_msgs:/home/autoware/vision_ws/src/object_manager_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Ijsk_recognition_msgs:/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg;-Iautoware_msgs:/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg;-Ipcl_msgs:/opt/ros/kinetic/share/pcl_msgs/cmake/../msg;-Ijsk_footstep_msgs:/opt/ros/kinetic/share/jsk_footstep_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(object_manager_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg" NAME_WE)
add_custom_target(_object_manager_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_manager_msgs" "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg" "std_msgs/ColorRGBA:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Polygon:geometry_msgs/Twist:jsk_recognition_msgs/BoundingBoxArray:autoware_msgs/Waypoint:geometry_msgs/Pose:geometry_msgs/Point32:autoware_msgs/Lane:autoware_msgs/DetectedObjectArray:jsk_recognition_msgs/BoundingBox:autoware_msgs/LaneArray:geometry_msgs/PolygonStamped:autoware_msgs/WaypointState:geometry_msgs/PoseStamped:autoware_msgs/DetectedObject:sensor_msgs/PointField:geometry_msgs/TwistStamped:geometry_msgs/Vector3:sensor_msgs/Image:sensor_msgs/PointCloud2:autoware_msgs/DTLane"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(object_manager_msgs
  "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Lane.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObjectArray.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DTLane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(object_manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_manager_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(object_manager_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(object_manager_msgs_generate_messages object_manager_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg" NAME_WE)
add_dependencies(object_manager_msgs_generate_messages_cpp _object_manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_manager_msgs_gencpp)
add_dependencies(object_manager_msgs_gencpp object_manager_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_manager_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(object_manager_msgs
  "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Lane.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObjectArray.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DTLane.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(object_manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_manager_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(object_manager_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(object_manager_msgs_generate_messages object_manager_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg" NAME_WE)
add_dependencies(object_manager_msgs_generate_messages_eus _object_manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_manager_msgs_geneus)
add_dependencies(object_manager_msgs_geneus object_manager_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_manager_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(object_manager_msgs
  "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Lane.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObjectArray.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DTLane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(object_manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_manager_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(object_manager_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(object_manager_msgs_generate_messages object_manager_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg" NAME_WE)
add_dependencies(object_manager_msgs_generate_messages_lisp _object_manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_manager_msgs_genlisp)
add_dependencies(object_manager_msgs_genlisp object_manager_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_manager_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(object_manager_msgs
  "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Lane.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObjectArray.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DTLane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(object_manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_manager_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(object_manager_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(object_manager_msgs_generate_messages object_manager_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg" NAME_WE)
add_dependencies(object_manager_msgs_generate_messages_nodejs _object_manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_manager_msgs_gennodejs)
add_dependencies(object_manager_msgs_gennodejs object_manager_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_manager_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(object_manager_msgs
  "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/Lane.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObjectArray.msg;/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/autoware/Autoware/ros/src/msgs/autoware_msgs/msg/DTLane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(object_manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_manager_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(object_manager_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(object_manager_msgs_generate_messages object_manager_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/vision_ws/src/object_manager_msgs/msg/combined.msg" NAME_WE)
add_dependencies(object_manager_msgs_generate_messages_py _object_manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_manager_msgs_genpy)
add_dependencies(object_manager_msgs_genpy object_manager_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_manager_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_manager_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(object_manager_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(object_manager_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(object_manager_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_cpp)
  add_dependencies(object_manager_msgs_generate_messages_cpp jsk_recognition_msgs_generate_messages_cpp)
endif()
if(TARGET autoware_msgs_generate_messages_cpp)
  add_dependencies(object_manager_msgs_generate_messages_cpp autoware_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_manager_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(object_manager_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(object_manager_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(object_manager_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_eus)
  add_dependencies(object_manager_msgs_generate_messages_eus jsk_recognition_msgs_generate_messages_eus)
endif()
if(TARGET autoware_msgs_generate_messages_eus)
  add_dependencies(object_manager_msgs_generate_messages_eus autoware_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_manager_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(object_manager_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(object_manager_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(object_manager_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_lisp)
  add_dependencies(object_manager_msgs_generate_messages_lisp jsk_recognition_msgs_generate_messages_lisp)
endif()
if(TARGET autoware_msgs_generate_messages_lisp)
  add_dependencies(object_manager_msgs_generate_messages_lisp autoware_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_manager_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(object_manager_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(object_manager_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(object_manager_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_nodejs)
  add_dependencies(object_manager_msgs_generate_messages_nodejs jsk_recognition_msgs_generate_messages_nodejs)
endif()
if(TARGET autoware_msgs_generate_messages_nodejs)
  add_dependencies(object_manager_msgs_generate_messages_nodejs autoware_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_manager_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_manager_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_manager_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(object_manager_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(object_manager_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(object_manager_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_py)
  add_dependencies(object_manager_msgs_generate_messages_py jsk_recognition_msgs_generate_messages_py)
endif()
if(TARGET autoware_msgs_generate_messages_py)
  add_dependencies(object_manager_msgs_generate_messages_py autoware_msgs_generate_messages_py)
endif()
