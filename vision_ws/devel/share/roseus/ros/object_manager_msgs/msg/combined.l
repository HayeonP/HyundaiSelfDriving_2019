;; Auto-generated. Do not edit!


(when (boundp 'object_manager_msgs::combined)
  (if (not (find-package "OBJECT_MANAGER_MSGS"))
    (make-package "OBJECT_MANAGER_MSGS"))
  (shadow 'combined (find-package "OBJECT_MANAGER_MSGS")))
(unless (find-package "OBJECT_MANAGER_MSGS::COMBINED")
  (make-package "OBJECT_MANAGER_MSGS::COMBINED"))

(in-package "ROS")
;;//! \htmlinclude combined.msg.html
(if (not (find-package "AUTOWARE_MSGS"))
  (ros::roseus-add-msgs "autoware_msgs"))
(if (not (find-package "JSK_RECOGNITION_MSGS"))
  (ros::roseus-add-msgs "jsk_recognition_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_manager_msgs::combined
  :super ros::object
  :slots (_header _bb_ary _obj_ary ))

(defmethod object_manager_msgs::combined
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bb_ary __bb_ary) (instance jsk_recognition_msgs::BoundingBoxArray :init))
    ((:obj_ary __obj_ary) (instance autoware_msgs::DetectedObjectArray :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _bb_ary __bb_ary)
   (setq _obj_ary __obj_ary)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bb_ary
   (&rest __bb_ary)
   (if (keywordp (car __bb_ary))
       (send* _bb_ary __bb_ary)
     (progn
       (if __bb_ary (setq _bb_ary (car __bb_ary)))
       _bb_ary)))
  (:obj_ary
   (&rest __obj_ary)
   (if (keywordp (car __obj_ary))
       (send* _obj_ary __obj_ary)
     (progn
       (if __obj_ary (setq _obj_ary (car __obj_ary)))
       _obj_ary)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/BoundingBoxArray _bb_ary
    (send _bb_ary :serialization-length)
    ;; autoware_msgs/DetectedObjectArray _obj_ary
    (send _obj_ary :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/BoundingBoxArray _bb_ary
       (send _bb_ary :serialize s)
     ;; autoware_msgs/DetectedObjectArray _obj_ary
       (send _obj_ary :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/BoundingBoxArray _bb_ary
     (send _bb_ary :deserialize buf ptr-) (incf ptr- (send _bb_ary :serialization-length))
   ;; autoware_msgs/DetectedObjectArray _obj_ary
     (send _obj_ary :deserialize buf ptr-) (incf ptr- (send _obj_ary :serialization-length))
   ;;
   self)
  )

(setf (get object_manager_msgs::combined :md5sum-) "0913eb20e42983a953910130fae82640")
(setf (get object_manager_msgs::combined :datatype-) "object_manager_msgs/combined")
(setf (get object_manager_msgs::combined :definition-)
      "std_msgs/Header header
jsk_recognition_msgs/BoundingBoxArray bb_ary
autoware_msgs/DetectedObjectArray obj_ary
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: jsk_recognition_msgs/BoundingBoxArray
# BoundingBoxArray is a list of BoundingBox.
# You can use jsk_rviz_plugins to visualize BoungingBoxArray on rviz.
Header header
BoundingBox[] boxes

================================================================================
MSG: jsk_recognition_msgs/BoundingBox
# BoundingBox represents a oriented bounding box.
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
# You can use this field to hold value such as likelihood
float32 value
uint32 label

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: autoware_msgs/DetectedObjectArray
std_msgs/Header header
DetectedObject[] objects
================================================================================
MSG: autoware_msgs/DetectedObject
std_msgs/Header                 header

uint32                          id
string                          label
float32                         score   #Score as defined by the detection, Optional
std_msgs/ColorRGBA              color   # Define this object specific color
bool                            valid   # Defines if this object is valid, or invalid as defined by the filtering

################ 3D BB
string                          space_frame #3D Space coordinate frame of the object, required if pose and dimensions are defines
geometry_msgs/Pose              pose
geometry_msgs/Vector3           dimensions
geometry_msgs/Vector3           variance
geometry_msgs/Twist             velocity
geometry_msgs/Twist             acceleration

sensor_msgs/PointCloud2         pointcloud

geometry_msgs/PolygonStamped    convex_hull
autoware_msgs/LaneArray         candidate_trajectories

bool                            pose_reliable
bool                            velocity_reliable
bool                            acceleration_reliable

############### 2D Rect
string                          image_frame # Image coordinate Frame,        Required if x,y,w,h defined
int32                           x           # X coord in image space(pixel) of the initial point of the Rect
int32                           y           # Y coord in image space(pixel) of the initial point of the Rect
int32                           width       # Width of the Rect in pixels
int32                           height      # Height of the Rect in pixels
float32                         angle       # Angle [0 to 2*PI), allow rotated rects

sensor_msgs/Image               roi_image

############### Indicator information
uint8                          indicator_state # INDICATOR_LEFT = 0, INDICATOR_RIGHT = 1, INDICATOR_BOTH = 2, INDICATOR_NONE = 3

############### Behavior State of the Detected Object
uint8                           behavior_state # FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6

#
string[]                        user_defined_info
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: geometry_msgs/PolygonStamped
# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon

================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
================================================================================
MSG: autoware_msgs/LaneArray
Lane[] lanes
================================================================================
MSG: autoware_msgs/Lane
Header header
int32 increment
int32 lane_id
Waypoint[] waypoints

uint32 lane_index
float32 cost
float32 closest_object_distance
float32 closest_object_velocity
bool is_blocked

================================================================================
MSG: autoware_msgs/Waypoint
# global id
int32 gid 
# local id
int32 lid
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
DTLane dtlane
int32 change_flag
WaypointState wpstate

uint32 lane_id
uint32 left_lane_id
uint32 right_lane_id
uint32 stop_line_id
float32 cost
float32 time_cost

# Lane Direction
# FORWARD				= 0
# FORWARD_LEFT	 		= 1
# FORWARD_RIGHT			= 2
# BACKWARD				= 3 
# BACKWARD_LEFT			= 4
# BACKWARD_RIGHT		= 5
# STANDSTILL	 		= 6
uint32 direction
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/TwistStamped
# A twist with reference coordinate frame and timestamp
Header header
Twist twist

================================================================================
MSG: autoware_msgs/DTLane
float64 dist
float64 dir
float64 apara
float64 r
float64 slope
float64 cant
float64 lw
float64 rw

================================================================================
MSG: autoware_msgs/WaypointState
int32 aid
uint8 NULLSTATE=0
uint8 lanechange_state

uint8 steering_state
uint8 STR_LEFT=1
uint8 STR_RIGHT=2
uint8 STR_STRAIGHT=3

uint8 accel_state
uint8 stopline_state
  uint8 TYPE_NULL=0
  uint8 TYPE_STOPLINE=1
  uint8 TYPE_STOP=2
# 1 is stopline, 2 is stop which 2 can only be released manually.
uint64 event_state

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

")



(provide :object_manager_msgs/combined "0913eb20e42983a953910130fae82640")


