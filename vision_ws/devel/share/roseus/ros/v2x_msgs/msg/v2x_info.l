;; Auto-generated. Do not edit!


(when (boundp 'v2x_msgs::v2x_info)
  (if (not (find-package "V2X_MSGS"))
    (make-package "V2X_MSGS"))
  (shadow 'v2x_info (find-package "V2X_MSGS")))
(unless (find-package "V2X_MSGS::V2X_INFO")
  (make-package "V2X_MSGS::V2X_INFO"))

(in-package "ROS")
;;//! \htmlinclude v2x_info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*BSM_MSG_TYPE*" (find-package "V2X_MSGS::V2X_INFO"))
(shadow '*BSM_MSG_TYPE* (find-package "V2X_MSGS::V2X_INFO"))
(defconstant v2x_msgs::v2x_info::*BSM_MSG_TYPE* 1)
(intern "*SPAT_MSG_TYPE*" (find-package "V2X_MSGS::V2X_INFO"))
(shadow '*SPAT_MSG_TYPE* (find-package "V2X_MSGS::V2X_INFO"))
(defconstant v2x_msgs::v2x_info::*SPAT_MSG_TYPE* 2)
(intern "*TIM_MSG_TYPE*" (find-package "V2X_MSGS::V2X_INFO"))
(shadow '*TIM_MSG_TYPE* (find-package "V2X_MSGS::V2X_INFO"))
(defconstant v2x_msgs::v2x_info::*TIM_MSG_TYPE* 3)
(intern "*MAP_MSG_TYPE*" (find-package "V2X_MSGS::V2X_INFO"))
(shadow '*MAP_MSG_TYPE* (find-package "V2X_MSGS::V2X_INFO"))
(defconstant v2x_msgs::v2x_info::*MAP_MSG_TYPE* 4)
(defclass v2x_msgs::v2x_info
  :super ros::object
  :slots (_header _msg_type _map_id_region _map_refpoint_lat _map_refpoint_lon _map_speed_limit _map_g_cnt _map_g_id _map_g_nodelist_cnt _map_g_nodelist_xy _map_g_connectsto_cnt _map_g_connectsto_lane _spat_id_region _spat_movement_cnt _spat_movement_name _spat_eventstate _spat_minendtime _bsm_id _bsm_lat _bsm_lon _bsm_angle _bsm_size_width _bsm_size_length _bsm_classification _tim_dataframe_cnt _tim_starttime _tim_durationtime _tim_anchor_lat _tim_anchor_lon _tim_lanewidth _tim_direction _tim_nodelist_xy_cnt _tim_nodelist_xy_latlon _tim_content _tim_speedlimit ))

(defmethod v2x_msgs::v2x_info
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:msg_type __msg_type) 0)
    ((:map_id_region __map_id_region) 0)
    ((:map_refpoint_lat __map_refpoint_lat) 0)
    ((:map_refpoint_lon __map_refpoint_lon) 0)
    ((:map_speed_limit __map_speed_limit) 0)
    ((:map_g_cnt __map_g_cnt) 0)
    ((:map_g_id __map_g_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:map_g_nodelist_cnt __map_g_nodelist_cnt) (make-array 0 :initial-element 0 :element-type :integer))
    ((:map_g_nodelist_xy __map_g_nodelist_xy) (make-array 0 :initial-element 0 :element-type :integer))
    ((:map_g_connectsto_cnt __map_g_connectsto_cnt) (make-array 0 :initial-element 0 :element-type :integer))
    ((:map_g_connectsto_lane __map_g_connectsto_lane) (make-array 0 :initial-element 0 :element-type :integer))
    ((:spat_id_region __spat_id_region) 0)
    ((:spat_movement_cnt __spat_movement_cnt) 0)
    ((:spat_movement_name __spat_movement_name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:spat_eventstate __spat_eventstate) (make-array 0 :initial-element 0 :element-type :integer))
    ((:spat_minendtime __spat_minendtime) (make-array 0 :initial-element 0 :element-type :integer))
    ((:bsm_id __bsm_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:bsm_lat __bsm_lat) 0)
    ((:bsm_lon __bsm_lon) 0)
    ((:bsm_angle __bsm_angle) 0)
    ((:bsm_size_width __bsm_size_width) 0)
    ((:bsm_size_length __bsm_size_length) 0)
    ((:bsm_classification __bsm_classification) 0)
    ((:tim_dataframe_cnt __tim_dataframe_cnt) 0)
    ((:tim_starttime __tim_starttime) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_durationtime __tim_durationtime) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_anchor_lat __tim_anchor_lat) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_anchor_lon __tim_anchor_lon) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_lanewidth __tim_lanewidth) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_direction __tim_direction) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_nodelist_xy_cnt __tim_nodelist_xy_cnt) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_nodelist_xy_latlon __tim_nodelist_xy_latlon) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_content __tim_content) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tim_speedlimit __tim_speedlimit) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _msg_type (round __msg_type))
   (setq _map_id_region (round __map_id_region))
   (setq _map_refpoint_lat (round __map_refpoint_lat))
   (setq _map_refpoint_lon (round __map_refpoint_lon))
   (setq _map_speed_limit (round __map_speed_limit))
   (setq _map_g_cnt (round __map_g_cnt))
   (setq _map_g_id __map_g_id)
   (setq _map_g_nodelist_cnt __map_g_nodelist_cnt)
   (setq _map_g_nodelist_xy __map_g_nodelist_xy)
   (setq _map_g_connectsto_cnt __map_g_connectsto_cnt)
   (setq _map_g_connectsto_lane __map_g_connectsto_lane)
   (setq _spat_id_region (round __spat_id_region))
   (setq _spat_movement_cnt (round __spat_movement_cnt))
   (setq _spat_movement_name __spat_movement_name)
   (setq _spat_eventstate __spat_eventstate)
   (setq _spat_minendtime __spat_minendtime)
   (setq _bsm_id __bsm_id)
   (setq _bsm_lat (round __bsm_lat))
   (setq _bsm_lon (round __bsm_lon))
   (setq _bsm_angle (round __bsm_angle))
   (setq _bsm_size_width (round __bsm_size_width))
   (setq _bsm_size_length (round __bsm_size_length))
   (setq _bsm_classification (round __bsm_classification))
   (setq _tim_dataframe_cnt (round __tim_dataframe_cnt))
   (setq _tim_starttime __tim_starttime)
   (setq _tim_durationtime __tim_durationtime)
   (setq _tim_anchor_lat __tim_anchor_lat)
   (setq _tim_anchor_lon __tim_anchor_lon)
   (setq _tim_lanewidth __tim_lanewidth)
   (setq _tim_direction __tim_direction)
   (setq _tim_nodelist_xy_cnt __tim_nodelist_xy_cnt)
   (setq _tim_nodelist_xy_latlon __tim_nodelist_xy_latlon)
   (setq _tim_content __tim_content)
   (setq _tim_speedlimit __tim_speedlimit)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:msg_type
   (&optional __msg_type)
   (if __msg_type (setq _msg_type __msg_type)) _msg_type)
  (:map_id_region
   (&optional __map_id_region)
   (if __map_id_region (setq _map_id_region __map_id_region)) _map_id_region)
  (:map_refpoint_lat
   (&optional __map_refpoint_lat)
   (if __map_refpoint_lat (setq _map_refpoint_lat __map_refpoint_lat)) _map_refpoint_lat)
  (:map_refpoint_lon
   (&optional __map_refpoint_lon)
   (if __map_refpoint_lon (setq _map_refpoint_lon __map_refpoint_lon)) _map_refpoint_lon)
  (:map_speed_limit
   (&optional __map_speed_limit)
   (if __map_speed_limit (setq _map_speed_limit __map_speed_limit)) _map_speed_limit)
  (:map_g_cnt
   (&optional __map_g_cnt)
   (if __map_g_cnt (setq _map_g_cnt __map_g_cnt)) _map_g_cnt)
  (:map_g_id
   (&optional __map_g_id)
   (if __map_g_id (setq _map_g_id __map_g_id)) _map_g_id)
  (:map_g_nodelist_cnt
   (&optional __map_g_nodelist_cnt)
   (if __map_g_nodelist_cnt (setq _map_g_nodelist_cnt __map_g_nodelist_cnt)) _map_g_nodelist_cnt)
  (:map_g_nodelist_xy
   (&optional __map_g_nodelist_xy)
   (if __map_g_nodelist_xy (setq _map_g_nodelist_xy __map_g_nodelist_xy)) _map_g_nodelist_xy)
  (:map_g_connectsto_cnt
   (&optional __map_g_connectsto_cnt)
   (if __map_g_connectsto_cnt (setq _map_g_connectsto_cnt __map_g_connectsto_cnt)) _map_g_connectsto_cnt)
  (:map_g_connectsto_lane
   (&optional __map_g_connectsto_lane)
   (if __map_g_connectsto_lane (setq _map_g_connectsto_lane __map_g_connectsto_lane)) _map_g_connectsto_lane)
  (:spat_id_region
   (&optional __spat_id_region)
   (if __spat_id_region (setq _spat_id_region __spat_id_region)) _spat_id_region)
  (:spat_movement_cnt
   (&optional __spat_movement_cnt)
   (if __spat_movement_cnt (setq _spat_movement_cnt __spat_movement_cnt)) _spat_movement_cnt)
  (:spat_movement_name
   (&optional __spat_movement_name)
   (if __spat_movement_name (setq _spat_movement_name __spat_movement_name)) _spat_movement_name)
  (:spat_eventstate
   (&optional __spat_eventstate)
   (if __spat_eventstate (setq _spat_eventstate __spat_eventstate)) _spat_eventstate)
  (:spat_minendtime
   (&optional __spat_minendtime)
   (if __spat_minendtime (setq _spat_minendtime __spat_minendtime)) _spat_minendtime)
  (:bsm_id
   (&optional __bsm_id)
   (if __bsm_id (setq _bsm_id __bsm_id)) _bsm_id)
  (:bsm_lat
   (&optional __bsm_lat)
   (if __bsm_lat (setq _bsm_lat __bsm_lat)) _bsm_lat)
  (:bsm_lon
   (&optional __bsm_lon)
   (if __bsm_lon (setq _bsm_lon __bsm_lon)) _bsm_lon)
  (:bsm_angle
   (&optional __bsm_angle)
   (if __bsm_angle (setq _bsm_angle __bsm_angle)) _bsm_angle)
  (:bsm_size_width
   (&optional __bsm_size_width)
   (if __bsm_size_width (setq _bsm_size_width __bsm_size_width)) _bsm_size_width)
  (:bsm_size_length
   (&optional __bsm_size_length)
   (if __bsm_size_length (setq _bsm_size_length __bsm_size_length)) _bsm_size_length)
  (:bsm_classification
   (&optional __bsm_classification)
   (if __bsm_classification (setq _bsm_classification __bsm_classification)) _bsm_classification)
  (:tim_dataframe_cnt
   (&optional __tim_dataframe_cnt)
   (if __tim_dataframe_cnt (setq _tim_dataframe_cnt __tim_dataframe_cnt)) _tim_dataframe_cnt)
  (:tim_starttime
   (&optional __tim_starttime)
   (if __tim_starttime (setq _tim_starttime __tim_starttime)) _tim_starttime)
  (:tim_durationtime
   (&optional __tim_durationtime)
   (if __tim_durationtime (setq _tim_durationtime __tim_durationtime)) _tim_durationtime)
  (:tim_anchor_lat
   (&optional __tim_anchor_lat)
   (if __tim_anchor_lat (setq _tim_anchor_lat __tim_anchor_lat)) _tim_anchor_lat)
  (:tim_anchor_lon
   (&optional __tim_anchor_lon)
   (if __tim_anchor_lon (setq _tim_anchor_lon __tim_anchor_lon)) _tim_anchor_lon)
  (:tim_lanewidth
   (&optional __tim_lanewidth)
   (if __tim_lanewidth (setq _tim_lanewidth __tim_lanewidth)) _tim_lanewidth)
  (:tim_direction
   (&optional __tim_direction)
   (if __tim_direction (setq _tim_direction __tim_direction)) _tim_direction)
  (:tim_nodelist_xy_cnt
   (&optional __tim_nodelist_xy_cnt)
   (if __tim_nodelist_xy_cnt (setq _tim_nodelist_xy_cnt __tim_nodelist_xy_cnt)) _tim_nodelist_xy_cnt)
  (:tim_nodelist_xy_latlon
   (&optional __tim_nodelist_xy_latlon)
   (if __tim_nodelist_xy_latlon (setq _tim_nodelist_xy_latlon __tim_nodelist_xy_latlon)) _tim_nodelist_xy_latlon)
  (:tim_content
   (&optional __tim_content)
   (if __tim_content (setq _tim_content __tim_content)) _tim_content)
  (:tim_speedlimit
   (&optional __tim_speedlimit)
   (if __tim_speedlimit (setq _tim_speedlimit __tim_speedlimit)) _tim_speedlimit)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _msg_type
    4
    ;; int32 _map_id_region
    4
    ;; int32 _map_refpoint_lat
    4
    ;; int32 _map_refpoint_lon
    4
    ;; int32 _map_speed_limit
    4
    ;; int32 _map_g_cnt
    4
    ;; int32[] _map_g_id
    (* 4    (length _map_g_id)) 4
    ;; int32[] _map_g_nodelist_cnt
    (* 4    (length _map_g_nodelist_cnt)) 4
    ;; int32[] _map_g_nodelist_xy
    (* 4    (length _map_g_nodelist_xy)) 4
    ;; int32[] _map_g_connectsto_cnt
    (* 4    (length _map_g_connectsto_cnt)) 4
    ;; int32[] _map_g_connectsto_lane
    (* 4    (length _map_g_connectsto_lane)) 4
    ;; int32 _spat_id_region
    4
    ;; int32 _spat_movement_cnt
    4
    ;; string[] _spat_movement_name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _spat_movement_name)) 4
    ;; int32[] _spat_eventstate
    (* 4    (length _spat_eventstate)) 4
    ;; int32[] _spat_minendtime
    (* 4    (length _spat_minendtime)) 4
    ;; int32[] _bsm_id
    (* 4    (length _bsm_id)) 4
    ;; int32 _bsm_lat
    4
    ;; int32 _bsm_lon
    4
    ;; int32 _bsm_angle
    4
    ;; int32 _bsm_size_width
    4
    ;; int32 _bsm_size_length
    4
    ;; int32 _bsm_classification
    4
    ;; int32 _tim_dataframe_cnt
    4
    ;; int32[] _tim_starttime
    (* 4    (length _tim_starttime)) 4
    ;; int32[] _tim_durationtime
    (* 4    (length _tim_durationtime)) 4
    ;; int32[] _tim_anchor_lat
    (* 4    (length _tim_anchor_lat)) 4
    ;; int32[] _tim_anchor_lon
    (* 4    (length _tim_anchor_lon)) 4
    ;; int32[] _tim_lanewidth
    (* 4    (length _tim_lanewidth)) 4
    ;; int32[] _tim_direction
    (* 4    (length _tim_direction)) 4
    ;; int32[] _tim_nodelist_xy_cnt
    (* 4    (length _tim_nodelist_xy_cnt)) 4
    ;; int32[] _tim_nodelist_xy_latlon
    (* 4    (length _tim_nodelist_xy_latlon)) 4
    ;; int32[] _tim_content
    (* 4    (length _tim_content)) 4
    ;; int32[] _tim_speedlimit
    (* 4    (length _tim_speedlimit)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _msg_type
       (write-long _msg_type s)
     ;; int32 _map_id_region
       (write-long _map_id_region s)
     ;; int32 _map_refpoint_lat
       (write-long _map_refpoint_lat s)
     ;; int32 _map_refpoint_lon
       (write-long _map_refpoint_lon s)
     ;; int32 _map_speed_limit
       (write-long _map_speed_limit s)
     ;; int32 _map_g_cnt
       (write-long _map_g_cnt s)
     ;; int32[] _map_g_id
     (write-long (length _map_g_id) s)
     (dotimes (i (length _map_g_id))
       (write-long (elt _map_g_id i) s)
       )
     ;; int32[] _map_g_nodelist_cnt
     (write-long (length _map_g_nodelist_cnt) s)
     (dotimes (i (length _map_g_nodelist_cnt))
       (write-long (elt _map_g_nodelist_cnt i) s)
       )
     ;; int32[] _map_g_nodelist_xy
     (write-long (length _map_g_nodelist_xy) s)
     (dotimes (i (length _map_g_nodelist_xy))
       (write-long (elt _map_g_nodelist_xy i) s)
       )
     ;; int32[] _map_g_connectsto_cnt
     (write-long (length _map_g_connectsto_cnt) s)
     (dotimes (i (length _map_g_connectsto_cnt))
       (write-long (elt _map_g_connectsto_cnt i) s)
       )
     ;; int32[] _map_g_connectsto_lane
     (write-long (length _map_g_connectsto_lane) s)
     (dotimes (i (length _map_g_connectsto_lane))
       (write-long (elt _map_g_connectsto_lane i) s)
       )
     ;; int32 _spat_id_region
       (write-long _spat_id_region s)
     ;; int32 _spat_movement_cnt
       (write-long _spat_movement_cnt s)
     ;; string[] _spat_movement_name
     (write-long (length _spat_movement_name) s)
     (dolist (elem _spat_movement_name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; int32[] _spat_eventstate
     (write-long (length _spat_eventstate) s)
     (dotimes (i (length _spat_eventstate))
       (write-long (elt _spat_eventstate i) s)
       )
     ;; int32[] _spat_minendtime
     (write-long (length _spat_minendtime) s)
     (dotimes (i (length _spat_minendtime))
       (write-long (elt _spat_minendtime i) s)
       )
     ;; int32[] _bsm_id
     (write-long (length _bsm_id) s)
     (dotimes (i (length _bsm_id))
       (write-long (elt _bsm_id i) s)
       )
     ;; int32 _bsm_lat
       (write-long _bsm_lat s)
     ;; int32 _bsm_lon
       (write-long _bsm_lon s)
     ;; int32 _bsm_angle
       (write-long _bsm_angle s)
     ;; int32 _bsm_size_width
       (write-long _bsm_size_width s)
     ;; int32 _bsm_size_length
       (write-long _bsm_size_length s)
     ;; int32 _bsm_classification
       (write-long _bsm_classification s)
     ;; int32 _tim_dataframe_cnt
       (write-long _tim_dataframe_cnt s)
     ;; int32[] _tim_starttime
     (write-long (length _tim_starttime) s)
     (dotimes (i (length _tim_starttime))
       (write-long (elt _tim_starttime i) s)
       )
     ;; int32[] _tim_durationtime
     (write-long (length _tim_durationtime) s)
     (dotimes (i (length _tim_durationtime))
       (write-long (elt _tim_durationtime i) s)
       )
     ;; int32[] _tim_anchor_lat
     (write-long (length _tim_anchor_lat) s)
     (dotimes (i (length _tim_anchor_lat))
       (write-long (elt _tim_anchor_lat i) s)
       )
     ;; int32[] _tim_anchor_lon
     (write-long (length _tim_anchor_lon) s)
     (dotimes (i (length _tim_anchor_lon))
       (write-long (elt _tim_anchor_lon i) s)
       )
     ;; int32[] _tim_lanewidth
     (write-long (length _tim_lanewidth) s)
     (dotimes (i (length _tim_lanewidth))
       (write-long (elt _tim_lanewidth i) s)
       )
     ;; int32[] _tim_direction
     (write-long (length _tim_direction) s)
     (dotimes (i (length _tim_direction))
       (write-long (elt _tim_direction i) s)
       )
     ;; int32[] _tim_nodelist_xy_cnt
     (write-long (length _tim_nodelist_xy_cnt) s)
     (dotimes (i (length _tim_nodelist_xy_cnt))
       (write-long (elt _tim_nodelist_xy_cnt i) s)
       )
     ;; int32[] _tim_nodelist_xy_latlon
     (write-long (length _tim_nodelist_xy_latlon) s)
     (dotimes (i (length _tim_nodelist_xy_latlon))
       (write-long (elt _tim_nodelist_xy_latlon i) s)
       )
     ;; int32[] _tim_content
     (write-long (length _tim_content) s)
     (dotimes (i (length _tim_content))
       (write-long (elt _tim_content i) s)
       )
     ;; int32[] _tim_speedlimit
     (write-long (length _tim_speedlimit) s)
     (dotimes (i (length _tim_speedlimit))
       (write-long (elt _tim_speedlimit i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _msg_type
     (setq _msg_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _map_id_region
     (setq _map_id_region (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _map_refpoint_lat
     (setq _map_refpoint_lat (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _map_refpoint_lon
     (setq _map_refpoint_lon (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _map_speed_limit
     (setq _map_speed_limit (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _map_g_cnt
     (setq _map_g_cnt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32[] _map_g_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_g_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _map_g_id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _map_g_nodelist_cnt
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_g_nodelist_cnt (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _map_g_nodelist_cnt i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _map_g_nodelist_xy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_g_nodelist_xy (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _map_g_nodelist_xy i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _map_g_connectsto_cnt
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_g_connectsto_cnt (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _map_g_connectsto_cnt i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _map_g_connectsto_lane
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_g_connectsto_lane (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _map_g_connectsto_lane i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _spat_id_region
     (setq _spat_id_region (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _spat_movement_cnt
     (setq _spat_movement_cnt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _spat_movement_name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _spat_movement_name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _spat_movement_name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; int32[] _spat_eventstate
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _spat_eventstate (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _spat_eventstate i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _spat_minendtime
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _spat_minendtime (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _spat_minendtime i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _bsm_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bsm_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _bsm_id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _bsm_lat
     (setq _bsm_lat (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bsm_lon
     (setq _bsm_lon (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bsm_angle
     (setq _bsm_angle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bsm_size_width
     (setq _bsm_size_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bsm_size_length
     (setq _bsm_size_length (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bsm_classification
     (setq _bsm_classification (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _tim_dataframe_cnt
     (setq _tim_dataframe_cnt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32[] _tim_starttime
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_starttime (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_starttime i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_durationtime
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_durationtime (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_durationtime i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_anchor_lat
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_anchor_lat (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_anchor_lat i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_anchor_lon
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_anchor_lon (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_anchor_lon i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_lanewidth
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_lanewidth (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_lanewidth i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_direction
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_direction (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_direction i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_nodelist_xy_cnt
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_nodelist_xy_cnt (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_nodelist_xy_cnt i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_nodelist_xy_latlon
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_nodelist_xy_latlon (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_nodelist_xy_latlon i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_content
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_content (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_content i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _tim_speedlimit
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tim_speedlimit (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tim_speedlimit i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get v2x_msgs::v2x_info :md5sum-) "0d901339c101aa3472bf57a8feba3d77")
(setf (get v2x_msgs::v2x_info :datatype-) "v2x_msgs/v2x_info")
(setf (get v2x_msgs::v2x_info :definition-)
      "Header header

int32 msg_type
int32 BSM_MSG_TYPE = 1
int32 SPAT_MSG_TYPE = 2
int32 TIM_MSG_TYPE = 3
int32 MAP_MSG_TYPE = 4
# map
int32 map_id_region
int32 map_refpoint_lat
int32 map_refpoint_lon
int32 map_speed_limit

int32 map_g_cnt
int32[] map_g_id
int32[] map_g_nodelist_cnt
int32[] map_g_nodelist_xy #xy, xy, xy...
int32[] map_g_connectsto_cnt
int32[] map_g_connectsto_lane

# SPaT
int32 spat_id_region
int32 spat_movement_cnt
string[] spat_movement_name # assume that movement state contains only one movement event
int32[] spat_eventstate #0 : unavaliable/ 3: stop and remain/ 5 : permissive_movement_allowed
int32[] spat_minendtime
    
# BSM
int32[] bsm_id
int32 bsm_lat
int32 bsm_lon
int32 bsm_angle
int32 bsm_size_width
int32 bsm_size_length
int32 bsm_classification #69 for ambulance, 0 for else

# TIM
int32 tim_dataframe_cnt
int32[] tim_starttime
int32[] tim_durationtime
int32[] tim_anchor_lat
int32[] tim_anchor_lon
int32[] tim_lanewidth
int32[] tim_direction
int32[] tim_nodelist_xy_cnt
int32[] tim_nodelist_xy_latlon # lat, lon, ...
int32[] tim_content
int32[] tim_speedlimit
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

")



(provide :v2x_msgs/v2x_info "0d901339c101aa3472bf57a8feba3d77")


