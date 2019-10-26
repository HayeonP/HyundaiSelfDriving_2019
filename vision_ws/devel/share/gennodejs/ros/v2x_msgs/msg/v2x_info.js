// Auto-generated. Do not edit!

// (in-package v2x_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class v2x_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.msg_type = null;
      this.map_id_region = null;
      this.map_refpoint_lat = null;
      this.map_refpoint_lon = null;
      this.map_speed_limit = null;
      this.map_g_cnt = null;
      this.map_g_id = null;
      this.map_g_nodelist_cnt = null;
      this.map_g_nodelist_xy = null;
      this.map_g_connectsto_cnt = null;
      this.map_g_connectsto_lane = null;
      this.spat_id_region = null;
      this.spat_movement_cnt = null;
      this.spat_movement_name = null;
      this.spat_eventstate = null;
      this.spat_minendtime = null;
      this.bsm_id = null;
      this.bsm_lat = null;
      this.bsm_lon = null;
      this.bsm_angle = null;
      this.bsm_size_width = null;
      this.bsm_size_length = null;
      this.bsm_classification = null;
      this.tim_dataframe_cnt = null;
      this.tim_starttime = null;
      this.tim_durationtime = null;
      this.tim_anchor_lat = null;
      this.tim_anchor_lon = null;
      this.tim_lanewidth = null;
      this.tim_direction = null;
      this.tim_nodelist_xy_cnt = null;
      this.tim_nodelist_xy_latlon = null;
      this.tim_content = null;
      this.tim_speedlimit = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('msg_type')) {
        this.msg_type = initObj.msg_type
      }
      else {
        this.msg_type = 0;
      }
      if (initObj.hasOwnProperty('map_id_region')) {
        this.map_id_region = initObj.map_id_region
      }
      else {
        this.map_id_region = 0;
      }
      if (initObj.hasOwnProperty('map_refpoint_lat')) {
        this.map_refpoint_lat = initObj.map_refpoint_lat
      }
      else {
        this.map_refpoint_lat = 0;
      }
      if (initObj.hasOwnProperty('map_refpoint_lon')) {
        this.map_refpoint_lon = initObj.map_refpoint_lon
      }
      else {
        this.map_refpoint_lon = 0;
      }
      if (initObj.hasOwnProperty('map_speed_limit')) {
        this.map_speed_limit = initObj.map_speed_limit
      }
      else {
        this.map_speed_limit = 0;
      }
      if (initObj.hasOwnProperty('map_g_cnt')) {
        this.map_g_cnt = initObj.map_g_cnt
      }
      else {
        this.map_g_cnt = 0;
      }
      if (initObj.hasOwnProperty('map_g_id')) {
        this.map_g_id = initObj.map_g_id
      }
      else {
        this.map_g_id = [];
      }
      if (initObj.hasOwnProperty('map_g_nodelist_cnt')) {
        this.map_g_nodelist_cnt = initObj.map_g_nodelist_cnt
      }
      else {
        this.map_g_nodelist_cnt = [];
      }
      if (initObj.hasOwnProperty('map_g_nodelist_xy')) {
        this.map_g_nodelist_xy = initObj.map_g_nodelist_xy
      }
      else {
        this.map_g_nodelist_xy = [];
      }
      if (initObj.hasOwnProperty('map_g_connectsto_cnt')) {
        this.map_g_connectsto_cnt = initObj.map_g_connectsto_cnt
      }
      else {
        this.map_g_connectsto_cnt = [];
      }
      if (initObj.hasOwnProperty('map_g_connectsto_lane')) {
        this.map_g_connectsto_lane = initObj.map_g_connectsto_lane
      }
      else {
        this.map_g_connectsto_lane = [];
      }
      if (initObj.hasOwnProperty('spat_id_region')) {
        this.spat_id_region = initObj.spat_id_region
      }
      else {
        this.spat_id_region = 0;
      }
      if (initObj.hasOwnProperty('spat_movement_cnt')) {
        this.spat_movement_cnt = initObj.spat_movement_cnt
      }
      else {
        this.spat_movement_cnt = 0;
      }
      if (initObj.hasOwnProperty('spat_movement_name')) {
        this.spat_movement_name = initObj.spat_movement_name
      }
      else {
        this.spat_movement_name = [];
      }
      if (initObj.hasOwnProperty('spat_eventstate')) {
        this.spat_eventstate = initObj.spat_eventstate
      }
      else {
        this.spat_eventstate = [];
      }
      if (initObj.hasOwnProperty('spat_minendtime')) {
        this.spat_minendtime = initObj.spat_minendtime
      }
      else {
        this.spat_minendtime = [];
      }
      if (initObj.hasOwnProperty('bsm_id')) {
        this.bsm_id = initObj.bsm_id
      }
      else {
        this.bsm_id = [];
      }
      if (initObj.hasOwnProperty('bsm_lat')) {
        this.bsm_lat = initObj.bsm_lat
      }
      else {
        this.bsm_lat = 0;
      }
      if (initObj.hasOwnProperty('bsm_lon')) {
        this.bsm_lon = initObj.bsm_lon
      }
      else {
        this.bsm_lon = 0;
      }
      if (initObj.hasOwnProperty('bsm_angle')) {
        this.bsm_angle = initObj.bsm_angle
      }
      else {
        this.bsm_angle = 0;
      }
      if (initObj.hasOwnProperty('bsm_size_width')) {
        this.bsm_size_width = initObj.bsm_size_width
      }
      else {
        this.bsm_size_width = 0;
      }
      if (initObj.hasOwnProperty('bsm_size_length')) {
        this.bsm_size_length = initObj.bsm_size_length
      }
      else {
        this.bsm_size_length = 0;
      }
      if (initObj.hasOwnProperty('bsm_classification')) {
        this.bsm_classification = initObj.bsm_classification
      }
      else {
        this.bsm_classification = 0;
      }
      if (initObj.hasOwnProperty('tim_dataframe_cnt')) {
        this.tim_dataframe_cnt = initObj.tim_dataframe_cnt
      }
      else {
        this.tim_dataframe_cnt = 0;
      }
      if (initObj.hasOwnProperty('tim_starttime')) {
        this.tim_starttime = initObj.tim_starttime
      }
      else {
        this.tim_starttime = [];
      }
      if (initObj.hasOwnProperty('tim_durationtime')) {
        this.tim_durationtime = initObj.tim_durationtime
      }
      else {
        this.tim_durationtime = [];
      }
      if (initObj.hasOwnProperty('tim_anchor_lat')) {
        this.tim_anchor_lat = initObj.tim_anchor_lat
      }
      else {
        this.tim_anchor_lat = [];
      }
      if (initObj.hasOwnProperty('tim_anchor_lon')) {
        this.tim_anchor_lon = initObj.tim_anchor_lon
      }
      else {
        this.tim_anchor_lon = [];
      }
      if (initObj.hasOwnProperty('tim_lanewidth')) {
        this.tim_lanewidth = initObj.tim_lanewidth
      }
      else {
        this.tim_lanewidth = [];
      }
      if (initObj.hasOwnProperty('tim_direction')) {
        this.tim_direction = initObj.tim_direction
      }
      else {
        this.tim_direction = [];
      }
      if (initObj.hasOwnProperty('tim_nodelist_xy_cnt')) {
        this.tim_nodelist_xy_cnt = initObj.tim_nodelist_xy_cnt
      }
      else {
        this.tim_nodelist_xy_cnt = [];
      }
      if (initObj.hasOwnProperty('tim_nodelist_xy_latlon')) {
        this.tim_nodelist_xy_latlon = initObj.tim_nodelist_xy_latlon
      }
      else {
        this.tim_nodelist_xy_latlon = [];
      }
      if (initObj.hasOwnProperty('tim_content')) {
        this.tim_content = initObj.tim_content
      }
      else {
        this.tim_content = [];
      }
      if (initObj.hasOwnProperty('tim_speedlimit')) {
        this.tim_speedlimit = initObj.tim_speedlimit
      }
      else {
        this.tim_speedlimit = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type v2x_info
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [msg_type]
    bufferOffset = _serializer.int32(obj.msg_type, buffer, bufferOffset);
    // Serialize message field [map_id_region]
    bufferOffset = _serializer.int32(obj.map_id_region, buffer, bufferOffset);
    // Serialize message field [map_refpoint_lat]
    bufferOffset = _serializer.int32(obj.map_refpoint_lat, buffer, bufferOffset);
    // Serialize message field [map_refpoint_lon]
    bufferOffset = _serializer.int32(obj.map_refpoint_lon, buffer, bufferOffset);
    // Serialize message field [map_speed_limit]
    bufferOffset = _serializer.int32(obj.map_speed_limit, buffer, bufferOffset);
    // Serialize message field [map_g_cnt]
    bufferOffset = _serializer.int32(obj.map_g_cnt, buffer, bufferOffset);
    // Serialize message field [map_g_id]
    bufferOffset = _arraySerializer.int32(obj.map_g_id, buffer, bufferOffset, null);
    // Serialize message field [map_g_nodelist_cnt]
    bufferOffset = _arraySerializer.int32(obj.map_g_nodelist_cnt, buffer, bufferOffset, null);
    // Serialize message field [map_g_nodelist_xy]
    bufferOffset = _arraySerializer.int32(obj.map_g_nodelist_xy, buffer, bufferOffset, null);
    // Serialize message field [map_g_connectsto_cnt]
    bufferOffset = _arraySerializer.int32(obj.map_g_connectsto_cnt, buffer, bufferOffset, null);
    // Serialize message field [map_g_connectsto_lane]
    bufferOffset = _arraySerializer.int32(obj.map_g_connectsto_lane, buffer, bufferOffset, null);
    // Serialize message field [spat_id_region]
    bufferOffset = _serializer.int32(obj.spat_id_region, buffer, bufferOffset);
    // Serialize message field [spat_movement_cnt]
    bufferOffset = _serializer.int32(obj.spat_movement_cnt, buffer, bufferOffset);
    // Serialize message field [spat_movement_name]
    bufferOffset = _arraySerializer.string(obj.spat_movement_name, buffer, bufferOffset, null);
    // Serialize message field [spat_eventstate]
    bufferOffset = _arraySerializer.int32(obj.spat_eventstate, buffer, bufferOffset, null);
    // Serialize message field [spat_minendtime]
    bufferOffset = _arraySerializer.int32(obj.spat_minendtime, buffer, bufferOffset, null);
    // Serialize message field [bsm_id]
    bufferOffset = _arraySerializer.int32(obj.bsm_id, buffer, bufferOffset, null);
    // Serialize message field [bsm_lat]
    bufferOffset = _serializer.int32(obj.bsm_lat, buffer, bufferOffset);
    // Serialize message field [bsm_lon]
    bufferOffset = _serializer.int32(obj.bsm_lon, buffer, bufferOffset);
    // Serialize message field [bsm_angle]
    bufferOffset = _serializer.int32(obj.bsm_angle, buffer, bufferOffset);
    // Serialize message field [bsm_size_width]
    bufferOffset = _serializer.int32(obj.bsm_size_width, buffer, bufferOffset);
    // Serialize message field [bsm_size_length]
    bufferOffset = _serializer.int32(obj.bsm_size_length, buffer, bufferOffset);
    // Serialize message field [bsm_classification]
    bufferOffset = _serializer.int32(obj.bsm_classification, buffer, bufferOffset);
    // Serialize message field [tim_dataframe_cnt]
    bufferOffset = _serializer.int32(obj.tim_dataframe_cnt, buffer, bufferOffset);
    // Serialize message field [tim_starttime]
    bufferOffset = _arraySerializer.int32(obj.tim_starttime, buffer, bufferOffset, null);
    // Serialize message field [tim_durationtime]
    bufferOffset = _arraySerializer.int32(obj.tim_durationtime, buffer, bufferOffset, null);
    // Serialize message field [tim_anchor_lat]
    bufferOffset = _arraySerializer.int32(obj.tim_anchor_lat, buffer, bufferOffset, null);
    // Serialize message field [tim_anchor_lon]
    bufferOffset = _arraySerializer.int32(obj.tim_anchor_lon, buffer, bufferOffset, null);
    // Serialize message field [tim_lanewidth]
    bufferOffset = _arraySerializer.int32(obj.tim_lanewidth, buffer, bufferOffset, null);
    // Serialize message field [tim_direction]
    bufferOffset = _arraySerializer.int32(obj.tim_direction, buffer, bufferOffset, null);
    // Serialize message field [tim_nodelist_xy_cnt]
    bufferOffset = _arraySerializer.int32(obj.tim_nodelist_xy_cnt, buffer, bufferOffset, null);
    // Serialize message field [tim_nodelist_xy_latlon]
    bufferOffset = _arraySerializer.int32(obj.tim_nodelist_xy_latlon, buffer, bufferOffset, null);
    // Serialize message field [tim_content]
    bufferOffset = _arraySerializer.int32(obj.tim_content, buffer, bufferOffset, null);
    // Serialize message field [tim_speedlimit]
    bufferOffset = _arraySerializer.int32(obj.tim_speedlimit, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type v2x_info
    let len;
    let data = new v2x_info(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [msg_type]
    data.msg_type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [map_id_region]
    data.map_id_region = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [map_refpoint_lat]
    data.map_refpoint_lat = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [map_refpoint_lon]
    data.map_refpoint_lon = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [map_speed_limit]
    data.map_speed_limit = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [map_g_cnt]
    data.map_g_cnt = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [map_g_id]
    data.map_g_id = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [map_g_nodelist_cnt]
    data.map_g_nodelist_cnt = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [map_g_nodelist_xy]
    data.map_g_nodelist_xy = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [map_g_connectsto_cnt]
    data.map_g_connectsto_cnt = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [map_g_connectsto_lane]
    data.map_g_connectsto_lane = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [spat_id_region]
    data.spat_id_region = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [spat_movement_cnt]
    data.spat_movement_cnt = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [spat_movement_name]
    data.spat_movement_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [spat_eventstate]
    data.spat_eventstate = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [spat_minendtime]
    data.spat_minendtime = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [bsm_id]
    data.bsm_id = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [bsm_lat]
    data.bsm_lat = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bsm_lon]
    data.bsm_lon = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bsm_angle]
    data.bsm_angle = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bsm_size_width]
    data.bsm_size_width = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bsm_size_length]
    data.bsm_size_length = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bsm_classification]
    data.bsm_classification = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tim_dataframe_cnt]
    data.tim_dataframe_cnt = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tim_starttime]
    data.tim_starttime = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_durationtime]
    data.tim_durationtime = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_anchor_lat]
    data.tim_anchor_lat = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_anchor_lon]
    data.tim_anchor_lon = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_lanewidth]
    data.tim_lanewidth = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_direction]
    data.tim_direction = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_nodelist_xy_cnt]
    data.tim_nodelist_xy_cnt = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_nodelist_xy_latlon]
    data.tim_nodelist_xy_latlon = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_content]
    data.tim_content = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [tim_speedlimit]
    data.tim_speedlimit = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.map_g_id.length;
    length += 4 * object.map_g_nodelist_cnt.length;
    length += 4 * object.map_g_nodelist_xy.length;
    length += 4 * object.map_g_connectsto_cnt.length;
    length += 4 * object.map_g_connectsto_lane.length;
    object.spat_movement_name.forEach((val) => {
      length += 4 + val.length;
    });
    length += 4 * object.spat_eventstate.length;
    length += 4 * object.spat_minendtime.length;
    length += 4 * object.bsm_id.length;
    length += 4 * object.tim_starttime.length;
    length += 4 * object.tim_durationtime.length;
    length += 4 * object.tim_anchor_lat.length;
    length += 4 * object.tim_anchor_lon.length;
    length += 4 * object.tim_lanewidth.length;
    length += 4 * object.tim_direction.length;
    length += 4 * object.tim_nodelist_xy_cnt.length;
    length += 4 * object.tim_nodelist_xy_latlon.length;
    length += 4 * object.tim_content.length;
    length += 4 * object.tim_speedlimit.length;
    return length + 136;
  }

  static datatype() {
    // Returns string type for a message object
    return 'v2x_msgs/v2x_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d901339c101aa3472bf57a8feba3d77';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new v2x_info(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.msg_type !== undefined) {
      resolved.msg_type = msg.msg_type;
    }
    else {
      resolved.msg_type = 0
    }

    if (msg.map_id_region !== undefined) {
      resolved.map_id_region = msg.map_id_region;
    }
    else {
      resolved.map_id_region = 0
    }

    if (msg.map_refpoint_lat !== undefined) {
      resolved.map_refpoint_lat = msg.map_refpoint_lat;
    }
    else {
      resolved.map_refpoint_lat = 0
    }

    if (msg.map_refpoint_lon !== undefined) {
      resolved.map_refpoint_lon = msg.map_refpoint_lon;
    }
    else {
      resolved.map_refpoint_lon = 0
    }

    if (msg.map_speed_limit !== undefined) {
      resolved.map_speed_limit = msg.map_speed_limit;
    }
    else {
      resolved.map_speed_limit = 0
    }

    if (msg.map_g_cnt !== undefined) {
      resolved.map_g_cnt = msg.map_g_cnt;
    }
    else {
      resolved.map_g_cnt = 0
    }

    if (msg.map_g_id !== undefined) {
      resolved.map_g_id = msg.map_g_id;
    }
    else {
      resolved.map_g_id = []
    }

    if (msg.map_g_nodelist_cnt !== undefined) {
      resolved.map_g_nodelist_cnt = msg.map_g_nodelist_cnt;
    }
    else {
      resolved.map_g_nodelist_cnt = []
    }

    if (msg.map_g_nodelist_xy !== undefined) {
      resolved.map_g_nodelist_xy = msg.map_g_nodelist_xy;
    }
    else {
      resolved.map_g_nodelist_xy = []
    }

    if (msg.map_g_connectsto_cnt !== undefined) {
      resolved.map_g_connectsto_cnt = msg.map_g_connectsto_cnt;
    }
    else {
      resolved.map_g_connectsto_cnt = []
    }

    if (msg.map_g_connectsto_lane !== undefined) {
      resolved.map_g_connectsto_lane = msg.map_g_connectsto_lane;
    }
    else {
      resolved.map_g_connectsto_lane = []
    }

    if (msg.spat_id_region !== undefined) {
      resolved.spat_id_region = msg.spat_id_region;
    }
    else {
      resolved.spat_id_region = 0
    }

    if (msg.spat_movement_cnt !== undefined) {
      resolved.spat_movement_cnt = msg.spat_movement_cnt;
    }
    else {
      resolved.spat_movement_cnt = 0
    }

    if (msg.spat_movement_name !== undefined) {
      resolved.spat_movement_name = msg.spat_movement_name;
    }
    else {
      resolved.spat_movement_name = []
    }

    if (msg.spat_eventstate !== undefined) {
      resolved.spat_eventstate = msg.spat_eventstate;
    }
    else {
      resolved.spat_eventstate = []
    }

    if (msg.spat_minendtime !== undefined) {
      resolved.spat_minendtime = msg.spat_minendtime;
    }
    else {
      resolved.spat_minendtime = []
    }

    if (msg.bsm_id !== undefined) {
      resolved.bsm_id = msg.bsm_id;
    }
    else {
      resolved.bsm_id = []
    }

    if (msg.bsm_lat !== undefined) {
      resolved.bsm_lat = msg.bsm_lat;
    }
    else {
      resolved.bsm_lat = 0
    }

    if (msg.bsm_lon !== undefined) {
      resolved.bsm_lon = msg.bsm_lon;
    }
    else {
      resolved.bsm_lon = 0
    }

    if (msg.bsm_angle !== undefined) {
      resolved.bsm_angle = msg.bsm_angle;
    }
    else {
      resolved.bsm_angle = 0
    }

    if (msg.bsm_size_width !== undefined) {
      resolved.bsm_size_width = msg.bsm_size_width;
    }
    else {
      resolved.bsm_size_width = 0
    }

    if (msg.bsm_size_length !== undefined) {
      resolved.bsm_size_length = msg.bsm_size_length;
    }
    else {
      resolved.bsm_size_length = 0
    }

    if (msg.bsm_classification !== undefined) {
      resolved.bsm_classification = msg.bsm_classification;
    }
    else {
      resolved.bsm_classification = 0
    }

    if (msg.tim_dataframe_cnt !== undefined) {
      resolved.tim_dataframe_cnt = msg.tim_dataframe_cnt;
    }
    else {
      resolved.tim_dataframe_cnt = 0
    }

    if (msg.tim_starttime !== undefined) {
      resolved.tim_starttime = msg.tim_starttime;
    }
    else {
      resolved.tim_starttime = []
    }

    if (msg.tim_durationtime !== undefined) {
      resolved.tim_durationtime = msg.tim_durationtime;
    }
    else {
      resolved.tim_durationtime = []
    }

    if (msg.tim_anchor_lat !== undefined) {
      resolved.tim_anchor_lat = msg.tim_anchor_lat;
    }
    else {
      resolved.tim_anchor_lat = []
    }

    if (msg.tim_anchor_lon !== undefined) {
      resolved.tim_anchor_lon = msg.tim_anchor_lon;
    }
    else {
      resolved.tim_anchor_lon = []
    }

    if (msg.tim_lanewidth !== undefined) {
      resolved.tim_lanewidth = msg.tim_lanewidth;
    }
    else {
      resolved.tim_lanewidth = []
    }

    if (msg.tim_direction !== undefined) {
      resolved.tim_direction = msg.tim_direction;
    }
    else {
      resolved.tim_direction = []
    }

    if (msg.tim_nodelist_xy_cnt !== undefined) {
      resolved.tim_nodelist_xy_cnt = msg.tim_nodelist_xy_cnt;
    }
    else {
      resolved.tim_nodelist_xy_cnt = []
    }

    if (msg.tim_nodelist_xy_latlon !== undefined) {
      resolved.tim_nodelist_xy_latlon = msg.tim_nodelist_xy_latlon;
    }
    else {
      resolved.tim_nodelist_xy_latlon = []
    }

    if (msg.tim_content !== undefined) {
      resolved.tim_content = msg.tim_content;
    }
    else {
      resolved.tim_content = []
    }

    if (msg.tim_speedlimit !== undefined) {
      resolved.tim_speedlimit = msg.tim_speedlimit;
    }
    else {
      resolved.tim_speedlimit = []
    }

    return resolved;
    }
};

// Constants for message
v2x_info.Constants = {
  BSM_MSG_TYPE: 1,
  SPAT_MSG_TYPE: 2,
  TIM_MSG_TYPE: 3,
  MAP_MSG_TYPE: 4,
}

module.exports = v2x_info;
