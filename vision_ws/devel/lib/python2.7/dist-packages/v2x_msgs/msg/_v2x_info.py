# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from v2x_msgs/v2x_info.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class v2x_info(genpy.Message):
  _md5sum = "0d901339c101aa3472bf57a8feba3d77"
  _type = "v2x_msgs/v2x_info"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

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
"""
  # Pseudo-constants
  BSM_MSG_TYPE = 1
  SPAT_MSG_TYPE = 2
  TIM_MSG_TYPE = 3
  MAP_MSG_TYPE = 4

  __slots__ = ['header','msg_type','map_id_region','map_refpoint_lat','map_refpoint_lon','map_speed_limit','map_g_cnt','map_g_id','map_g_nodelist_cnt','map_g_nodelist_xy','map_g_connectsto_cnt','map_g_connectsto_lane','spat_id_region','spat_movement_cnt','spat_movement_name','spat_eventstate','spat_minendtime','bsm_id','bsm_lat','bsm_lon','bsm_angle','bsm_size_width','bsm_size_length','bsm_classification','tim_dataframe_cnt','tim_starttime','tim_durationtime','tim_anchor_lat','tim_anchor_lon','tim_lanewidth','tim_direction','tim_nodelist_xy_cnt','tim_nodelist_xy_latlon','tim_content','tim_speedlimit']
  _slot_types = ['std_msgs/Header','int32','int32','int32','int32','int32','int32','int32[]','int32[]','int32[]','int32[]','int32[]','int32','int32','string[]','int32[]','int32[]','int32[]','int32','int32','int32','int32','int32','int32','int32','int32[]','int32[]','int32[]','int32[]','int32[]','int32[]','int32[]','int32[]','int32[]','int32[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,msg_type,map_id_region,map_refpoint_lat,map_refpoint_lon,map_speed_limit,map_g_cnt,map_g_id,map_g_nodelist_cnt,map_g_nodelist_xy,map_g_connectsto_cnt,map_g_connectsto_lane,spat_id_region,spat_movement_cnt,spat_movement_name,spat_eventstate,spat_minendtime,bsm_id,bsm_lat,bsm_lon,bsm_angle,bsm_size_width,bsm_size_length,bsm_classification,tim_dataframe_cnt,tim_starttime,tim_durationtime,tim_anchor_lat,tim_anchor_lon,tim_lanewidth,tim_direction,tim_nodelist_xy_cnt,tim_nodelist_xy_latlon,tim_content,tim_speedlimit

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(v2x_info, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.msg_type is None:
        self.msg_type = 0
      if self.map_id_region is None:
        self.map_id_region = 0
      if self.map_refpoint_lat is None:
        self.map_refpoint_lat = 0
      if self.map_refpoint_lon is None:
        self.map_refpoint_lon = 0
      if self.map_speed_limit is None:
        self.map_speed_limit = 0
      if self.map_g_cnt is None:
        self.map_g_cnt = 0
      if self.map_g_id is None:
        self.map_g_id = []
      if self.map_g_nodelist_cnt is None:
        self.map_g_nodelist_cnt = []
      if self.map_g_nodelist_xy is None:
        self.map_g_nodelist_xy = []
      if self.map_g_connectsto_cnt is None:
        self.map_g_connectsto_cnt = []
      if self.map_g_connectsto_lane is None:
        self.map_g_connectsto_lane = []
      if self.spat_id_region is None:
        self.spat_id_region = 0
      if self.spat_movement_cnt is None:
        self.spat_movement_cnt = 0
      if self.spat_movement_name is None:
        self.spat_movement_name = []
      if self.spat_eventstate is None:
        self.spat_eventstate = []
      if self.spat_minendtime is None:
        self.spat_minendtime = []
      if self.bsm_id is None:
        self.bsm_id = []
      if self.bsm_lat is None:
        self.bsm_lat = 0
      if self.bsm_lon is None:
        self.bsm_lon = 0
      if self.bsm_angle is None:
        self.bsm_angle = 0
      if self.bsm_size_width is None:
        self.bsm_size_width = 0
      if self.bsm_size_length is None:
        self.bsm_size_length = 0
      if self.bsm_classification is None:
        self.bsm_classification = 0
      if self.tim_dataframe_cnt is None:
        self.tim_dataframe_cnt = 0
      if self.tim_starttime is None:
        self.tim_starttime = []
      if self.tim_durationtime is None:
        self.tim_durationtime = []
      if self.tim_anchor_lat is None:
        self.tim_anchor_lat = []
      if self.tim_anchor_lon is None:
        self.tim_anchor_lon = []
      if self.tim_lanewidth is None:
        self.tim_lanewidth = []
      if self.tim_direction is None:
        self.tim_direction = []
      if self.tim_nodelist_xy_cnt is None:
        self.tim_nodelist_xy_cnt = []
      if self.tim_nodelist_xy_latlon is None:
        self.tim_nodelist_xy_latlon = []
      if self.tim_content is None:
        self.tim_content = []
      if self.tim_speedlimit is None:
        self.tim_speedlimit = []
    else:
      self.header = std_msgs.msg.Header()
      self.msg_type = 0
      self.map_id_region = 0
      self.map_refpoint_lat = 0
      self.map_refpoint_lon = 0
      self.map_speed_limit = 0
      self.map_g_cnt = 0
      self.map_g_id = []
      self.map_g_nodelist_cnt = []
      self.map_g_nodelist_xy = []
      self.map_g_connectsto_cnt = []
      self.map_g_connectsto_lane = []
      self.spat_id_region = 0
      self.spat_movement_cnt = 0
      self.spat_movement_name = []
      self.spat_eventstate = []
      self.spat_minendtime = []
      self.bsm_id = []
      self.bsm_lat = 0
      self.bsm_lon = 0
      self.bsm_angle = 0
      self.bsm_size_width = 0
      self.bsm_size_length = 0
      self.bsm_classification = 0
      self.tim_dataframe_cnt = 0
      self.tim_starttime = []
      self.tim_durationtime = []
      self.tim_anchor_lat = []
      self.tim_anchor_lon = []
      self.tim_lanewidth = []
      self.tim_direction = []
      self.tim_nodelist_xy_cnt = []
      self.tim_nodelist_xy_latlon = []
      self.tim_content = []
      self.tim_speedlimit = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_6i().pack(_x.msg_type, _x.map_id_region, _x.map_refpoint_lat, _x.map_refpoint_lon, _x.map_speed_limit, _x.map_g_cnt))
      length = len(self.map_g_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.map_g_id))
      length = len(self.map_g_nodelist_cnt)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.map_g_nodelist_cnt))
      length = len(self.map_g_nodelist_xy)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.map_g_nodelist_xy))
      length = len(self.map_g_connectsto_cnt)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.map_g_connectsto_cnt))
      length = len(self.map_g_connectsto_lane)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.map_g_connectsto_lane))
      _x = self
      buff.write(_get_struct_2i().pack(_x.spat_id_region, _x.spat_movement_cnt))
      length = len(self.spat_movement_name)
      buff.write(_struct_I.pack(length))
      for val1 in self.spat_movement_name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.spat_eventstate)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.spat_eventstate))
      length = len(self.spat_minendtime)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.spat_minendtime))
      length = len(self.bsm_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.bsm_id))
      _x = self
      buff.write(_get_struct_7i().pack(_x.bsm_lat, _x.bsm_lon, _x.bsm_angle, _x.bsm_size_width, _x.bsm_size_length, _x.bsm_classification, _x.tim_dataframe_cnt))
      length = len(self.tim_starttime)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_starttime))
      length = len(self.tim_durationtime)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_durationtime))
      length = len(self.tim_anchor_lat)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_anchor_lat))
      length = len(self.tim_anchor_lon)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_anchor_lon))
      length = len(self.tim_lanewidth)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_lanewidth))
      length = len(self.tim_direction)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_direction))
      length = len(self.tim_nodelist_xy_cnt)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_nodelist_xy_cnt))
      length = len(self.tim_nodelist_xy_latlon)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_nodelist_xy_latlon))
      length = len(self.tim_content)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_content))
      length = len(self.tim_speedlimit)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.tim_speedlimit))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.msg_type, _x.map_id_region, _x.map_refpoint_lat, _x.map_refpoint_lon, _x.map_speed_limit, _x.map_g_cnt,) = _get_struct_6i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_id = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_nodelist_cnt = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_nodelist_xy = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_connectsto_cnt = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_connectsto_lane = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 8
      (_x.spat_id_region, _x.spat_movement_cnt,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.spat_movement_name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.spat_movement_name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.spat_eventstate = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.spat_minendtime = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.bsm_id = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 28
      (_x.bsm_lat, _x.bsm_lon, _x.bsm_angle, _x.bsm_size_width, _x.bsm_size_length, _x.bsm_classification, _x.tim_dataframe_cnt,) = _get_struct_7i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_starttime = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_durationtime = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_anchor_lat = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_anchor_lon = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_lanewidth = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_direction = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_nodelist_xy_cnt = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_nodelist_xy_latlon = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_content = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_speedlimit = struct.unpack(pattern, str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_6i().pack(_x.msg_type, _x.map_id_region, _x.map_refpoint_lat, _x.map_refpoint_lon, _x.map_speed_limit, _x.map_g_cnt))
      length = len(self.map_g_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.map_g_id.tostring())
      length = len(self.map_g_nodelist_cnt)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.map_g_nodelist_cnt.tostring())
      length = len(self.map_g_nodelist_xy)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.map_g_nodelist_xy.tostring())
      length = len(self.map_g_connectsto_cnt)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.map_g_connectsto_cnt.tostring())
      length = len(self.map_g_connectsto_lane)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.map_g_connectsto_lane.tostring())
      _x = self
      buff.write(_get_struct_2i().pack(_x.spat_id_region, _x.spat_movement_cnt))
      length = len(self.spat_movement_name)
      buff.write(_struct_I.pack(length))
      for val1 in self.spat_movement_name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.spat_eventstate)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.spat_eventstate.tostring())
      length = len(self.spat_minendtime)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.spat_minendtime.tostring())
      length = len(self.bsm_id)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.bsm_id.tostring())
      _x = self
      buff.write(_get_struct_7i().pack(_x.bsm_lat, _x.bsm_lon, _x.bsm_angle, _x.bsm_size_width, _x.bsm_size_length, _x.bsm_classification, _x.tim_dataframe_cnt))
      length = len(self.tim_starttime)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_starttime.tostring())
      length = len(self.tim_durationtime)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_durationtime.tostring())
      length = len(self.tim_anchor_lat)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_anchor_lat.tostring())
      length = len(self.tim_anchor_lon)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_anchor_lon.tostring())
      length = len(self.tim_lanewidth)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_lanewidth.tostring())
      length = len(self.tim_direction)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_direction.tostring())
      length = len(self.tim_nodelist_xy_cnt)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_nodelist_xy_cnt.tostring())
      length = len(self.tim_nodelist_xy_latlon)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_nodelist_xy_latlon.tostring())
      length = len(self.tim_content)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_content.tostring())
      length = len(self.tim_speedlimit)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.tim_speedlimit.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.msg_type, _x.map_id_region, _x.map_refpoint_lat, _x.map_refpoint_lon, _x.map_speed_limit, _x.map_g_cnt,) = _get_struct_6i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_id = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_nodelist_cnt = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_nodelist_xy = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_connectsto_cnt = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.map_g_connectsto_lane = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 8
      (_x.spat_id_region, _x.spat_movement_cnt,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.spat_movement_name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.spat_movement_name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.spat_eventstate = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.spat_minendtime = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.bsm_id = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 28
      (_x.bsm_lat, _x.bsm_lon, _x.bsm_angle, _x.bsm_size_width, _x.bsm_size_length, _x.bsm_classification, _x.tim_dataframe_cnt,) = _get_struct_7i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_starttime = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_durationtime = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_anchor_lat = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_anchor_lon = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_lanewidth = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_direction = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_nodelist_xy_cnt = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_nodelist_xy_latlon = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_content = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.tim_speedlimit = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6i = None
def _get_struct_6i():
    global _struct_6i
    if _struct_6i is None:
        _struct_6i = struct.Struct("<6i")
    return _struct_6i
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i
_struct_7i = None
def _get_struct_7i():
    global _struct_7i
    if _struct_7i is None:
        _struct_7i = struct.Struct("<7i")
    return _struct_7i
