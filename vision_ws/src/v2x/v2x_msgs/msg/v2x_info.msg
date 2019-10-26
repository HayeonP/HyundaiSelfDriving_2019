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