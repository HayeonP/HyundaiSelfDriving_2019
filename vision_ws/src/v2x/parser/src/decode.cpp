#include <v2x_udpbroadcast2/decode.h>
#include <type_traits> //std::is_same
#include <string>
#define J2735_UPER_MAP  18
#define J2735_UPER_SPAT 19
#define J2735_UPER_BSM  20
#define J2735_UPER_RSA  27
#define J2735_UPER_RTCM 28
#define J2735_UPER_TIM  31

/*
	comment by TW:
		when asn library function parses OPTIONAL member, 
		I figured out that if it is consist of pointer and
		the pointer has value only when the data exists.
*/

typedef struct AVC_HDR {
	uint32_t sec;
	uint32_t usec;
	uint32_t type;
	uint32_t len;
} __attribute__((packed)) AVC_HDR_t;

typedef struct AVC_TAIL {
	uint16_t crc16;
} __attribute__((packed)) AVC_TAIL_t;

//black printing members
static int space = 0;
#define SPACE_UP (space += 2)
#define SPACE_DOWN (space -= 2)
#define SPACE_ASSERT (assert(space >= 0))
#define SPACE_PRINT {for(int i = 0 ; i < space; ++i)	fprintf(stdout, " ");}
constexpr static int BIT_PER_BYTE = 8;

/* K-city debug function */
// Because it is too time consuming to parse and my team won't use all of v2x data,
// I only parse data that K-city sends.
// To avoid missing, I'll cover all of j2735 format
// and I'll leave debug message that I didn't parsed.
#define K_CITY_MISSING_STR "!@#$%^K-city!^%$#@!\n"
static void asn_print_k_city_debug(int line){
	std::string s = std::to_string(line) + " : " + K_CITY_MISSING_STR;
	fprintf(stdout, s.c_str());
}

/* fundamental printing functions */
static inline void asn_print_integer__(const char *str, long data){
	SPACE_ASSERT;
	SPACE_PRINT;
	fprintf(stdout, str, data);
}
static inline void asn_print_integer_o(const char *str, long* data_o){
	SPACE_ASSERT;
	if(data_o) {
		SPACE_PRINT;
		fprintf(stdout, str, *data_o);
	}
}
static inline void asn_print_istring_o(const char *str, IA5String_t* data){
	SPACE_ASSERT;
	if(data){//assumes that it is ASCII encoding.
		SPACE_PRINT;
		fprintf(stdout, str);		
		for(unsigned i = 0 ; i < data->size; ++i) {
			fprintf(stdout, "%c", data->buf[i]);
		}
		fprintf(stdout, "\n");
	}
}
static inline void asn_print_istring__(const char *str, const IA5String_t& data){
	SPACE_ASSERT;
	SPACE_PRINT;
	fprintf(stdout, str);		
	for(unsigned i = 0 ; i < data.size; ++i) {
		fprintf(stdout, "%c", data.buf[i]);
	}
	fprintf(stdout, "\n");
}
static inline void asn_print_bit_ary__(const char *str,const BIT_STRING_s& data){
	SPACE_ASSERT;
	SPACE_PRINT;
	fprintf(stdout, str);
	fprintf(stdout, "(length : %ld bits) - ", data.size*BIT_PER_BYTE - data.bits_unused);//1B = 8 bit
	for(size_t i = 0 ; i < data.size; ++i)
		fprintf(stdout, "%02X", data.buf[i]);
	fprintf(stdout, "\n");
}
static inline void asn_print_bit_ary_o(const char *str,BIT_STRING_s* data_p){
	SPACE_ASSERT;
	if(data_p){
		SPACE_PRINT;
		fprintf(stdout, str);
		fprintf(stdout, "(length : %ld bits) - ", data_p->size*BIT_PER_BYTE - data_p->bits_unused);
		for(size_t i = 0 ; i < data_p->size; ++i)
			fprintf(stdout, "%02X", data_p->buf[i]);
		fprintf(stdout, "\n");
	}
}
static inline void asn_print_ostring__(const char *str,const OCTET_STRING_t& data){
	SPACE_ASSERT;
	SPACE_PRINT;
	fprintf(stdout, str);
	for(size_t i = 0 ; i < data.size; ++i)
		fprintf(stdout, "%02x", data.buf[i]);
	fprintf(stdout, "\n");
}
static inline void asn_print_ostring_o(const char *str,const OCTET_STRING_t* data){
	SPACE_ASSERT;
	SPACE_PRINT;
	fprintf(stdout, str);
	for(size_t i = 0 ; i < data->size; ++i)
		fprintf(stdout, "%02x", data->buf[i]);
	fprintf(stdout, "\n");
}
static inline void asn_print_normalstr(const char *str){
	SPACE_ASSERT;
	SPACE_PRINT;
	fprintf(stdout, str);
}

template <typename ListType_> //because the ListType_ is anonymous type, 
//to make function I must use template.
static inline void asn_print_sz_arry__(const char *str, ListType_ list){
	static_assert(!std::is_same<decltype(list.array), long*>::value, "list array type is not long*!\n");

	SPACE_ASSERT;
	SPACE_PRINT;
	fprintf(stdout, str);
	for(int i = 0 ; i < list.count; ++i)
		fprintf(stdout, " %ld", list.array[i]);
	fprintf(stdout, "\n");
}

static void gen_mapMsg(MapData_t *msg, v2x_msgs::v2x_info *msg_ptr){
	msg_ptr->msg_type = v2x_msgs::v2x_info::MAP_MSG_TYPE;

	msg_ptr->map_id_region = -1;
	msg_ptr->map_refpoint_lat = 0.0;
	msg_ptr->map_refpoint_lon = 0.0;
	msg_ptr->map_speed_limit = 0;
	msg_ptr->map_g_cnt = 0;
	msg_ptr->map_g_id.resize(0);
	msg_ptr->map_g_nodelist_cnt.resize(0);
	msg_ptr->map_g_nodelist_xy.resize(0);
	msg_ptr->map_g_connectsto_cnt.resize(0);
	msg_ptr->map_g_connectsto_lane.resize(0);

	IntersectionGeometry_t* geo = msg->intersections->list.array[0];
	msg_ptr->map_id_region = geo->id.region;
	msg_ptr->map_refpoint_lat = geo->refPoint.lat;
	msg_ptr->map_refpoint_lon = geo->refPoint.Long;
	if (geo->speedLimits){
		RegulatorySpeedLimit_t *lmt = geo->speedLimits->list.array[0];
		msg_ptr->map_speed_limit = lmt->speed;
	} 
	msg_ptr->map_g_cnt = geo->laneSet.list.count;
	for (int i = 0 ; i < msg_ptr->map_g_cnt; ++i){
		GenericLane_t *lane = geo->laneSet.list.array[i];
		msg_ptr->map_g_id.push_back(lane->laneID);
		msg_ptr->map_g_nodelist_cnt.push_back(lane->nodeList.nodes.list.count);
		for (int i = 0 ; i < msg_ptr->map_g_nodelist_cnt.back(); ++i){
			NodeXY* node_p = lane->nodeList.nodes.list.array[i];
			msg_ptr->map_g_nodelist_xy.push_back(node_p->delta.node_XY6.x);
			msg_ptr->map_g_nodelist_xy.push_back(node_p->delta.node_XY6.y);
		}

		if (lane->connectsTo){
			msg_ptr->map_g_connectsto_cnt.push_back(lane->connectsTo->list.count);
			for (int i = 0 ; i < msg_ptr->map_g_connectsto_cnt.back(); ++i){
				Connection_t *conn_p = lane->connectsTo->list.array[i];
				msg_ptr->map_g_connectsto_lane.push_back(conn_p->connectingLane.lane);
			}
		}
	}
}

static void decode_MapData(MessageFrame_t *frame)
{
	space = 0;
	MapData_t *msg = &frame->value.MapData;
	
	asn_print_integer_o("timeStamp : %ld\n", msg->timeStamp);
	asn_print_integer__("msgIssueRevision : %ld\n", msg->msgIssueRevision);
	asn_print_integer_o("layerType : %ld\n", msg->layerType);
	asn_print_integer_o("layerID : %ld\n", msg->layerID);
	if (msg->intersections){
		asn_print_integer__("intersections : %ld items\n", msg->intersections->list.count);
		SPACE_UP;
		for(int i = 0 ; i < msg->intersections->list.count; ++i){
			asn_print_integer__("item %ld\n", i);
			SPACE_UP;
			IntersectionGeometry_t* geo = msg->intersections->list.array[i];
			asn_print_istring_o("name : ", geo->name);
			asn_print_normalstr("id\n");
			if(geo->id.region){
				SPACE_UP;
				asn_print_integer_o("region : %ld\n", geo->id.region);
				asn_print_integer__("id : %ld\n", geo->id.id);
				SPACE_DOWN;
			}
			asn_print_integer__("revision : %ld\n",geo->revision);
			asn_print_normalstr("refPoint\n");
			SPACE_UP;
			asn_print_integer__("lat : %ld\n", geo->refPoint.lat);
			asn_print_integer__("Long : %ld\n", geo->refPoint.Long);
			asn_print_integer_o("elevation : %ld\n", geo->refPoint.elevation);
			SPACE_DOWN;
			asn_print_integer_o("landWidth : %ld\n", geo->laneWidth);
			if (geo->speedLimits){
				asn_print_integer__("speedLimits : %ld items\n", geo->speedLimits->list.count);
				SPACE_UP;
				for(int i = 0 ; i < geo->speedLimits->list.count; ++i){
					RegulatorySpeedLimit_t *lmt = geo->speedLimits->list.array[0];
					asn_print_integer__("type : %ld\n", lmt->type);
					asn_print_integer__("speed : %ld\n", lmt->speed);
				}
				SPACE_DOWN;
			}
			if (geo->laneSet.list.count){
				asn_print_integer__("laneSet : %ld items\n", geo->laneSet.list.count);
				SPACE_UP;
				for(int i = 0 ; i < geo->laneSet.list.count; ++i){
					asn_print_integer__("Item %ld\n", i);
					SPACE_UP;
					asn_print_normalstr("GenericLane\n");
					SPACE_UP;
					GenericLane_t *lane = geo->laneSet.list.array[i];
					asn_print_integer__("landID : %ld\n", lane->laneID);
					asn_print_istring_o("name : ",lane->name);
					asn_print_integer_o("ingressApproach : %ld\n", lane->ingressApproach);
					asn_print_integer_o("egressApproach : %ld\n", lane->egressApproach);
					asn_print_normalstr("laneAttributes\n");
					SPACE_UP;
					asn_print_bit_ary__("directionalUse", lane->laneAttributes.directionalUse);
					asn_print_bit_ary__("sharedWidth", lane->laneAttributes.sharedWith);
					asn_print_normalstr("laneType\n");
					SPACE_UP;
					switch(lane->laneAttributes.laneType.present){
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_vehicle:
						asn_print_bit_ary__("vehicle - ", lane->laneAttributes.laneType.vehicle);
						break;					
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_crosswalk:
						asn_print_bit_ary__("crosswalk - ", lane->laneAttributes.laneType.crosswalk);
						break;
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_bikeLane:
						asn_print_bit_ary__("bikeLane - ", lane->laneAttributes.laneType.bikeLane);
						break;
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_sidewalk:
						asn_print_bit_ary__("sidewalk - ", lane->laneAttributes.laneType.sidewalk);
						break;
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_median:
						asn_print_bit_ary__("median - ", lane->laneAttributes.laneType.median);
						break;
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_striping:
						asn_print_bit_ary__("striping - ", lane->laneAttributes.laneType.striping);
						break;
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_trackedVehicle:
						asn_print_bit_ary__("trackedVehicle - ", lane->laneAttributes.laneType.trackedVehicle);
						break;
					case LaneTypeAttributes_PR::LaneTypeAttributes_PR_parking:
						asn_print_bit_ary__("parking - ", lane->laneAttributes.laneType.parking);
						break;
					default:
						asn_print_normalstr(__PRETTY_FUNCTION__);
						asn_print_normalstr("impossible situation happened. a programmer must find why this happend");
						exit(-1);
					}
					SPACE_DOWN;
					SPACE_DOWN;
					asn_print_bit_ary__("maneuvers", *lane->maneuvers);
					if (lane->nodeList.present == NodeListXY_PR::NodeListXY_PR_nodes){
						asn_print_normalstr("nodelist : nodes\n");
						SPACE_UP;
						asn_print_integer__("nodes : %ld items\n", lane->nodeList.nodes.list.count);
						SPACE_UP;
						for(int i = 0 ; i  < lane->nodeList.nodes.list.count; ++i){
							asn_print_integer__("Item %ld\n", i);
							SPACE_UP;
							asn_print_normalstr("NodeXY\n");
							SPACE_UP;
							NodeXY* node_p = lane->nodeList.nodes.list.array[i];
							switch(node_p->delta.present){
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY1:
								asn_print_normalstr("delta : XY1\n");
								SPACE_UP;
								asn_print_normalstr("node-XY1\n");
								SPACE_UP;
								asn_print_integer__("x : %ld\n", node_p->delta.node_XY1.x);
								asn_print_integer__("y : %ld\n", node_p->delta.node_XY1.y);
								SPACE_DOWN;
								SPACE_DOWN;
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY2:
								asn_print_normalstr("delta : XY2\n");
								SPACE_UP;
								asn_print_normalstr("node-XY2\n");
								SPACE_UP;
								asn_print_integer__("x : %ld\n", node_p->delta.node_XY2.x);
								asn_print_integer__("y : %ld\n", node_p->delta.node_XY2.y);
								SPACE_DOWN;
								SPACE_DOWN;
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY3:
								asn_print_normalstr("delta : XY3\n");
								SPACE_UP;
								asn_print_normalstr("node-XY3\n");
								SPACE_UP;
								asn_print_integer__("x : %ld\n", node_p->delta.node_XY3.x);
								asn_print_integer__("y : %ld\n", node_p->delta.node_XY3.y);
								SPACE_DOWN;
								SPACE_DOWN;
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY4:
								asn_print_normalstr("delta : XY4\n");
								SPACE_UP;
								asn_print_normalstr("node-XY4\n");
								SPACE_UP;								
								asn_print_integer__("x : %ld\n", node_p->delta.node_XY4.x);
								asn_print_integer__("y : %ld\n", node_p->delta.node_XY4.y);
								SPACE_DOWN;
								SPACE_DOWN;
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY5:
								asn_print_normalstr("delta : XY5\n");
								SPACE_UP;
								asn_print_normalstr("node-XY5\n");
								SPACE_UP;
								asn_print_integer__("x : %ld\n", node_p->delta.node_XY5.x);
								asn_print_integer__("y : %ld\n", node_p->delta.node_XY5.y);
								SPACE_DOWN;
								SPACE_DOWN;
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY6:
								asn_print_normalstr("delta : XY6\n");
								SPACE_UP;
								asn_print_normalstr("node-XY6\n");
								SPACE_UP;
								asn_print_integer__("x : %ld\n", node_p->delta.node_XY6.x);
								asn_print_integer__("y : %ld\n", node_p->delta.node_XY6.y);
								SPACE_DOWN;
								SPACE_DOWN;
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_LatLon:
								asn_print_normalstr("delta : LatLon\n");		
								SPACE_UP;
								asn_print_integer__("lon : %ld",node_p->delta.node_LatLon.lon);
								asn_print_integer__("lat : %ld",node_p->delta.node_LatLon.lat);
								SPACE_DOWN;
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_regional:
								asn_print_normalstr("delta : regional\n");										
								break;
							default:
								asn_print_normalstr(__PRETTY_FUNCTION__);
								asn_print_normalstr("impossible situation happened. a programmer must find why this happend");
								exit(-1);
							}
							SPACE_DOWN;
							SPACE_DOWN;
							if (node_p->attributes){
								asn_print_normalstr("attributes\n");
								SPACE_UP;
								if (node_p->attributes->localNode->list.size){
									asn_print_normalstr("localNode\n");
									SPACE_UP;
									asn_print_integer__("size : %ld\n", node_p->attributes->localNode->list.size);
									SPACE_UP;
									for(int i = 0 ; i < node_p->attributes->localNode->list.size; ++i){
										NodeAttributeXY_t* attr_p = node_p->attributes->localNode->list.array[i];
										asn_print_integer__("type : %ld\n", *attr_p);
									}
									SPACE_DOWN;
									SPACE_DOWN;
								}
								if (node_p->attributes->disabled){
									asn_print_normalstr("disabled\n");
									SPACE_UP;									
									asn_print_integer__("size : %ld", node_p->attributes->disabled->list.size);
									SPACE_UP;									
									for (int i = 0 ; i < node_p->attributes->disabled->list.size;++i){
										SegmentAttributeXY_t *seg_p = node_p->attributes->disabled->list.array[i];
										asn_print_integer__("type : %ld\n", *seg_p);
									}
									SPACE_DOWN;
									SPACE_DOWN;
								}
								if (node_p->attributes->enabled){
									asn_print_normalstr("enabled\n");
									SPACE_UP;
									asn_print_integer__("size : %ld", node_p->attributes->enabled->list.size);
									SPACE_UP;
									for (int i = 0 ; i < node_p->attributes->enabled->list.size; ++i){
										SegmentAttributeXY_t *seg_p = node_p->attributes->enabled->list.array[i];
										asn_print_integer__("type : %ld\n", *seg_p);
									}
									SPACE_DOWN;
									SPACE_DOWN;
								}
								if (node_p->attributes->data){
									asn_print_normalstr("data\n");
									SPACE_UP;
									asn_print_integer__("size : %ld", node_p->attributes->data->list.size);
									SPACE_UP;
									for (int i = 0; i < node_p->attributes->data->list.size; ++i){
										LaneDataAttribute_t *data_p = node_p->attributes->data->list.array[i];
										switch (data_p->present){
										case LaneDataAttribute_PR::LaneDataAttribute_PR_pathEndPointAngle:
											asn_print_integer__("pathEndPointAngle : %ld\n", data_p->pathEndPointAngle);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointCenter:
											asn_print_integer__("laneCrownPointLeft : %ld\n", data_p->laneCrownPointLeft);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointLeft:
											asn_print_integer__("laneCrownPointLeft : %ld\n", data_p->laneCrownPointLeft);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointRight:
											asn_print_integer__("laneCrownPointRight : %ld\n", data_p->laneCrownPointRight);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneAngle:
											asn_print_integer__("laneAngle : %ld\n", data_p->laneAngle);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_speedLimits:
											asn_print_normalstr("speedLimits\n");
											SPACE_UP;
											for(int i = 0 ; i < data_p->speedLimits.list.size; ++i){
												RegulatorySpeedLimit_t *lmt = data_p->speedLimits.list.array[i];
												asn_print_integer__("type : %ld\n", lmt->type);
												asn_print_integer__("speed : %ld\n", lmt->speed);
											}
											SPACE_DOWN;
											break;
										default:
											asn_print_normalstr( __PRETTY_FUNCTION__);
											asn_print_normalstr("impossible situation happened. a programmer must find why this happend");
											exit(-1);
										}//end of switch
										SPACE_DOWN;
									}//end of for()
									SPACE_DOWN;
								}//end of data section
								SPACE_DOWN;
								asn_print_integer_o("dWidth : %ld", node_p->attributes->dWidth);
								asn_print_integer_o("dElevation : %ld", node_p->attributes->dElevation);
							}
						}
						SPACE_DOWN;
						SPACE_DOWN;
					}
					else {
						asn_print_normalstr("nodelist : computed or something wrong\n");						
						asn_print_normalstr("When parsing MAP, nodelist has type for computed. It is out of my expectation\n");
						exit(-1);
					}

					if (lane->connectsTo){
						asn_print_integer__("connectsTo : %ld items\n", lane->connectsTo->list.count);
						SPACE_UP;
						for(int i = 0 ; i < lane->connectsTo->list.count; ++i){
							asn_print_integer__("Item %ld\n", i);
							SPACE_UP;
							asn_print_normalstr("Connection\n");
							SPACE_UP;
							Connection_t *conn_p = lane->connectsTo->list.array[i];
							asn_print_normalstr("connectingLane\n");
							SPACE_UP;
							asn_print_integer__("lane : %ld\n", conn_p->connectingLane.lane);
							asn_print_bit_ary_o("maneuver : ", conn_p->connectingLane.maneuver);
							SPACE_DOWN;

							if (conn_p->remoteIntersection){
								asn_print_normalstr("remoteIntersection\n");
								asn_print_integer_o("region : %ld\n", conn_p->remoteIntersection->region);
								asn_print_integer__("id : %ld\n", conn_p->remoteIntersection->id);
							}
							asn_print_integer_o("signalGroup : %ld\n", conn_p->signalGroup);
							asn_print_integer_o("userClass : %ld\n", conn_p->userClass);
							asn_print_integer_o("connectionID : %ld\n", conn_p->connectionID);
							SPACE_DOWN;
							SPACE_DOWN;
						}
						SPACE_DOWN;
					}//end of connectsTo
					if (lane->overlays){
						asn_print_normalstr("        overlayes : ");
						asn_print_sz_arry__("        overlayes : ", lane->overlays->list);
					}
					SPACE_DOWN;
					SPACE_DOWN;
				}//end of generic lane
				if (geo->preemptPriorityData){
					asn_print_normalstr("      preemptPriorityData : I don't know how to parse preemptPriorityData!\n");
				}
				SPACE_DOWN;
			}
		SPACE_DOWN;
		}
	}
	if (msg->roadSegments){
		asn_print_normalstr("roadSegments\n");
		for(int i = 0 ; i < msg->roadSegments->list.size; ++i){
			RoadSegment* road_p = msg->roadSegments->list.array[i];
			asn_print_istring_o("  name : ", road_p->name);
			asn_print_normalstr("  id\n");
			asn_print_integer_o("    region : %ld\n", road_p->id.region);
			asn_print_integer__("    id : %ld\n", road_p->id.id);
			asn_print_integer__("  msgCount : %ld\n", road_p->revision);
			asn_print_normalstr("  refPoint\n");
			asn_print_integer__("    lat : %ld\n", road_p->refPoint.lat);
			asn_print_integer__("    Long : %ld\n", road_p->refPoint.Long);
			asn_print_integer_o("    elevation : %ld\n", road_p->refPoint.elevation);
			asn_print_integer_o("  laneWidth : %ld\n", road_p->laneWidth);
			asn_print_normalstr("  speedLimits\n");
			for(int i = 0 ; i < road_p->speedLimits->list.size; ++i){
			RegulatorySpeedLimit_t *lmt = road_p->speedLimits->list.array[i];
				asn_print_integer__("    type : %ld\n", lmt->type);
				asn_print_integer__("    speed : %ld\n", lmt->speed);
			}
			asn_print_normalstr("  roadLaneSet\n");
			asn_print_normalstr("    GenericLane\n");
			for(int i = 0 ; i < road_p->roadLaneSet.list.size; ++i){
					GenericLane_t *lane = road_p->roadLaneSet.list.array[i];
					asn_print_integer__("      landID : %ld\n", lane->laneID);
					asn_print_istring_o("      name : ",lane->name);
					asn_print_integer_o("      ingressApproach : %ld\n", lane->ingressApproach);
					asn_print_integer_o("      egressApproach : %ld\n", lane->egressApproach);
					asn_print_normalstr("      laneAttributes\n");
					asn_print_normalstr("        directionalUse\n");
					asn_print_integer__("          size : %ld", lane->laneAttributes.directionalUse.size);
					asn_print_bit_ary__("          content : ", lane->laneAttributes.directionalUse);
					asn_print_normalstr("        sharedWidth\n");
					asn_print_integer__("          size : %ld", lane->laneAttributes.sharedWith.size);
					asn_print_bit_ary__("          content : ", lane->laneAttributes.sharedWith);
					asn_print_integer__("        laneTypeAttritubes : %ld\n", lane->laneAttributes.laneType.present);
					if(lane->maneuvers){
						asn_print_normalstr("        maneuvers\n");						
						asn_print_integer__("          size : %ld", lane->maneuvers->size);
						asn_print_bit_ary_o("          content : ", lane->maneuvers);
					}//each bit has meaning. ex-first bit means "straightAllowed"
					asn_print_normalstr("      nodelist\n");
					if (lane->nodeList.present == NodeListXY_PR::NodeListXY_PR_nodes){
						asn_print_integer__("        size : ",lane->nodeList.nodes.list.size);
						for(int i = 0 ; i  < lane->nodeList.nodes.list.size; ++i){
							NodeXY* node_p = lane->nodeList.nodes.list.array[i];
							switch(node_p->delta.present){
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY1:
								asn_print_normalstr("          delta : XY1\n");
								asn_print_integer__("            x : %ld\n", node_p->delta.node_XY1.x);
								asn_print_integer__("            y : %ld\n", node_p->delta.node_XY1.y);
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY2:
								asn_print_normalstr("          delta : XY2\n");
								asn_print_integer__("            x : %ld\n", node_p->delta.node_XY2.x);
								asn_print_integer__("            y : %ld\n", node_p->delta.node_XY2.y);
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY3:
								asn_print_normalstr("          delta : XY3\n");
								asn_print_integer__("            x : %ld\n", node_p->delta.node_XY3.x);
								asn_print_integer__("            y : %ld\n", node_p->delta.node_XY3.y);
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY4:
								asn_print_normalstr("          delta : XY4\n");
								asn_print_integer__("            x : %ld\n", node_p->delta.node_XY4.x);
								asn_print_integer__("            y : %ld\n", node_p->delta.node_XY4.y);
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY5:
								asn_print_normalstr("          delta : XY5\n");
								asn_print_integer__("            x : %ld\n", node_p->delta.node_XY5.x);
								asn_print_integer__("            y : %ld\n", node_p->delta.node_XY5.y);
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY6:
								asn_print_normalstr("          delta : XY6\n");
								asn_print_integer__("            x : %ld\n", node_p->delta.node_XY6.x);
								asn_print_integer__("            y : %ld\n", node_p->delta.node_XY6.y);
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_LatLon:
								asn_print_normalstr("          delta : LatLon\n");		
								asn_print_integer__("            lon : %ld\n",node_p->delta.node_LatLon.lon);
								asn_print_integer__("            lat : %ld\n",node_p->delta.node_LatLon.lat);
								break;
							case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_regional:
								asn_print_normalstr("          delta : regional\n");										
								break;
							default:
								asn_print_normalstr(__PRETTY_FUNCTION__);
								asn_print_normalstr("impossible situation happened. a programmer must find why this happend");
								exit(-1);
							}
							if (node_p->delta.present == NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_NOTHING){
								asn_print_normalstr("          delta : nothing\n");
								++i; continue;
							}
							if (node_p->attributes){
								asn_print_normalstr("          attributes\n");
								
								if (node_p->attributes->localNode->list.size){
									asn_print_normalstr("            localNode\n");
									asn_print_integer__("              size : %ld\n", node_p->attributes->localNode->list.size);
									for(int i = 0 ; i < node_p->attributes->localNode->list.size; ++i){
										NodeAttributeXY_t* attr_p = node_p->attributes->localNode->list.array[i];
										asn_print_integer__("              type : %ld\n", *attr_p);
									}
								}
								if (node_p->attributes->disabled){
									asn_print_normalstr("            disabled\n");
									asn_print_integer__("              size : %ld", node_p->attributes->disabled->list.size);
									for (int i = 0 ; i < node_p->attributes->disabled->list.size;++i){
										SegmentAttributeXY_t *seg_p = node_p->attributes->disabled->list.array[i];
										asn_print_integer__("              type : %ld\n", *seg_p);
									}
								}
								if (node_p->attributes->enabled){
									asn_print_normalstr("            enabled\n");
									asn_print_integer__("              size : %ld", node_p->attributes->enabled->list.size);
									for (int i = 0 ; i < node_p->attributes->enabled->list.size; ++i){
										SegmentAttributeXY_t *seg_p = node_p->attributes->enabled->list.array[i];
										asn_print_integer__("              type : %ld\n", *seg_p);
									}
								}
								if (node_p->attributes->data){
									asn_print_normalstr("            data\n");
									asn_print_integer__("              size : %ld", node_p->attributes->data->list.size);
									for (int i = 0; i < node_p->attributes->data->list.size; ++i){
										LaneDataAttribute_t *data_p = node_p->attributes->data->list.array[i];
										switch (data_p->present){
										case LaneDataAttribute_PR::LaneDataAttribute_PR_pathEndPointAngle:
											asn_print_integer__("                pathEndPointAngle : %ld\n", data_p->pathEndPointAngle);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointCenter:
											asn_print_integer__("                laneCrownPointLeft : %ld\n", data_p->laneCrownPointLeft);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointLeft:
											asn_print_integer__("                laneCrownPointLeft : %ld\n", data_p->laneCrownPointLeft);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointRight:
											asn_print_integer__("                laneCrownPointRight : %ld\n", data_p->laneCrownPointRight);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_laneAngle:
											asn_print_integer__("                laneAngle : %ld\n", data_p->laneAngle);
											break;
										case LaneDataAttribute_PR::LaneDataAttribute_PR_speedLimits:
											asn_print_normalstr("                speedLimits\n");
											for(int i = 0 ; i < data_p->speedLimits.list.size; ++i){
												RegulatorySpeedLimit_t *lmt = data_p->speedLimits.list.array[i];
												asn_print_integer__("                  type : %ld\n", lmt->type);
												asn_print_integer__("                  speed : %ld\n", lmt->speed);
											}
											break;
										default:
											asn_print_normalstr(__PRETTY_FUNCTION__);
											asn_print_normalstr("impossible situation happened. a programmer must find why this happend");
											exit(-1);
										}//end of switch
									}//end of for()
								}//end of data section
								asn_print_integer_o("            dWidth : %ld", node_p->attributes->dWidth);
								asn_print_integer_o("            dElevation : %ld", node_p->attributes->dElevation);
							}
						}
					}
					else {
						asn_print_normalstr("When parsing MAP, nodelist has type for computed. It is out of my expectation\n");
						exit(-1);
					}
					if (lane->connectsTo){
						asn_print_normalstr("      connectsTo\n");
						for(int i = 0 ; i < lane->connectsTo->list.size; ++i){
							Connection_t *conn_p = lane->connectsTo->list.array[i];
							asn_print_normalstr("        connectingLane\n");
							asn_print_integer__("          land : %ld", conn_p->connectingLane.lane);
							asn_print_bit_ary_o("          maneuver : ", conn_p->connectingLane.maneuver);

							if (conn_p->remoteIntersection){
								asn_print_normalstr("        remoteIntersection\n");
								asn_print_integer_o("          region : %ld\n", conn_p->remoteIntersection->region);
								asn_print_integer__("          id : %ld\n", conn_p->remoteIntersection->id);
							}
							asn_print_integer_o("        signalGroup : %ld\n", conn_p->signalGroup);
							asn_print_integer_o("        userClass : %ld\n", conn_p->userClass);
							asn_print_integer_o("        connectionID : %ld\n", conn_p->connectionID);
						}
					}//end of connectsTo
					if (lane->overlays){
						asn_print_normalstr("        overlayes : ");
						asn_print_sz_arry__("        overlayes : ", lane->overlays->list);
					}
			}//end of generic lane
		}
	}//end of road segments
	if (msg->dataParameters){
		asn_print_normalstr("  dataParamters : I don't know how to parse dataParamters!\n");
	}
	if (msg->restrictionList){
		asn_print_normalstr("  restrictionList : If this msg show in K-city, I should develop more...\n");
	}
}

static void gen_SPaTMsg(SPAT_t *msg, v2x_msgs::v2x_info *msg_ptr){
	msg_ptr->msg_type = v2x_msgs::v2x_info::SPAT_MSG_TYPE;
	
	msg_ptr->spat_id_region = 0;
	msg_ptr->spat_movement_cnt = 0;
	msg_ptr->spat_movement_name.resize(0);
	msg_ptr->spat_eventstate.resize(0);
	msg_ptr->spat_minendtime.resize(0);

	IntersectionState *inter_p = msg->intersections.list.array[0];
	msg_ptr->spat_id_region = *inter_p->id.region;
	msg_ptr->spat_movement_cnt = inter_p->states.list.count;
	for (int i = 0 ; i < msg_ptr->spat_movement_cnt; ++i){
		MovementState* move_p = inter_p->states.list.array[i];
		msg_ptr->spat_movement_name.push_back((char*)move_p->movementName->buf);
		MovementEvent *mvevent_p = move_p->state_time_speed.list.array[0];
		msg_ptr->spat_eventstate.push_back(mvevent_p->eventState);
		if (mvevent_p->timing)
			msg_ptr->spat_minendtime.push_back(mvevent_p->timing->minEndTime);
	}
}
static void decode_SignalPhaseAndTimingMessage(MessageFrame_t *frame)
{
	space = 0;
	
	SPAT_t* msg = &frame->value.SPAT;
	asn_print_integer_o("timeStamp : %ld\n", msg->timeStamp);
	asn_print_istring_o("name : ", msg->name);
	if (msg->intersections.list.count){
		asn_print_integer__("Intersections : %ld items\n", msg->intersections.list.count);
		SPACE_UP;
		for (int i = 0 ; i < msg->intersections.list.count; ++i){
			asn_print_integer__("Item %ld\n", i);
			SPACE_UP;
			asn_print_normalstr("IntersectionState\n");
			SPACE_UP;

			IntersectionState *inter_p = msg->intersections.list.array[i];
			asn_print_istring_o("name : ", inter_p->name);
			asn_print_normalstr("id\n");
			SPACE_UP;
			asn_print_integer_o("region : %ld\n", inter_p->id.region);
			asn_print_integer__("id : %ld\n", inter_p->id.id);
			SPACE_DOWN;//id
			asn_print_integer__("revision : %ld\n", inter_p->revision);
			asn_print_bit_ary__("status : ", inter_p->status);
			asn_print_integer_o("moy : %ld\n", inter_p->moy);
			asn_print_integer_o("timeStamp : %ld\n", inter_p->timeStamp);
			if (inter_p->enabledLanes){
				asn_print_k_city_debug(__LINE__);
			}
			if (inter_p->states.list.count){
				asn_print_integer__("states : %ld items\n", inter_p->states.list.count);
				SPACE_UP;
				for(int i = 0 ; i < inter_p->states.list.count; ++i){
					asn_print_integer__("Item %ld\n", i);
					SPACE_UP;
					asn_print_normalstr("MovementState\n");
					SPACE_UP;
					
					MovementState* move_p = inter_p->states.list.array[i];
					asn_print_istring_o("movementName : ", move_p->movementName);
					asn_print_integer__("signalGroup : %ld\n", move_p->signalGroup);
					if (move_p->state_time_speed.list.count){
						asn_print_integer__("state-time-speed : %ld items\n", move_p->state_time_speed.list.count);
						for (int i = 0 ; i < move_p->state_time_speed.list.count; ++i){
							asn_print_integer__("Item %ld\n", i);
							SPACE_UP;
							asn_print_normalstr("MovementEvent\n");
							SPACE_UP;
							
							MovementEvent *mvevent_p = move_p->state_time_speed.list.array[i];
							asn_print_integer__("eventState(%ld) \n", mvevent_p->eventState);
							SPACE_UP;
							switch (mvevent_p->eventState){
							case e_MovementPhaseState::MovementPhaseState_unavailable:
								asn_print_normalstr("unavailable\n"); break;
							case e_MovementPhaseState::MovementPhaseState_dark:
								asn_print_normalstr("dark\n"); break;
							case e_MovementPhaseState::MovementPhaseState_stop_Then_Proceed:
								asn_print_normalstr("stop_Then_Proceeddark\n"); break;
							case e_MovementPhaseState::MovementPhaseState_stop_And_Remain:
								asn_print_normalstr("stop_And_Remain\n"); break;
							case e_MovementPhaseState::MovementPhaseState_pre_Movement:
								asn_print_normalstr("pre_Movement\n"); break;
							case e_MovementPhaseState::MovementPhaseState_permissive_Movement_Allowed:
								asn_print_normalstr("permissive_Movement_Allowed\n"); break;
							case e_MovementPhaseState::MovementPhaseState_protected_Movement_Allowed:
								asn_print_normalstr("protected_Movement_Allowed\n"); break;
							case e_MovementPhaseState::MovementPhaseState_permissive_clearance:
								asn_print_normalstr("permissive_clearance\n"); break;
							case e_MovementPhaseState::MovementPhaseState_protected_clearance:
								asn_print_normalstr("protected_clearance\n"); break;
							case e_MovementPhaseState::MovementPhaseState_caution_Conflicting_Traffic:
								asn_print_normalstr("caution_Conflicting_Traffic\n"); break;
							default:
								asn_print_normalstr(__PRETTY_FUNCTION__);
								asn_print_normalstr("Impossible situation happened. please check this\n");
								exit(-1);
							}
							SPACE_DOWN;
							if (mvevent_p->timing){
								asn_print_normalstr("timing\n");
								SPACE_UP;
								asn_print_integer_o("startTime : %ld\n", mvevent_p->timing->startTime);
								asn_print_integer__("minEndTime : %ld\n", mvevent_p->timing->minEndTime);
								asn_print_integer_o("maxEntTime : %ld\n", mvevent_p->timing->maxEndTime);
								asn_print_integer_o("likelyTime : %ld\n", mvevent_p->timing->likelyTime);
								asn_print_integer_o("confidence : %ld\n", mvevent_p->timing->confidence);
								asn_print_integer_o("nextTime : %ld\n", mvevent_p->timing->nextTime);
								SPACE_DOWN;//timing
							}
							if (mvevent_p->speeds){
								asn_print_k_city_debug(__LINE__);
							}

							SPACE_DOWN;//movementeEvent
							SPACE_DOWN;//item
						}
					}
					SPACE_DOWN;//Movements
					if (move_p->maneuverAssistList){
						asn_print_k_city_debug(__LINE__);
					}
					SPACE_DOWN;//item
				}
				SPACE_DOWN;//states : %ld
			}
			if (inter_p->maneuverAssistList){
				asn_print_k_city_debug(__LINE__);
			}
			SPACE_DOWN;//intersectionState
			SPACE_DOWN;//item
		}
		SPACE_DOWN;//intersection state
	}
}

static void gen_BSMMsg(BasicSafetyMessage_t *msg, v2x_msgs::v2x_info *msg_ptr){
	msg_ptr->msg_type = v2x_msgs::v2x_info::BSM_MSG_TYPE;
	msg_ptr->bsm_classification = 0;
	
	msg_ptr->bsm_id.resize(0);
	BSMcoreData_t *coreData = &msg->coreData;
	for(int i = 0 ; i < 4 ; ++i)
		msg_ptr->bsm_id.push_back(coreData->id.buf[i]);
	msg_ptr->bsm_lat = coreData->lat;
	msg_ptr->bsm_lon = coreData->Long;
	msg_ptr->bsm_angle = coreData->angle * 0.0125; //gps frame
	msg_ptr->bsm_size_width = coreData->size.width;
	msg_ptr->bsm_size_length = coreData->size.length;

	for (int i = 0; i < msg->partII->list.count; i++) {
		if (msg->partII->list.array[i]->partII_Value.present ==
				PartIIcontent__partII_Value_PR_SupplementalVehicleExtensions) {
			SupplementalVehicleExtensions_t *s_ptr =
		 		&msg->partII->list.array[i]->partII_Value.SupplementalVehicleExtensions;
			if (s_ptr->classification){
				msg_ptr->bsm_classification = *s_ptr->classification;
			}
		}
	}
}
static void decode_BasicSafetyMessage(MessageFrame_t *frame)
{
	BasicSafetyMessage_t *msg = &frame->value.BasicSafetyMessage;
    BSMcoreData_t *coreData = &msg->coreData;

    fprintf(stdout, "coreData\n");

    fprintf(stdout, "  msgCnt       : %ld\n", coreData->msgCnt);
    fprintf(stdout, "  id           : %02X.%02X.%02X.%02X\n",
			coreData->id.buf[0], coreData->id.buf[1],
			coreData->id.buf[2], coreData->id.buf[3]);
    fprintf(stdout, "  secMark      : %ld\n", coreData->secMark);
    fprintf(stdout, "  lat          : %ld\n", coreData->lat);
    fprintf(stdout, "  Long         : %ld\n", coreData->Long);
    fprintf(stdout, "  elev         : %ld\n", coreData->elev);

    fprintf(stdout, "  accuracy\n");
    fprintf(stdout, "    semiMajor  : %ld\n", coreData->accuracy.semiMajor);
    fprintf(stdout, "    semiMinor  : %ld\n", coreData->accuracy.semiMinor);
    fprintf(stdout, "    orientation: %ld\n", coreData->accuracy.orientation);

    fprintf(stdout, "  transmission : %ld\n", coreData->transmission);
    fprintf(stdout, "  speed        : %ld\n", coreData->speed);
    fprintf(stdout, "  heading      : %ld\n", coreData->heading);
    fprintf(stdout, "  angle        : %ld\n", coreData->angle);

    fprintf(stdout, "  accelSet\n");
    fprintf(stdout, "    Long       : %ld\n", coreData->accelSet.Long);
    fprintf(stdout, "    lat        : %ld\n", coreData->accelSet.lat);
    fprintf(stdout, "    vert       : %ld\n", coreData->accelSet.vert);
    fprintf(stdout, "    yaw        : %ld\n", coreData->accelSet.yaw);

    fprintf(stdout, "  brakes\n");
    fprintf(stdout, "    wheelBrakes: %02X\n", coreData->brakes.wheelBrakes.buf[0]);
    fprintf(stdout, "    traction   : %ld\n", coreData->brakes.traction);
    fprintf(stdout, "    albs       : %ld\n", coreData->brakes.albs);
    fprintf(stdout, "    scs        : %ld\n", coreData->brakes.scs);
    fprintf(stdout, "    brakeBoos  : %ld\n", coreData->brakes.brakeBoost);
    fprintf(stdout, "    auxBrakes  : %ld\n", coreData->brakes.auxBrakes);

    fprintf(stdout, "  size\n");
    fprintf(stdout, "    width      : %ld\n", coreData->size.width);
    fprintf(stdout, "    length     : %ld\n", coreData->size.length);

	if (msg->partII == NULL || msg->partII->list.count == 0)
		return;

    fprintf(stdout, "partII\n");

	for (int i = 0; i < msg->partII->list.count; i++) {
		if (msg->partII->list.array[i]->partII_Value.present ==
				PartIIcontent__partII_Value_PR_SupplementalVehicleExtensions) {
			SupplementalVehicleExtensions_t *s_ptr =
		 		&msg->partII->list.array[i]->partII_Value.SupplementalVehicleExtensions;
			if (s_ptr->classification){
				fprintf(stdout, "  SupplementalVehicleExt's classification : %ld\n",
					*s_ptr->classification);
			}
		}
	}
}

static void decode_RTCMcorrections(MessageFrame_t *frame)
{
	// msg = &message->RTCMcorrections;
}

static void gen_TIMMsg(TravelerInformation_t *msg, v2x_msgs::v2x_info *msg_ptr){
	msg_ptr->msg_type = v2x_msgs::v2x_info::TIM_MSG_TYPE;
	
	msg_ptr->tim_dataframe_cnt = -1;
	msg_ptr->tim_starttime.resize(0);
	msg_ptr->tim_durationtime.resize(0);
	msg_ptr->tim_anchor_lat.resize(0);
	msg_ptr->tim_anchor_lon.resize(0);
	msg_ptr->tim_lanewidth.resize(0);
	msg_ptr->tim_direction.resize(0);
	msg_ptr->tim_nodelist_xy_cnt.resize(0);
	msg_ptr->tim_nodelist_xy_latlon.resize(0);
	msg_ptr->tim_content.resize(0);
	msg_ptr->tim_speedlimit.resize(0);

	msg_ptr->tim_dataframe_cnt = msg->dataFrames.list.count;
	for(int i = 0 ; i < msg->dataFrames.list.count; ++i){
		TravelerDataFrame_t *trvd_p = msg->dataFrames.list.array[i];
	
		//process only workzone
		SpeedLimit__Member *spdm_p = trvd_p->content.speedLimit.list.array[0];
		if (spdm_p->item.present == SpeedLimit__Member__item_PR::SpeedLimit__Member__item_PR_text){
			std::string text = "";
			for(int i = 0; i <  spdm_p->item.text.size; ++i)
				text += static_cast<char>(spdm_p->item.text.buf[i]);
			fprintf(stdout, "ss : %s\n", text.c_str());
			std::stringstream ss(text);
			std::string word;

			std::getline(ss, word, ',');
			fprintf(stdout, "word : %s", word.c_str());
			//if (word == "sc") {
			//	fprintf(stdout, "###");
			//	msg_ptr->tim_dataframe_cnt--;
			//	continue;
			//}
			std::getline(ss, word, ',');
			double speedlimit = -1;
			try{
				speedlimit = std::stod(word);			
			}
			catch(std::exception& e){
				fprintf(stdout, "tim stod error : %s", e.what());
				return;
			}
			msg_ptr->tim_speedlimit.push_back(speedlimit);
		}


		msg_ptr->tim_starttime.push_back(trvd_p->startTime);
		msg_ptr->tim_durationtime.push_back(trvd_p->duratonTime);
		GeographicalPath *geo_p = trvd_p->regions.list.array[0];
		msg_ptr->tim_anchor_lat.push_back(geo_p->anchor->lat);
		msg_ptr->tim_anchor_lon.push_back(geo_p->anchor->Long);
		msg_ptr->tim_lanewidth.push_back(*geo_p->laneWidth);
		for(int i = 0 ; i < 2; ++i) //direction member has length 2
			msg_ptr->tim_direction.push_back(geo_p->direction->buf[i]);
		OffsetSystem_t& off_p = geo_p->description->path;
		NodeListXY_t& node_r = off_p.offset.xy;
		if (node_r.nodes.list.count){
			msg_ptr->tim_nodelist_xy_cnt.push_back(node_r.nodes.list.count);
			for(int i = 0 ; i < msg_ptr->tim_nodelist_xy_cnt.back(); ++i){
				NodeXY* node_p = node_r.nodes.list.array[i];
				msg_ptr->tim_nodelist_xy_latlon.push_back(node_p->delta.node_LatLon.lat);
				msg_ptr->tim_nodelist_xy_latlon.push_back(node_p->delta.node_LatLon.lon);
			}
		}
		msg_ptr->tim_content.push_back(trvd_p->content.present);
		
	}
}
static void decode_TravelerInformation(MessageFrame_t *frame)
{
	space = 0;
	TravelerInformation_t *msg = &frame->value.TravelerInformation;
	asn_print_integer__("msgCnt : %ld\n", msg->msgCnt);
	asn_print_integer_o("timeStamp : %ld\n", msg->timeStamp);
	asn_print_ostring_o("packetID : ", msg->packetID);
	if (msg->dataFrames.list.count){
		asn_print_integer__("dataFrames : %ld items\n", msg->dataFrames.list.count);
		SPACE_UP;
		for (int i = 0 ; i < msg->dataFrames.list.count; ++i){
			asn_print_integer__("Item %ld\n", i);
			SPACE_UP;
			asn_print_normalstr("TravelerDataFrame\n");
			SPACE_UP;
			
			TravelerDataFrame_t *trvd_p = msg->dataFrames.list.array[i];
			asn_print_integer__("sspTimRights : %ld\n", trvd_p->sspTimRights);
			asn_print_normalstr("frameType : \n");
			SPACE_UP;
			switch(trvd_p->frameType){
			case e_TravelerInfoType::TravelerInfoType_unknown:
				asn_print_normalstr("unknown\n"); break;
			case e_TravelerInfoType::TravelerInfoType_advisory:
				asn_print_normalstr("advisory\n"); break;
			case e_TravelerInfoType::TravelerInfoType_roadSignage:
				asn_print_normalstr("roadSignage\n"); break;
			case e_TravelerInfoType::TravelerInfoType_commercialSignage:
				asn_print_normalstr("commercialSignage\n"); break;
			default:
				asn_print_normalstr("impossible situation happened. Check this\n");
				exit(-1);
			}
			SPACE_DOWN;//switch
			switch(trvd_p->msgId.present){
			case TravelerDataFrame__msgId_PR::TravelerDataFrame__msgId_PR_furtherInfoID:
				asn_print_ostring__("msgID - furtherInfoID : ", trvd_p->msgId.furtherInfoID); break;
			case TravelerDataFrame__msgId_PR::TravelerDataFrame__msgId_PR_roadSignID:
				asn_print_k_city_debug(__LINE__); break;
			default:
				asn_print_normalstr("impossible situation happened. Check this\n");
				exit(-1);
			}
			asn_print_integer_o("startYear : %ld\n", trvd_p->startYear);
			asn_print_integer__("startTime : %ld\n", trvd_p->startTime);
			asn_print_integer__("durationTime : %ld\n", trvd_p->duratonTime);
			asn_print_integer__("priority : %ld\n", trvd_p->priority);
			asn_print_integer__("sspLocationRight : %ld\n", trvd_p->sspLocationRights);
			if (trvd_p->regions.list.count){
				asn_print_integer__("regions : %ld items\n", trvd_p->regions.list.count);
				SPACE_UP;
				for (int i = 0 ; i < trvd_p->regions.list.count; ++i){
					asn_print_integer__("Item %ld\n", i);
					SPACE_UP;
					asn_print_normalstr("GeographicalPath\n");
					SPACE_UP;
					
					GeographicalPath *geo_p = trvd_p->regions.list.array[i];
					asn_print_istring_o("name : ", geo_p->name);
					if (geo_p->id){
						asn_print_normalstr("id \n");
						SPACE_UP;
						asn_print_integer_o("region : %ld\n", geo_p->id->region);
						asn_print_integer__("id : %ld\n", geo_p->id->id);
						SPACE_DOWN;//id
					}
					if (geo_p->anchor){
						asn_print_normalstr("anchor \n");
						SPACE_UP;
						asn_print_integer__("lat : %ld\n", geo_p->anchor->lat);
						asn_print_integer__("Long : %ld\n", geo_p->anchor->Long);
						asn_print_integer_o("elevation : %ld\n", geo_p->anchor->elevation);
						SPACE_DOWN;//anchor
					}
					asn_print_integer_o("landWidth : %ld\n", geo_p->laneWidth);
					if (geo_p->directionality) asn_print_k_city_debug(__LINE__);
					if (geo_p->closedPath) asn_print_k_city_debug(__LINE__);
					asn_print_bit_ary_o("direction", geo_p->direction);
					if (geo_p->description){
						if (geo_p->description->present == GeographicalPath__description_PR::GeographicalPath__description_PR_path){
							asn_print_normalstr("description : path\n");
							SPACE_UP;
							asn_print_normalstr("path\n");
							SPACE_UP;

							OffsetSystem_t& off_p = geo_p->description->path;
							asn_print_integer_o("scale : %ld\n", off_p.scale);
							if (off_p.offset.present == OffsetSystem__offset_PR::OffsetSystem__offset_PR_xy){
								asn_print_normalstr("offset : xy\n");
								SPACE_UP;
								NodeListXY_t& node_r = off_p.offset.xy;
								if(node_r.present == NodeListXY_PR::NodeListXY_PR_nodes){
									asn_print_normalstr("xy: nodes\n");
									SPACE_UP;
									if (node_r.nodes.list.count){
										asn_print_integer__("nodes : %ld items\n", node_r.nodes.list.count);
										SPACE_UP;
										for(int i = 0 ; i < node_r.nodes.list.count; ++i){
											asn_print_integer__("Item %ld\n", i);
											SPACE_UP;
											asn_print_normalstr("NodeXY\n");
											SPACE_UP;
											NodeXY* node_p = node_r.nodes.list.array[i];
											switch(node_p->delta.present){
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY1:
												asn_print_normalstr("delta : XY1\n");
												SPACE_UP;
												asn_print_normalstr("node-XY1\n");
												SPACE_UP;
												asn_print_integer__("x : %ld\n", node_p->delta.node_XY1.x);
												asn_print_integer__("y : %ld\n", node_p->delta.node_XY1.y);
												SPACE_DOWN;
												SPACE_DOWN;
												break;
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY2:
												asn_print_normalstr("delta : XY2\n");
												SPACE_UP;
												asn_print_normalstr("node-XY2\n");
												SPACE_UP;
												asn_print_integer__("x : %ld\n", node_p->delta.node_XY2.x);
												asn_print_integer__("y : %ld\n", node_p->delta.node_XY2.y);
												SPACE_DOWN;
												SPACE_DOWN;
												break;
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY3:
												asn_print_normalstr("delta : XY3\n");
												SPACE_UP;
												asn_print_normalstr("node-XY3\n");
												SPACE_UP;
												asn_print_integer__("x : %ld\n", node_p->delta.node_XY3.x);
												asn_print_integer__("y : %ld\n", node_p->delta.node_XY3.y);
												SPACE_DOWN;
												SPACE_DOWN;
												break;
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY4:
												asn_print_normalstr("delta : XY4\n");
												SPACE_UP;
												asn_print_normalstr("node-XY4\n");
												SPACE_UP;								
												asn_print_integer__("x : %ld\n", node_p->delta.node_XY4.x);
												asn_print_integer__("y : %ld\n", node_p->delta.node_XY4.y);
												SPACE_DOWN;
												SPACE_DOWN;
												break;
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY5:
												asn_print_normalstr("delta : XY5\n");
												SPACE_UP;
												asn_print_normalstr("node-XY5\n");
												SPACE_UP;
												asn_print_integer__("x : %ld\n", node_p->delta.node_XY5.x);
												asn_print_integer__("y : %ld\n", node_p->delta.node_XY5.y);
												SPACE_DOWN;
												SPACE_DOWN;
												break;
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_XY6:
												asn_print_normalstr("delta : XY6\n");
												SPACE_UP;
												asn_print_normalstr("node-XY6\n");
												SPACE_UP;
												asn_print_integer__("x : %ld\n", node_p->delta.node_XY6.x);
												asn_print_integer__("y : %ld\n", node_p->delta.node_XY6.y);
												SPACE_DOWN;
												SPACE_DOWN;
												break;
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_node_LatLon:
												asn_print_normalstr("delta : LatLon\n");		
												SPACE_UP;
												asn_print_integer__("lon : %ld\n",node_p->delta.node_LatLon.lon);
												asn_print_integer__("lat : %ld\n",node_p->delta.node_LatLon.lat);
												SPACE_DOWN;
												break;
											case NodeOffsetPointXY_PR::NodeOffsetPointXY_PR_regional:
												asn_print_normalstr("delta : regional\n");										
												break;
											default:
												asn_print_normalstr(__PRETTY_FUNCTION__);
												asn_print_normalstr("impossible situation happened. a programmer must find why this happend");
												exit(-1);
											}
										if (node_p->attributes)
											asn_print_k_city_debug(__LINE__);
											SPACE_DOWN;//NodeXY
											SPACE_DOWN;//Item : %ld\n
										}
										SPACE_DOWN;//nodes : %ld
									}
									SPACE_DOWN;// xy : nodes
								}
								else {
									asn_print_k_city_debug(__LINE__);
								}
								SPACE_DOWN;//offset : xy
							}
							else {
								asn_print_k_city_debug(__LINE__);
							}
							SPACE_DOWN;//path
							SPACE_DOWN;//description - path
						}
						else
							asn_print_k_city_debug(__LINE__);
					}
					SPACE_DOWN;//geographical path
					SPACE_DOWN;//item
				}
				SPACE_DOWN;//region
			}
			asn_print_integer__("sspMsgRights1 : %ld\n", trvd_p->sspMsgRights1);
			asn_print_integer__("sspMsgRights2 : %ld\n", trvd_p->sspMsgRights2);
			if (trvd_p->content.present == TravelerDataFrame__content_PR::TravelerDataFrame__content_PR_speedLimit){
				asn_print_normalstr("content : speedLimit\n");
				SPACE_UP;
				asn_print_integer__("speedLimits : %ld items\n", trvd_p->content.speedLimit.list.count);
				SPACE_UP;
				for(int i = 0 ; i < trvd_p->content.speedLimit.list.count; ++i){
					asn_print_integer__("Item %ld\n", i);
					SPACE_UP;
					asn_print_normalstr("SpeedLimit item\n");
					SPACE_UP;
					SpeedLimit__Member *spdm_p = trvd_p->content.speedLimit.list.array[i];
					if (spdm_p->item.present == SpeedLimit__Member__item_PR::SpeedLimit__Member__item_PR_text){
						asn_print_normalstr("item : text\n");
						asn_print_istring__("text : ", spdm_p->item.text);
					}
					else 
						asn_print_k_city_debug(__LINE__);
					SPACE_DOWN;//speedlimit item
					SPACE_DOWN;//Item : %ld
				}
				SPACE_DOWN;
				SPACE_DOWN;//content : speedLimit
			}
			else 
				asn_print_k_city_debug(__LINE__);
			asn_print_istring_o("url : ", trvd_p->url);
			SPACE_DOWN;//travelerDataFrame
			SPACE_DOWN;//Item
		}
		SPACE_DOWN;//dataFrames
	}
}



static int decode_MessageFrame(const uint8_t *buf, int len, MessageFrame_t *frame, v2x_msgs::v2x_info *msg_ptr)
{
    if (frame){ 
        ASN_STRUCT_FREE(asn_DEF_MessageFrame, frame);
        frame = nullptr;
    }
	asn_dec_rval_t rval;

	rval = uper_decode_complete(NULL, &asn_DEF_MessageFrame,
			(void **)&frame, buf, len);
	if (rval.code != RC_OK) {
		//fprintf(stderr, "failed to decode MessageFrame\n");
		//fprintf(stdout, "================================================\n");
		ASN_STRUCT_FREE(asn_DEF_MessageFrame, frame);
		return -1;
	}

	fprintf(stdout, "------------------------------------------------\n");
	fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
	fprintf(stdout,"------------------------------------------------\n");
	//asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
	fprintf(stdout, "================================================\n");
	if (frame->messageId == J2735_UPER_MAP &&
			frame->value.present == MessageFrame__value_PR_MapData) {
		//decode_MapData(frame);
		gen_mapMsg((MapData_t*)&frame->value.MapData, msg_ptr);
	}
	else if (frame->messageId == J2735_UPER_SPAT &&
			frame->value.present == MessageFrame__value_PR_SPAT) {
		//decode_SignalPhaseAndTimingMessage(frame);
		gen_SPaTMsg((SPAT_t*)&frame->value.SPAT, msg_ptr);
	}
	else if (frame->messageId == J2735_UPER_BSM &&
			frame->value.present == MessageFrame__value_PR_BasicSafetyMessage) {
		//decode_BasicSafetyMessage(frame);
		gen_BSMMsg((BasicSafetyMessage_t*)&frame->value.BasicSafetyMessage, msg_ptr);		
	}
	else if (frame->messageId == J2735_UPER_TIM &&
			frame->value.present == MessageFrame__value_PR_TravelerInformation) {
		//decode_TravelerInformation(frame);
		gen_TIMMsg((TravelerInformation_t*)&frame->value.BasicSafetyMessage, msg_ptr);
	}

	return rval.code;
}

int decode_avc2019(const uint8_t *buf, int len, MessageFrame_t *frame, v2x_msgs::v2x_info *msg_ptr)
{
	if (len < sizeof(AVC_HDR_t)) {
		fprintf(stderr, "packet size is less than header\n");
		return -1;
	}

	const AVC_HDR_t *hdr = (AVC_HDR_t *)buf;

	uint32_t sec  = ntohl(hdr->sec);
	uint32_t usec = ntohl(hdr->usec);
	uint32_t type = ntohl(hdr->type);
	uint32_t dlen = ntohl(hdr->len);

	fprintf(stdout, "================================================\n");
	fprintf(stdout, "time: %u.%06u, type: %2u, len: %4u\n", sec, usec, type, dlen);
	switch(type){
	case J2735_UPER_BSM:
		fprintf(stdout, "=============            BSM         ===========\n"); break;
	case J2735_UPER_MAP:
		fprintf(stdout, "=============            MAP         ===========\n"); break;
	case J2735_UPER_SPAT:
		fprintf(stdout, "=============            SPAT        ===========\n"); break;
	case J2735_UPER_TIM:
		fprintf(stdout, "=============            TIM         ===========\n"); break;
	default:
		fprintf(stdout, "=============  msg of no interest    ===========\n"); break;
	}

	if (dlen + sizeof(AVC_HDR_t) + sizeof(AVC_TAIL_t) != len) {
		fprintf(stderr, "packet size is less than data: %lu %u\n",
				dlen + sizeof(AVC_HDR_t), len);
		fprintf(stdout, "================================================\n");
		return -1;
	}

	return decode_MessageFrame(buf + sizeof(AVC_HDR_t), dlen, frame, msg_ptr);
}
