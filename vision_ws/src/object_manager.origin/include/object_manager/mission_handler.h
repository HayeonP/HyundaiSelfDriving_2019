#pragma once

#include <ros/ros.h>

//mission processor includes
#include <object_manager/mission_processor.h>
#include <object_manager/mission_pedestrian.h>
#include <object_manager/mission_traffic_sign.h>
#include <object_manager/mission_none.h>
#include <object_manager/mission_emergency_vehicle.h>
#include <object_manager/mission_accident_vehicle.h>

//sub, pub includes
#include <autoware_msgs/DetectedObjectArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/TwistStamped.h>

//c++ libraries
#include <vector>
#include <mutex>

//AVC specific header
#include <object_manager/AVCInterface.h>

enum {
    MISSION_NONE,
    MISSION_PEDESTRIAN,
    MISSION_TRAFFIC_SIGN,
    MISSION_ACCIDENT_VEHICLE,
    MISSION_EMERGENCY_VEHICLE,
    N_MISSION
};

class MissionHandler{
public:
    MissionHandler() : 
        mission_list(N_MISSION)
    {   
        mission_list[MISSION_NONE]            =
            std::make_shared<NoneProcessor>();
        mission_list[MISSION_PEDESTRIAN]      = 
            std::make_shared<PedestrianProcessor>();
        mission_list[MISSION_TRAFFIC_SIGN]    = 
            std::make_shared<TrafficSignProcessor>();
        mission_list[MISSION_ACCIDENT_VEHICLE]    = 
            std::make_shared<AccidentVehilceProcessor>();
        mission_list[MISSION_EMERGENCY_VEHICLE]    = 
            std::make_shared<EmergencyVehicleProcessor>();        
        //mission_list[MISSION_NONE]            =
        //     std::make_shared<AccidentVehilceProcessor>();
        // mission_list[MISSION_PEDESTRIAN]      = 
        //     std::make_shared<AccidentVehilceProcessor>();
        // mission_list[MISSION_TRAFFIC_SIGN]    = 
        //     std::make_shared<AccidentVehilceProcessor>();
        // mission_list[MISSION_ACCIDENT_VEHICLE]    = 
        //     std::make_shared<AccidentVehilceProcessor>();
        // mission_list[MISSION_EMERGENCY_VEHICLE]    = 
        //     std::make_shared<AccidentVehilceProcessor>();        

    }

    void run(AVCInterface& avc_interface,
        int mission_identifier, 
        const object_manager_msgs::combined msg){

        //set current mission processor
        MissionProcessorPtr cur_mission_ptr = mission_list[mission_identifier];
        
        //process mission specific workings and set twist filter
        cur_mission_ptr->processMission(avc_interface, msg);
    }

    void set_param(object_manager::ObjectManagerConfig &config){
        for(auto mission_ptr : mission_list)
            mission_ptr->set_param(config);
    }

private:
    ros::NodeHandle nh, private_nh;
    std::vector<MissionProcessorPtr> mission_list;
};