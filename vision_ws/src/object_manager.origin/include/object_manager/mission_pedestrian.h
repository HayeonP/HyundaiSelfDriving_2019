#pragma once
#include <object_manager/mission_processor.h>

class PedestrianProcessor : public MissionProcessor{
public:
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){

        auto& detectedObj = msg.obj_ary;
        bool isPersonMsg = false;

        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if(it->label == "person") {
                isPersonMsg = true;                                
                break;
            }
        }

        if(!isPersonMsg) return;

        ROS_WARN("[MISSION 1] Person msg detected");
        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if((*it).label == "person" && (*it).pose.position.x <= pedestrian_stop_distance_threshold
                && it->pose.position.y >= pedestrian_right_distance_threshold 
                && it->pose.position.y <= pedestrian_left_distance_threshold ){
                ROS_WARN("[MISSION 1] STOP!");
                avc_interface.enableEstop(true);
                avc_interface.publish();
                
                return;
            }
        }        

        ROS_WARN("[MISSION 1] GO!");
        avc_interface.enableEstop(false); 
        avc_interface.publish();
        return;
    }

    
    virtual void set_param(object_manager::ObjectManagerConfig &config)
    { 
        pedestrian_stop_distance_threshold = config.pedestrian_STOP_distance_threshold; 
        pedestrian_right_distance_threshold = config.pedestrian_RIGHT_distance_threshold; 
        pedestrian_left_distance_threshold = config.pedestrian_LEFT_distance_threshold; 
    }
private:
    double pedestrian_stop_distance_threshold = 30.0;
    double pedestrian_right_distance_threshold = -0.9;
    double pedestrian_left_distance_threshold = 4.55;
};