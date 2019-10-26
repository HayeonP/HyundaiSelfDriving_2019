#pragma once
#include <object_manager/mission_processor.h>
#include <autoware_msgs/DetectedObject.h>
#include <object_manager/ROIcheck.h>
#include <geometry_msgs/PolygonStamped.h>

enum {
    SPAT_1= 0, SPAT_2, SPAT_3, N_SPAT
};
enum {
    DECISION_NONE, DECISION_GO, DECISION_REDLIGHT_GO, DECISION_REDLIGHT_WAIT
};

enum {SPAT_RED_LABEL = 0, SPAT_GREEN_LABEL = 1, SPAT_YELLOW_LABEL = 2};
constexpr static double WAIT_VELOCITY_MPS = 2;//7 kph

class TrafficSignProcessor : public MissionProcessor{
public:
    TrafficSignProcessor() : private_nh("~"){
        if (!private_nh.getParam("spat_stop_area_x", spat_stop_area_x))
            throw std::runtime_error("set spat_stop_area_x!");
        if (!private_nh.getParam("decision_distance", decision_distance))
            throw std::runtime_error("set decision_distance!");
        if (!private_nh.getParam("max_speed", max_speed))
            throw std::runtime_error("set max_speed!");
        if (!private_nh.getParam("mission_end_speed", mission_end_speed))
            throw std::runtime_error("set mission_end_speed!");
        if (!private_nh.getParam("dist_margin", dist_margin))
            throw std::runtime_error("set dist_margin!");
        if (!private_nh.getParam("time_margin", time_margin))
            throw std::runtime_error("set time_margin!");
        decision_state = DECISION_NONE;
        current_spat = SPAT_1;//when mission started, the stop line's x coordinate is 
        //below car's x coordinate.

        bPassedAllStopline = false;
    }

    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){

        auto& detectedObj = msg.obj_ary;
        
        //check if spat msg
        bool isSpatMsg = false;
        const std::vector<std::string> signal_names{"signal_0", "signal_1", "signal_2"};
        auto n = signal_names.cbegin();
        for(auto&& obj : detectedObj.objects){
            if (obj.label == *n){
                isSpatMsg = true;
                break;
            }
            n++;
        }

        if (isSpatMsg == false)
            return ;

        //hard coded
        const double x_cur = avc_interface.getCurpos().pose.position.x;
        double dist = x_cur - spat_stop_area_x[current_spat];
        if (current_spat == SPAT_1){
            if (dist > 13){ // meter
                dist = 10 + (dist - 10) / 0.9064611; // cos(24.9792) 
            }
        }
        ROS_WARN("[MISSION 3] dist : %lf", dist);
        
        double remaining_time = detectedObj.objects[current_spat].score;
        
        if (!bPassedAllStopline){
            switch(decision_state){
            case DECISION_NONE:{
                ROS_WARN("DECISION__NONE");
                if (dist > decision_distance) {
                    goal_speed = max_speed;
                    break;
                }

                if (detectedObj.objects[current_spat].indicator_state == SPAT_GREEN_LABEL){
                    printf("DECISION NONE - GREEN : remaining - %lf", remaining_time);
                    remaining_time -= 0.5; // tolerance
                    //if (current_spat == SPAT_1) remaining_time -= 1; //distance error tolerance

                    if (remaining_time < time_margin) { //not enough time with long distance
                        goal_speed = WAIT_VELOCITY_MPS;//10 kph
                    }

                    double enough_speed = dist / remaining_time;
                    
                    if (enough_speed > max_speed) { //vehicle's controllability lacks
                        goal_speed = WAIT_VELOCITY_MPS;//10 kph
                    }
                    else {
                        goal_speed = max_speed;
                        decision_state = DECISION_GO;
                    }
                }
                else if (detectedObj.objects[current_spat].indicator_state == SPAT_YELLOW_LABEL){
                    printf("DECISION NONE - YELLOW : remaining - %lf\n", remaining_time);                    
                    goal_speed = WAIT_VELOCITY_MPS;
                }
                else if (detectedObj.objects[current_spat].indicator_state == SPAT_RED_LABEL){
                    printf("DECISION NONE - RED : remaining - %lf\n", remaining_time);                    
                    
                    if (dist > dist_margin){
                        double enough_speed = (dist - dist_margin) / remaining_time;
                        if (enough_speed > max_speed) enough_speed = max_speed;
                        goal_speed = enough_speed;
                        decision_state = DECISION_REDLIGHT_GO;
                    }
                    else {
                        avc_interface.enableEstop(true);
                        decision_state = DECISION_REDLIGHT_WAIT;
                    }
                }

                break;
            }
            case DECISION_GO:
                printf("DECISION__GO\n");                
                if (dist < 0) {
                    decision_state = DECISION_NONE;
                    current_spat++;
                    if (current_spat > SPAT_3) bPassedAllStopline = true;
                } 
                break;
            case DECISION_REDLIGHT_GO:
                printf("DECISION__REDLIGHT_GO\n");                
                if (remaining_time < time_margin){
                    goal_speed = max_speed;
                    decision_state = DECISION_GO;
                } else{
                    if (dist < dist_margin){ // too much time with short distance
                        avc_interface.enableEstop(true);
                        decision_state = DECISION_REDLIGHT_WAIT;
                    }
                    else {
                        double enough_speed = (dist - dist_margin) / remaining_time;
                        if (enough_speed > max_speed) enough_speed = max_speed;
                        goal_speed = enough_speed;
                    }
                }
            case DECISION_REDLIGHT_WAIT: //merely happen
                printf("DECISION_REDLIGHT_WAIT\n");
                if (remaining_time < time_margin){
                    avc_interface.enableEstop(false);
                    goal_speed = max_speed;
                    decision_state = DECISION_GO;
                }
                break;
            default : ROS_ERROR("impossible situation happened"); break;
            }
            
        } else goal_speed = mission_end_speed;
        
        avc_interface.publish();
        avc_interface.publishGoalSpeed(goal_speed);
    }

    virtual void set_param(object_manager::ObjectManagerConfig &config)
    { 
    }
private:
    ros::NodeHandle private_nh, nh;

    double decision_distance;
    double goal_speed;
    double max_speed, mission_end_speed;
    double dist_margin, time_margin;

    std::vector<double> spat_stop_area_x;
    int decision_state;
    int current_spat;
    bool bPassedAllStopline;

};
