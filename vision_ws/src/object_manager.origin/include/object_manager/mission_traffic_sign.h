#pragma once
#include <object_manager/mission_processor.h>
#include <autoware_msgs/DetectedObject.h>
#include <object_manager/ROIcheck.h>
#include <geometry_msgs/PolygonStamped.h>

enum {
    SPAT_1= 0, SPAT_2, SPAT_3, N_SPAT
};
enum {
    DECISION_NONE, DECISION_GO, DECISION_ESTOP
};

enum {SPAT_RED_LABEL = 0, SPAT_GREEN_LABEL = 1, SPAT_YELLOW_LABEL = 2};


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
                dist = 13 + (dist - 13) / 0.9064611; // cos(24.9792) 
            }
        }
        ROS_WARN("[MISSION 3] dist : %lf", dist);
                
        if (!bPassedAllStopline){
            switch(decision_state){
            case DECISION_NONE:{
                if (dist > decision_distance) {
                    ROS_WARN("dist > decision_distance!");
                    goal_speed = max_speed;
                    break;
                }

                double remaining_time = detectedObj.objects[current_spat].score;
                
                if (detectedObj.objects[current_spat].indicator_state == SPAT_GREEN_LABEL){
                    remaining_time -= 0.5; // tolerance

                    if (remaining_time < 2) {
                        ROS_WARN("remaining_time < 2! decrease speed");
                        goal_speed = 3;//10 kph
                    }

                    double enough_speed = dist / remaining_time;
                    
                    if (enough_speed > max_speed) {
                        ROS_WARN("not enough time! decrease car speed");
                        goal_speed = 3;//10 kph
                    }
                    else {
                        ROS_WARN("speed up!");
                        goal_speed = max_speed;
                        decision_state = DECISION_GO;
                    }
                }
                else if (detectedObj.objects[current_spat].indicator_state == SPAT_RED_LABEL){
                    remaining_time += 0.5; //tolerance
                    double enough_speed = dist / remaining_time;
                    if (enough_speed > max_speed) enough_speed = max_speed;
                    goal_speed = enough_speed;
                    ROS_WARN("red light. set appropriate speed : %lf kph", goal_speed * 3.6);
                }
                else {
                    goal_speed = 3;//10 kph
                }

                break;
            }
            case DECISION_GO:
                if (dist < 0) {
                    ROS_WARN("[Mission 3] : DECISION_GO - passed stop line");                
                    decision_state = DECISION_NONE;
                    current_spat++;
                    if (current_spat > SPAT_3) bPassedAllStopline = true;
                } else
                    ROS_WARN("[Mission 3] : DECISION_GO - not passed stop line");
                
                break;
            default : ROS_ERROR("impossible situation happened"); break;
            }
        } else goal_speed = mission_end_speed;

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

    std::vector<double> spat_stop_area_x;
    int decision_state;
    int current_spat;
    bool bPassedAllStopline;

};
