#pragma once
#include <object_manager/mission_processor.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

class PedestrianProcessor : public MissionProcessor{
public:
    PedestrianProcessor(){
        workzone_sub = nh.subscribe("workzone_blocked_paths_raw", 10, &PedestrianProcessor::workzoneCB, this);
        workzone_pub = nh.advertise<std_msgs::Int32MultiArray>("workzone_blocked_paths", 10);
        estop_mission1_pub = nh.advertise<std_msgs::Bool>("estop_mission1", 10);
        twist_sub = nh.subscribe("pure_pursuit_ok", 10, &PedestrianProcessor::okCB, this);
    }
    void okCB(const std_msgs::BoolConstPtr& ){
        pubedWorkzone = true;        
    }

    void workzoneCB(const std_msgs::Int32MultiArrayConstPtr& ptr){
        if (pubedWorkzone) return;
        workzone_msg = *ptr;
        if (!workzone_msg.data.size()) return;
        workzone_pub.publish(workzone_msg);
    }
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


        bool estop_in_mission1 = false;
        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if((*it).label == "person" && (*it).pose.position.x <= pedestrian_stop_distance_threshold
                && it->pose.position.y >= pedestrian_right_distance_threshold 
                && it->pose.position.y <= pedestrian_left_distance_threshold ){
                avc_interface.enableEstop(true);
                avc_interface.publish();
                
                estop_in_mission1 = true;
                std_msgs::Bool m;
                m.data = estop_in_mission1;
                estop_mission1_pub.publish(m);
                
                return;
            }
        }
        std_msgs::Bool m;
        m.data = estop_in_mission1;
        estop_mission1_pub.publish(m);

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
    ros::NodeHandle nh;
    ros::Subscriber workzone_sub, replan_sub, twist_sub;
    ros::Publisher workzone_pub, estop_mission1_pub;

    double pedestrian_stop_distance_threshold = 30.0;
    double pedestrian_right_distance_threshold = -0.9;
    double pedestrian_left_distance_threshold = 4.55;

    std_msgs::Int32MultiArray workzone_msg;
    bool estop_in_mission1;

    bool pubedWorkzone = false;
};
