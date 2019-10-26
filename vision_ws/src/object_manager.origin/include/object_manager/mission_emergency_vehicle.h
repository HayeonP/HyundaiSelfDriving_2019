#pragma once
#include <object_manager/mission_processor.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_manager/genAutowareObstacles.h>

class EmergencyVehicleProcessor : public MissionProcessor{
public:
    EmergencyVehicleProcessor() : private_nh("~"),
        e_car_poly_gap(0.5)
    {
        std::vector<double> section_tmp;
        if (!private_nh.getParam("emergency_critical_section", section_tmp))
            throw std::runtime_error("set emergency_critical_section!");
        
        section_poly.points.resize(4);
        for(int i = 0 ; i < 4; ++i){
            section_poly.points[i].x = section_tmp[2*i];
            section_poly.points[i].y = section_tmp[2*i + 1];
        }
    }
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){
        
        // 기술 교류회때 응급차량 테스트시 미션 수행 이후 패트롤에 의한 오작동 발생시 한번만 수행하게 하자.
        auto& detectedObj = msg.obj_ary;

        bool processMission = false;
        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if(it->label == "AMB" && (!emergency_car_appeared)){
                processMission = true;
                break;
            }
            if(it->label == "car" && emergency_car_appeared && it->convex_hull.polygon.points.size()){
                processMission = true;
                break;
            }
        }
        if (!processMission) return;

        if(!emergency_car_appeared){ // AMB was not detected before            
            for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
                if(it->label == "AMB" && (-it->pose.position.x <= AMB_start_distance)){
                    emergency_car_appeared = true;
                    ROS_WARN("[MISSION 5] AMB Detected!");
                    
                    double new_x = avc_interface.getCurpos().pose.position.x + 30;
                    if (new_x < section_poly.points[2].x) {
                        makeFakebox = true;
                        section_poly.points[0].x = new_x;
                        section_poly.points[1].x = new_x;
                    }
                    else makeFakebox = false;
                    
                    amb_detected_time = ros::Time::now();
                    break;
                }   
            }
        }
        else{ // AMB is located nearby
            double dt = (ros::Time::now() - amb_detected_time).toSec();
            if (dt > 3){
                ROS_WARN("[MISSION 5] dt : %lf", dt);
                ROS_WARN("[MISSION 5] AMB is located nearby!");
                for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){                
                    if( it->label == "car" && it->pose.position.x >= AMB_escape_distance ){
                        ROS_WARN("[MISSION 5] distance : %lf", it->pose.position.x);                                                
                        if (!it->convex_hull.polygon.points.size()) break;                        
                        emergency_car_appeared = false;
                        ROS_WARN("[MISSION 5] AMB disappeared! / distance : %lf", it->pose.position.x);
                    }
                }
            }
        }

        if (emergency_car_appeared){
            ROS_WARN("[MISSION 5] Changing lane!");
            if (makeFakebox){
                autoware_msgs::DetectedObject obs;
                genObstacleMsgFromPolygon(obs, section_poly, 0.5);
                avc_interface.setObstacle("AMB", obs);
            }
            else ROS_WARN("[MISSION 5] X make fakebox!");
        } else{
            ROS_WARN("[MISSION 5] No AMB!");
            avc_interface.eraseObstacle("AMB");
        }
        avc_interface.publish();

    }

    void set_param(object_manager::ObjectManagerConfig &config)
    {
    }
private:
    ros::Publisher e_vehicle_debug_pub;
    ros::NodeHandle private_nh, nh;
    geometry_msgs::Polygon section_poly;
    double e_car_poly_gap;
    bool emergency_car_appeared = false;
    const double AMB_escape_distance = 14.0;
    const double AMB_start_distance = 10.0;
    ros::Time amb_detected_time;
    bool makeFakebox;
};