#pragma once

#include <ros/ros.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <object_manager_msgs/combined.h>
#include <visualization_msgs/MarkerArray.h>
#include <map>
#include <mutex>
#include <std_msgs/Bool.h>
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include "object_manager/genAutowareObstacles.h"
#include <geometry_msgs/TwistStamped.h>

class AVCInterface{
    AVCInterface(): private_nh("~"){//singletone. AVCInterface object must be one
        estop_pub = nh.advertise<std_msgs::Bool>("obj_mgr_estop", 100);
        obstacles_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("tracked_objects", 100);
        goal_speed_pub = nh.advertise<std_msgs::Float32>("mission_manager_maxvel", 10);
        debug_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles_debug", 100);
        twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd2", 10);

        twist_sub = nh.subscribe("twist_cmd", 10, &AVCInterface::twistCB, this);

        loadPredefinedObstacles();
        initializeEstopAndObstacles();
        bControlVelocity = false;
    }
public:
    static AVCInterface* getInstancePtr(){
        if (!self) self = new AVCInterface();
        return self;
    }
    void enableEstop(bool value){
        processEstop.data = value;
    }
    void setObstacle(const std::string key, const autoware_msgs::DetectedObject& obstacle){
        enabledObstacles[key] = obstacle;    //overwriting is included
        obstacles_msg.objects.resize(0);
        for(auto p : enabledObstacles)
            obstacles_msg.objects.push_back(p.second);
    }
    void eraseObstacle(const std::string key){
        enabledObstacles.erase(key);
        obstacles_msg.objects.resize(0);
        for(auto p : enabledObstacles)
            obstacles_msg.objects.push_back(p.second);
    }
    void publishGoalSpeed(double goal){
        std_msgs::Float32 msg;
        msg.data = goal;
        goal_speed_pub.publish(msg);
    }
    void publish(){
        estop_pub.publish(processEstop);
        obstacles_pub.publish(obstacles_msg);

        publishDebugObstaclesMarkers();//erase this in avc contest
    }

    void initializeEstopAndObstacles(){
        enableEstop(false);
        enabledObstacles = {};
        obstacles_msg.objects.resize(0);
        setPredefinedObstacles();     
    }
    void publishDebugObstaclesMarkers(){
        visualization_msgs::MarkerArray m;
        for(auto& p : enabledObstacles)
            addDebugMarkersFromObstacles(m, p.second);
        debug_pub.publish(m);
    }
    void setCurpos(const geometry_msgs::PoseStamped& curpos){
        this->curpos = curpos;
    }
    const geometry_msgs::PoseStamped& getCurpos() const{
        return curpos;
    }
    void lock(){
        avc_lock.lock();
    }
    void unlock(){
        avc_lock.unlock();
    }

    void twistCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        geometry_msgs::TwistStamped twist = *ptr;
        if (bControlVelocity){            
            bool isLinearTwist = true;
            if (std::fabs(ptr->twist.angular.z) > 0.01) isLinearTwist = false;
            if (isLinearTwist) twist.twist.linear.x = twist_v;
        }
        twist_pub.publish(twist);
    }
    
    void setGoalVelocity(double v){
        twist_v = v;
    }

    void enableControlTwist(bool value){
        bControlVelocity = value;
    }

private:
    void loadPredefinedObstacles(){
        constexpr static int NEWLINE_CHECKER = -99999999;
        std::vector<double> poly_raw;
        if (!private_nh.getParam("predefined_polygons", poly_raw))
            throw std::runtime_error("set debug_polygons!");
        
        std::vector<geometry_msgs::Polygon> polys;
        polys.emplace_back();
        for(auto it = poly_raw.begin(); it != poly_raw.end(); ){
            geometry_msgs::Point32 p;
            p.x = *it++;
            p.y = *it++;
            polys.back().points.push_back(p);
            if (*it == NEWLINE_CHECKER) {
                polys.emplace_back();
                it++;
            }
        }
        polys.pop_back();
        
        obstacles_debug.header.frame_id = "map";
        obstacles_debug.header.stamp = ros::Time::now();
        for(auto& poly : polys){
            obstacles_debug.objects.emplace_back();
            genObstacleMsgFromPolygon(obstacles_debug.objects.back(), poly, 0.5);
        }
    }

    void setPredefinedObstacles(){
        for(size_t i = 0 ; i < obstacles_debug.objects.size(); ++i){
            std::string n = "debug_" + std::to_string(i);
            setObstacle(n, obstacles_debug.objects[i]);
        }
    }


private:
    ros::Publisher estop_pub, obstacles_pub, goal_speed_pub, twist_pub;
    ros::Subscriber twist_sub;
    ros::NodeHandle nh, private_nh;

    std::mutex avc_lock;
    std_msgs::Bool processEstop;
    std::map<std::string, autoware_msgs::DetectedObject> enabledObstacles;

    autoware_msgs::DetectedObjectArray obstacles_msg;    

    ros::Publisher debug_pub;
    static AVCInterface* self;

    autoware_msgs::DetectedObjectArray obstacles_debug;

    geometry_msgs::PoseStamped curpos;

    bool bActiveEstop;
    bool bControlVelocity;
    double twist_v, twist_w;
};
