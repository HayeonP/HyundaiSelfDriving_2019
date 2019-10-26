#pragma once

#include <ros/ros.h>
#include <v2x_msgs/v2x_info.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <vector>
#include <autoware_msgs/DetectedObjectArray.h>
#include <v2x_processor/tf_mat.h>
#include <object_manager_msgs/combined.h>

class TIMProcessor{
private:
    struct PoseXYZ{
        double x, y, z;
    };
    struct LatLon{
        double lat, lon; 
    };
    typedef std::vector<LatLon> LatLonVecType;
    typedef std::vector<LatLonVecType> AnchorLatLonVecType;
    typedef std::vector<PoseXYZ> XYZVecType;
    typedef std::vector<XYZVecType> AnchorXYZVecType;
    static constexpr int N_PATH = 8; //for workzone processing, I number each path.
    static constexpr double KCITY_ALTITUDE = 23.5; //Kcity altitude. It don't have to be precise 

public:
    TIMProcessor(ros::NodeHandle& nh, std::string combined_topic_name);
    void processTIM(const v2x_msgs::v2x_infoConstPtr& ptr);
    void setTfProcessor(const TransformationMatrixProcessor& tf);
    void publish();
    void findBlockedPathAndPublish(const v2x_msgs::v2x_infoConstPtr& ptr);
    void publishFakeWorkzone(int);
private:
    ros::Publisher tim_boundingbox_pub, tim_detectedobj_pub;
    ros::Publisher tim_workzone_debug;
    ros::Publisher tim_mission_manager_plugin_pub;
    ros::Publisher combined_msg_pub;    
    object_manager_msgs::combined combined_msg;

    jsk_recognition_msgs::BoundingBoxArray boundingBoxAry;
    autoware_msgs::DetectedObjectArray detectedObjAry;
    
    TransformationMatrixProcessor tf_processor;   
    ros::NodeHandle private_nh; 

    AnchorLatLonVecType anchor_latlon_vec;
    AnchorXYZVecType anchor_xyz_vec;
    GeographicLib::Geocentric earth;
    PoseXYZ xyz_standard;
};