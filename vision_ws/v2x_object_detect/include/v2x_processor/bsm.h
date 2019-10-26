#pragma once

#include <ros/ros.h>
#include <v2x_msgs/v2x_info.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <v2x_processor/tf_mat.h>
#include <object_manager_msgs/combined.h>


class BSMProcessor{
public:
    BSMProcessor(ros::NodeHandle& nh, std::string combined_topic_name);
    void processBSM(const v2x_msgs::v2x_infoConstPtr& ptr);
    void setTfProcessor(const TransformationMatrixProcessor& tf);
    void publish();
private:
    jsk_recognition_msgs::BoundingBoxArray boundingBoxAry;
    autoware_msgs::DetectedObjectArray detectedObjAry;
    TransformationMatrixProcessor tf_processor;    

    ros::Publisher combined_msg_pub;    
    object_manager_msgs::combined combined_msg;
};