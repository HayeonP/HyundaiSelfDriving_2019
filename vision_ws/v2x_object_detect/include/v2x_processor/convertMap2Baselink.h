#pragma once
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
void convertMap2Baselink_boundingbox(jsk_recognition_msgs::BoundingBoxArray& boxAry);
void convertMap2Baselink_detectedobj(autoware_msgs::DetectedObjectArray& detectedobj);