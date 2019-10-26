#include <v2x_processor/bsm.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_broadcaster.h>

BSMProcessor::BSMProcessor(ros::NodeHandle& nh, std::string combined_topic_name){
    
    combined_msg_pub = nh.advertise<object_manager_msgs::combined>(
        combined_topic_name, 10
    );

    boundingBoxAry.boxes.resize(1);
    detectedObjAry.objects.resize(1);
}

void BSMProcessor::setTfProcessor(const TransformationMatrixProcessor& tf){
    tf_processor = tf;
}

void BSMProcessor::processBSM(const v2x_msgs::v2x_infoConstPtr& ptr){
    ROS_INFO("processBSM");

    boundingBoxAry.header.seq++;
    boundingBoxAry.header.stamp = ros::Time::now();
    boundingBoxAry.header.frame_id = "map";
    boundingBoxAry.boxes.resize(1);
    jsk_recognition_msgs::BoundingBox& box = boundingBoxAry.boxes[0];

    box.header.seq++;
    box.header.stamp = ros::Time::now();
    box.header.frame_id = "map";

    ROS_INFO("[bsm] lat, lon : %lf %lf", ptr->bsm_lat/10000000.0, ptr->bsm_lon/10000000.0);
    tf_processor.convertLatLonAltToMapXYZ(ptr->bsm_lat/10000000.0, ptr->bsm_lon/10000000.0, 0 ,
    box.pose.position.x, box.pose.position.y, box.pose.position.z);
    box.pose.orientation = tf::createQuaternionMsgFromYaw(ptr->bsm_angle);

    box.dimensions.x = ptr->bsm_size_width/100.0;
    box.dimensions.y = ptr->bsm_size_length/100.0;
    box.dimensions.z = 3;

    detectedObjAry.objects.resize(1);
    autoware_msgs::DetectedObject& obj = detectedObjAry.objects.front();
    obj.header.seq++;
    obj.header.stamp = ros::Time::now();
    obj.header.frame_id = "map";

    obj.label = "emergencyVehicle";
    obj.pose = box.pose;
    obj.dimensions = box.dimensions;

    convertMap2Baselink_boundingbox(boundingBoxAry);
    convertMap2Baselink_detectedobj(detectedObjAry);
    publish();
}

void BSMProcessor::publish(){
    combined_msg.header = boundingBoxAry.header;
    combined_msg.bb_ary = boundingBoxAry;
    combined_msg.obj_ary = detectedObjAry;
    combined_msg_pub.publish(combined_msg);
}