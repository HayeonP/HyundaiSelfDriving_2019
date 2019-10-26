#include "v2x_processor/convertMap2Baselink.h"
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>

void convertMap2Baselink_boundingbox(jsk_recognition_msgs::BoundingBoxArray& boxAry){
    //this function changes only geometry_msgs::Pose
    static tf::TransformListener listener;
    return;//map-baselink tf is not stable
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/base_link", "/map",  
            ros::Time::now(), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("map-baselink tf error ! : %s", ex.what());
        return;
    }
    if (boxAry.header.frame_id == "map"){
        boxAry.header.frame_id = "base_link";

        const double &x_tf = transform.getOrigin().x();
        const double &y_tf = transform.getOrigin().y();
        const tf::Quaternion& quat = transform.getRotation();
        const double &yaw_tf = tf::getYaw(quat);

        for(auto& box : boxAry.boxes){
            //consider only yaw rotation - 
            if (box.header.frame_id == "map"){
                box.header.frame_id = "base_link";

                //calc new x, y
                const double& x_target = box.pose.position.x;
                const double& y_target = box.pose.position.y;

                box.pose.position.x = std::cos(yaw_tf) * x_target - std::sin(yaw_tf) * y_target + x_tf;
                box.pose.position.y = std::sin(yaw_tf) * x_target + std::cos(yaw_tf) * y_target + y_tf;

                //calc new quaternion
                tf::Quaternion quat_target = tf::createQuaternionFromYaw(
                    tf::getYaw(box.pose.orientation)
                );

                tf::quaternionTFToMsg(
                     quat * quat_target * quat.inverse(),
                     box.pose.orientation
                );
            }
        }
    }
}

void convertMap2Baselink_detectedobj(autoware_msgs::DetectedObjectArray& detectedobj){
    //this function changes only geometry_msgs::Pose  
    static tf::TransformListener listener;  
    return;//map-baselink tf is not stable    
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/base_link", "/map",  
            ros::Time::now(), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("map-baselink tf error ! : %s", ex.what());
        return;
    }
    if (detectedobj.header.frame_id == "map"){
        detectedobj.header.frame_id = "base_link";

        const double &x_tf = transform.getOrigin().x();
        const double &y_tf = transform.getOrigin().y();
        const tf::Quaternion& quat = transform.getRotation();
        const double &yaw_tf = tf::getYaw(quat);

        for(auto& obj : detectedobj.objects){
            //consider only yaw rotation - 
            if (obj.header.frame_id == "map"){
                obj.header.frame_id = "base_link";

                //calc new x, y
                const double& x_target = obj.pose.position.x;
                const double& y_target = obj.pose.position.y;

                obj.pose.position.x = std::cos(yaw_tf) * x_target - std::sin(yaw_tf) * y_target + x_tf;
                obj.pose.position.y = std::sin(yaw_tf) * x_target + std::cos(yaw_tf) * y_target + y_tf;

                //calc new quaternion
                tf::Quaternion quat_target = tf::createQuaternionFromYaw(
                    tf::getYaw(obj.pose.orientation)
                );
                
                tf::quaternionTFToMsg(
                     quat * quat_target * quat.inverse(),
                     obj.pose.orientation
                );
            }
        }
    }
}