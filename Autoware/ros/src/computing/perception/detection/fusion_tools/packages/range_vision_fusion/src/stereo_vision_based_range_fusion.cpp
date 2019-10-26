/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * range_vision_fusion_node.cpp
 *
 *  Created on: July, 05th, 2018
 */

#include "range_vision_fusion/stereo_vision_based_range_fusion.h"
#define TF_Z 1.87615


int main(int argc, char **argv)
{
    ros::init(argc, argv, __APP_NAME__);

    RosRangeVisionFusionApp app;

    app.Run();

    return 0;
}

cv::Point3f
RosRangeVisionFusionApp::TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform)
{
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_t = in_transform * tf_point;
    return cv::Point3f(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

cv::Point2i
RosRangeVisionFusionApp::ProjectPoint(const cv::Point3f &in_point, std::string camera_name)
{   
    int u, v;
    if(camera_name == left_camera_name_){
        u = int(in_point.x * left_fx_ / in_point.z + left_cx_);
        v = int(in_point.y * left_fy_ / in_point.z + left_cy_);
    }
    else {
        u = int(in_point.x * right_fx_ / in_point.z + right_cx_);
        v = int(in_point.y * right_fy_ / in_point.z + right_cy_);
    }
    
    return cv::Point2i(u, v);
}

autoware_msgs::DetectedObject
RosRangeVisionFusionApp::TransformObject(const autoware_msgs::DetectedObject &in_detection,
                                                                       const tf::StampedTransform& in_transform)
{
    autoware_msgs::DetectedObject t_obj = in_detection;

    tf::Vector3 in_pos(in_detection.pose.position.x,
                       in_detection.pose.position.y,
                       in_detection.pose.position.z);
    tf::Quaternion in_quat(in_detection.pose.orientation.x,
                           in_detection.pose.orientation.y,
                           in_detection.pose.orientation.w,
                           in_detection.pose.orientation.z);

    tf::Vector3 in_pos_t = in_transform * in_pos;
    tf::Quaternion in_quat_t = in_transform * in_quat;

    t_obj.pose.position.x = in_pos_t.x();
    t_obj.pose.position.y = in_pos_t.y();
    t_obj.pose.position.z = in_pos_t.z();

    t_obj.pose.orientation.x = in_quat_t.x();
    t_obj.pose.orientation.y = in_quat_t.y();
    t_obj.pose.orientation.z = in_quat_t.z();
    t_obj.pose.orientation.w = in_quat_t.w();

    return t_obj;
}

void
RosRangeVisionFusionApp::CalculateObjectFeatures(autoware_msgs::DetectedObject &in_out_object, bool in_estimate_pose)
{

    float min_x=std::numeric_limits<float>::max();float max_x=-std::numeric_limits<float>::max();
    float min_y=std::numeric_limits<float>::max();float max_y=-std::numeric_limits<float>::max();
    float min_z=std::numeric_limits<float>::max();float max_z=-std::numeric_limits<float>::max();
    float average_x = 0, average_y = 0, average_z = 0, length, width, height;
    pcl::PointXYZ centroid, min_point, max_point, average_point;

    std::vector<cv::Point2f> object_2d_points;

    pcl::PointCloud<pcl::PointXYZ> in_cloud;
    pcl::fromROSMsg(in_out_object.pointcloud, in_cloud);

    for (const auto &point : in_cloud.points)
    {
        average_x+=point.x;		average_y+=point.y;		average_z+=point.z;
        centroid.x += point.x; centroid.y += point.y;	centroid.z += point.z;

        if(point.x<min_x)	min_x = point.x;
        if(point.y<min_y)	min_y = point.y;
        if(point.z<min_z)	min_z = point.z;
        if(point.x>max_x)	max_x = point.x;
        if(point.y>max_y)	max_y = point.y;
        if(point.z>max_z)	max_z = point.z;

        cv::Point2f pt;
        pt.x = point.x;
        pt.y = point.y;
        object_2d_points.push_back(pt);
    }
    min_point.x = min_x;	min_point.y = min_y;	min_point.z = min_z;
    max_point.x = max_x;	max_point.y = max_y;	max_point.z = max_z;

    if (in_cloud.points.size() > 0)
    {
        centroid.x /= in_cloud.points.size();
        centroid.y /= in_cloud.points.size();
        centroid.z /= in_cloud.points.size();

        average_x /= in_cloud.points.size();
        average_y /= in_cloud.points.size();
        average_z /= in_cloud.points.size();
    }

    average_point.x = average_x; average_point.y = average_y;	average_point.z = average_z;

    length = max_point.x - min_point.x;
    width = max_point.y - min_point.y;
    height = max_point.z - min_point.z;

    geometry_msgs::PolygonStamped  convex_hull;
    std::vector<cv::Point2f> hull_points;
    if (object_2d_points.size() > 0)
        cv::convexHull(object_2d_points, hull_points);

    convex_hull.header = in_out_object.header;
    for (size_t i = 0; i < hull_points.size() + 1 ; i++)
    {
        geometry_msgs::Point32 point;
        point.x = hull_points[i%hull_points.size()].x;
        point.y = hull_points[i%hull_points.size()].y;
        point.z = min_point.z;
        convex_hull.polygon.points.push_back(point);
    }

    for (size_t i = 0; i < hull_points.size() + 1 ; i++)
    {
        geometry_msgs::Point32 point;
        point.x = hull_points[i%hull_points.size()].x;
        point.y = hull_points[i%hull_points.size()].y;
        point.z = max_point.z;
        convex_hull.polygon.points.push_back(point);
    }

    double rz = 0;
    if (in_estimate_pose)
    {
        cv::RotatedRect box = cv::minAreaRect(hull_points);
        rz = box.angle*3.14/180;
        in_out_object.pose.position.x = box.center.x;
        in_out_object.pose.position.y = box.center.y;
        in_out_object.dimensions.x = box.size.width;
        in_out_object.dimensions.y = box.size.height;
    }

    in_out_object.convex_hull = convex_hull;

    in_out_object.pose.position.x = min_point.x + length/2;
    in_out_object.pose.position.y = min_point.y + width/2;
    in_out_object.pose.position.z = min_point.z + height/2;

    in_out_object.dimensions.x = ((length<0)?-1*length:length);
    in_out_object.dimensions.y = ((width<0)?-1*width:width);
    in_out_object.dimensions.z = ((height<0)?-1*height:height);

    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
    tf::quaternionTFToMsg(quat, in_out_object.pose.orientation);
}

autoware_msgs::DetectedObject RosRangeVisionFusionApp::MergeObjects(const autoware_msgs::DetectedObject &in_object_a,
                                           const autoware_msgs::DetectedObject & in_object_b)
{
    autoware_msgs::DetectedObject object_merged;
    object_merged = in_object_b;

    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_merged;

    if (!in_object_a.pointcloud.data.empty())
        pcl::fromROSMsg(in_object_a.pointcloud, cloud_a);
    if (!in_object_b.pointcloud.data.empty())
        pcl::fromROSMsg(in_object_b.pointcloud, cloud_b);

    cloud_merged = cloud_a + cloud_b;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_merged, cloud_msg);
    cloud_msg.header = object_merged.pointcloud.header;

    object_merged.pointcloud = cloud_msg;

    return object_merged;

}

double RosRangeVisionFusionApp::GetDistanceToObject(const autoware_msgs::DetectedObject &in_object)
{
    return sqrt(in_object.dimensions.x*in_object.dimensions.x +
                in_object.dimensions.y*in_object.dimensions.y +
                in_object.dimensions.z*in_object.dimensions.z);
}

void RosRangeVisionFusionApp::CheckMinimumDimensions(autoware_msgs::DetectedObject &in_out_object)
{
    if (in_out_object.label == "car")
    {
        if (in_out_object.dimensions.x < car_depth_)
            in_out_object.dimensions.x = car_depth_;
        if (in_out_object.dimensions.y < car_width_)
            in_out_object.dimensions.y = car_width_;
        if (in_out_object.dimensions.z < car_height_)
            in_out_object.dimensions.z = car_height_;
    }
    if (in_out_object.label == "person")
    {
        if (in_out_object.dimensions.x < person_depth_)
            in_out_object.dimensions.x = person_depth_;
        if (in_out_object.dimensions.y < person_width_)
            in_out_object.dimensions.y = person_width_;
        if (in_out_object.dimensions.z < person_height_)
            in_out_object.dimensions.z = person_height_;
    }

    if (in_out_object.label == "truck" || in_out_object.label == "bus")
    {
        if (in_out_object.dimensions.x < truck_depth_)
            in_out_object.dimensions.x = truck_depth_;
        if (in_out_object.dimensions.y < truck_width_)
            in_out_object.dimensions.y = truck_width_;
        if (in_out_object.dimensions.z < truck_height_)
            in_out_object.dimensions.z = truck_height_;
    }
}

visualization_msgs::MarkerArray
RosRangeVisionFusionApp::ObjectsToMarkers(const autoware_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray final_markers;

    for(const autoware_msgs::DetectedObject& object : in_objects.objects)
    {
        if (object.label != "unknown"
            && object.pose.position.x != 0
            && object.pose.position.y != 0
            && object.pose.position.z != 0)
        {
            visualization_msgs::Marker marker;
            marker.header = in_objects.header;
            marker.ns = "range_vision_fusion";
            marker.id = object.id;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 1.0;
            marker.text = object.label;
            if (object.id != 0)
                marker.text += " " + std::to_string(object.id);
            marker.pose.position = object.pose.position;
            marker.pose.position.z += 1.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 1.5;
            marker.scale.y = 1.5;
            marker.scale.z = 1.5;

            marker.lifetime = ros::Duration(0.1);
            final_markers.markers.push_back(marker);
        }
    }
    return final_markers;
}

jsk_recognition_msgs::BoundingBoxArray
RosRangeVisionFusionApp::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
    jsk_recognition_msgs::BoundingBoxArray final_boxes;
    final_boxes.header = in_objects.header;

    for(const autoware_msgs::DetectedObject& object : in_objects.objects)
    {
        jsk_recognition_msgs::BoundingBox box;

        box.header = in_objects.header;
        box.label = object.id;
        box.dimensions = object.dimensions;
        box.pose = object.pose;
        box.value = object.score;

        // if (box.dimensions.x > 0 && box.dimensions.y > 0 && box.dimensions.z > 0)
        // {
        final_boxes.boxes.push_back(box);
        // }
    }
    return final_boxes;
}

void
RosRangeVisionFusionApp::VisionDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{

    if (!processing_ && !in_vision_detections->objects.empty())
    {
        processing_ = true;
        if(in_vision_detections->header.frame_id == left_camera_name_)
            SyncedDetectionsCallback(in_vision_detections, right_vision_detections_, cloud_detections_);
        else
            SyncedDetectionsCallback(left_vision_detections_, in_vision_detections, cloud_detections_);
        processing_ = false;
    }
}

void
RosRangeVisionFusionApp::LeftIntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{   
    left_image_size_.height = in_message.height;
    left_image_size_.width = in_message.width;

    left_camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            left_camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
        }
    }

    left_distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
    for (int col = 0; col < 5; col++)
    {
        left_distortion_coefficients_.at<double>(col) = in_message.D[col];
    }

    left_fx_ = static_cast<float>(in_message.P[0]);
    left_fy_ = static_cast<float>(in_message.P[5]);
    left_cx_ = static_cast<float>(in_message.P[2]);
    left_cy_ = static_cast<float>(in_message.P[6]);

    left_intrinsics_subscriber_.shutdown();
    left_camera_info_ok_ = true;
    left_image_frame_id_ = in_message.header.frame_id;
    // ROS_INFO("[%s] Left_CameraIntrinsics obtained.", __APP_NAME__);
    // ROS_INFO("Left Projection Parameter : %f, %f, %f, %f", left_fx_, left_fy_, left_cx_, left_cy_);
}

void
RosRangeVisionFusionApp::RightIntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{   
    right_image_size_.height = in_message.height;
    right_image_size_.width = in_message.width;

    right_camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            right_camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
        }
    }

    right_distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
    for (int col = 0; col < 5; col++)
    {
        right_distortion_coefficients_.at<double>(col) = in_message.D[col];
    }

    right_fx_ = static_cast<float>(in_message.P[0]);
    right_fy_ = static_cast<float>(in_message.P[5]);
    right_cx_ = static_cast<float>(in_message.P[2]);
    right_cy_ = static_cast<float>(in_message.P[6]);

    right_intrinsics_subscriber_.shutdown();
    right_camera_info_ok_ = true;
    right_image_frame_id_ = in_message.header.frame_id;
    // ROS_INFO("[%s] Right_CameraIntrinsics obtained.", __APP_NAME__);
    // ROS_INFO("Right Projection Parameter : %f, %f, %f, %f", right_fx_, right_fy_, right_cx_, right_cy_);
}

tf::StampedTransform
RosRangeVisionFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
    tf::StampedTransform transform;

    // ROS_INFO("%s - > %s", in_source_frame.c_str(), in_target_frame.c_str());
    // ROS_INFO("IN TARGET FRAME %s", in_target_frame.c_str());

    if(in_target_frame == left_image_frame_id_){
        ROS_INFO("LEFT");
        left_camera_lidar_tf_ok_ = false;
        try
        {
            transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
            left_camera_lidar_tf_ok_ = true;
            // ROS_INFO("[%s] Left Camera-Lidar TF obtained", __APP_NAME__);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
        }
    }
    else if(in_target_frame == right_image_frame_id_){
        ROS_INFO("RIGHT");
        right_camera_lidar_tf_ok_ = false;
        try
        {
            transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
            right_camera_lidar_tf_ok_ = true;
            // ROS_INFO("[%s] Right Camera-Lidar TF obtained", __APP_NAME__);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
        }
    }
    else if(in_target_frame== base_id_){
        lidar_base_tf_ok_ = false;
        try
        {
            transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
            lidar_base_tf_ok_ = true;
            // ROS_INFO("[%s] Lidar-Base TF obtained", __APP_NAME__);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
        }      
    }
    else if(in_target_frame==map_name_){
        map_lidar_tf_ok_ = false;
        try
        {
            transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
            map_lidar_tf_ok_ = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_INFO("[%s] %s", __APP_NAME__, ex.what());
        }      
    }
    else{
        try
        {
            transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
        }      
    }

    return transform;
}

void
RosRangeVisionFusionApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
    //get params
    std::string left_camera_info_src, right_camera_info_src, left_detected_objects_vision, right_detected_objects_vision;    
    std::string min_car_dimensions, min_person_dimensions, min_truck_dimensions;
    std::string fused_topic_str = "/detection/combined_objects", fused_boxes_str = "/detection/combined_objects_boxes";
    std::string fused_text_str = "detection/combined_objects_labels";
    std::string name_space_str = ros::this_node::getNamespace();
    bool sync_topics = false;

    // Start
    std::string lidar_topic_str;
    // End

    ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Vision and Range Detections being published.", __APP_NAME__);

    in_private_handle.param<std::string>("left_detected_objects_vision", left_detected_objects_vision, "/detection/image_detector/left_objects");
    ROS_INFO("[%s] left_detected_objects_vision: %s", __APP_NAME__, left_detected_objects_vision.c_str());

    in_private_handle.param<std::string>("right_detected_objects_vision", right_detected_objects_vision, "/detection/image_detector/right_objects");
    ROS_INFO("[%s] right_detected_objects_vision: %s", __APP_NAME__, right_detected_objects_vision.c_str());

    in_private_handle.param<std::string>("left_camera_name", left_camera_name_, "left_camera");
    ROS_INFO("[%s] left_camera_name: %s", __APP_NAME__, left_camera_name_.c_str());

    in_private_handle.param<std::string>("right_camera_name", right_camera_name_, "right_camera");
    ROS_INFO("[%s] right_camera_name: %s", __APP_NAME__, right_camera_name_.c_str());

    in_private_handle.param<std::string>("map_name", map_name_, "map");
    ROS_INFO("[%s] map_name: %s", __APP_NAME__, map_name_.c_str());

    in_private_handle.param<std::string>("left_camera_info_src", left_camera_info_src, "/left_camera_info");
    ROS_INFO("[%s] left_camera_info_src: %s", __APP_NAME__, left_camera_info_src.c_str());

    in_private_handle.param<std::string>("right_camera_info_src", right_camera_info_src, "/right_camera_info");
    ROS_INFO("[%s] right_camera_info_src: %s", __APP_NAME__, right_camera_info_src.c_str());

    in_private_handle.param<double>("overlap_threshold", overlap_threshold_, 0.5);
    ROS_INFO("[%s] overlap_threshold: %f", __APP_NAME__, overlap_threshold_);

    in_private_handle.param<std::string>("min_car_dimensions", min_car_dimensions, "[2,2,4]");//w,h,d
    ROS_INFO("[%s] min_car_dimensions: %s", __APP_NAME__, min_car_dimensions.c_str());

    in_private_handle.param<std::string>("min_person_dimensions", min_person_dimensions, "[1,2,1]");
    ROS_INFO("[%s] min_person_dimensions: %s", __APP_NAME__, min_person_dimensions.c_str());

    in_private_handle.param<std::string>("min_truck_dimensions", min_truck_dimensions, "[2,2,4.5]");
    ROS_INFO("[%s] min_truck_dimensions: %s", __APP_NAME__, min_truck_dimensions.c_str());

    in_private_handle.param<bool>("sync_topics", sync_topics, false);
    ROS_INFO("[%s] sync_topics: %d", __APP_NAME__, sync_topics);

    // Start
    in_private_handle.param<std::string>("lidar_points_raw", lidar_topic_str, "/points_raw");
    ROS_INFO("[%s] lidar_points_raw: %s", __APP_NAME__, lidar_topic_str.c_str());

    in_private_handle.param<std::string>("base_id", base_id_, "base_link");
    ROS_INFO("[%s] base_id: %s", __APP_NAME__, base_id_.c_str());
    // End

    YAML::Node car_dimensions = YAML::Load(min_car_dimensions);
    YAML::Node person_dimensions = YAML::Load(min_person_dimensions);
    YAML::Node truck_dimensions = YAML::Load(min_truck_dimensions);

    if (car_dimensions.size() == 3)
    {
        car_width_ = car_dimensions[0].as<double>();
        car_height_ = car_dimensions[1].as<double>();
        car_depth_ = car_dimensions[2].as<double>();
    }
    if (person_dimensions.size() == 3)
    {
        person_width_ = person_dimensions[0].as<double>();
        person_height_ = person_dimensions[1].as<double>();
        person_depth_ = person_dimensions[2].as<double>();
    }
    if (truck_dimensions.size() == 3)
    {
        truck_width_ = truck_dimensions[0].as<double>();
        truck_height_ = truck_dimensions[1].as<double>();
        truck_depth_ = truck_dimensions[2].as<double>();
    }

    if (name_space_str != "/")
    {
        if (name_space_str.substr(0, 2) == "//")
        {
            name_space_str.erase(name_space_str.begin());
        }
        left_camera_info_src = name_space_str + left_camera_info_src;
        right_camera_info_src = name_space_str + right_camera_info_src;
    }

    //generate subscribers and sychronizers
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, left_camera_info_src.c_str());
    left_intrinsics_subscriber_ = in_private_handle.subscribe(left_camera_info_src,
                                                         1,
                                                         &RosRangeVisionFusionApp::LeftIntrinsicsCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, right_camera_info_src.c_str());
    right_intrinsics_subscriber_ = in_private_handle.subscribe(right_camera_info_src,
                                                         1,
                                                         &RosRangeVisionFusionApp::RightIntrinsicsCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, left_detected_objects_vision.c_str());
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, right_detected_objects_vision.c_str());
    if (!sync_topics)
    {
        left_detections_vision_subscriber_ = in_private_handle.subscribe(left_detected_objects_vision,
                                                                   1,
                                                                   &RosRangeVisionFusionApp::VisionDetectionsCallback, this);
        right_detections_vision_subscriber_ = in_private_handle.subscribe(right_detected_objects_vision,
                                                                   1,
                                                                   &RosRangeVisionFusionApp::VisionDetectionsCallback, this);

        // Start
        detections_lidar_subscriber_ = in_private_handle.subscribe(lidar_topic_str,
                                                                    1,
                                                                    &RosRangeVisionFusionApp::LidarDetectionsCallback, this);
        // End
    }
    else
    {
        left_vision_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                        left_detected_objects_vision, 1);

        right_vision_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                        right_detected_objects_vision, 1);

        // Start
        lidar_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                                        lidar_topic_str, 1);
        detections_synchronizer_ =
                new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                               *left_vision_filter_subscriber_,
                                                               *right_vision_filter_subscriber_,
                                                               *lidar_filter_subscriber_);
        detections_synchronizer_->registerCallback(boost::bind(&RosRangeVisionFusionApp::SyncedDetectionsCallback, this, _1, _2, _3));
        ROS_ERROR("SYNCHED!");
        // End
    }

    publisher_fused_objects_ = node_handle_.advertise<object_manager_msgs::combined>(fused_topic_str, 30);
    publisher_fused_text_ = node_handle_.advertise<visualization_msgs::MarkerArray>(fused_text_str, 30);
    //publisher_fused_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>(fused_boxes_str, 1);

    ROS_INFO("[%s] Publishing fused objects in %s", __APP_NAME__, fused_topic_str.c_str());
    ROS_INFO("[%s] Publishing fused boxes in %s", __APP_NAME__, fused_boxes_str.c_str());

    // Start
    //points_raw_subscriber_ = in_private_handle.subscribe(lidar_topic_str, 1, &RosRangeVisionFusionApp::LidarCallback, this);
    publisher_AMB_test_points_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/AMB_test_points", 30);
    publisher_obj_test_points_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/obj_test_points", 30);
    publisher_img_test_points_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/img_test_points", 30);
    publisher_obj_test_marker_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/obj_test_marker", 30);

    publisher_AMB_object_ = node_handle_.advertise<object_manager_msgs::combined>("/AMB_object", 2000);
    //publisher_AMB_box_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/AMB_box", 1);
    // End

    //debug
    debug_obj_pub_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/debug_obj", 30);
    debug_box_pub_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/debug_box", 30);

}

void
RosRangeVisionFusionApp::Run()
{
    ros::NodeHandle private_node_handle("~");
    tf::TransformListener transform_listener;

    transform_listener_ = &transform_listener;

    InitializeRosIo(private_node_handle);

    ROS_ERROR("[%s] Ready. Waiting for data...", __APP_NAME__);
    sleep(10);
    ROS_ERROR("FUSION NODE READY!");
    //std::cout<<"STEREO FUSION DONE"<<std::endl;
    ros::spin();

    ROS_INFO("[%s] END", __APP_NAME__);
}

RosRangeVisionFusionApp::RosRangeVisionFusionApp()
{
    left_camera_lidar_tf_ok_ = false;
    right_camera_lidar_tf_ok_ = false;
    left_camera_info_ok_ = false;
    right_camera_info_ok_ = false;
    map_lidar_tf_ok_ = false;
    processing_ = false;
    left_image_frame_id_ = "";
    right_image_frame_id_ = "";
    overlap_threshold_ = 0.5;
    empty_frames_ = 0;
}

// Start
object_manager_msgs::combined RosRangeVisionFusionApp::AMB_parse(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud){
    //ROS_INFO("LidarCallback");
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    static int frame_cnt = 0, frame_threshold = 3;
    static double x_threshold = -20.0, y_threshold = 0.5, z_threshold = 0.6, x_range_threshold = 0.5;
    bool isDetected = false;
    bool past_dist_flg = false;
    static double distance = std::numeric_limits<double>::min();
    static double past_distance = -100;
    pcl::PointXYZ AMB_point;

    pcl::fromROSMsg(*in_sensor_cloud, *in_sensor_cloud_ptr);

    for(unsigned int i = 0; i < in_sensor_cloud_ptr->points.size(); i++){            
        pcl::PointXYZ& temp_point = in_sensor_cloud_ptr->points[i];            
        temp_point.x += 1.2;
        temp_point.z += 2.0;
        if(temp_point.x <= 0 && temp_point.x >= x_threshold &&
            temp_point.y <= y_threshold && temp_point.y >= -1 * y_threshold &&
            temp_point.z >= z_threshold){
            filtered_point_cloud_ptr->points.push_back(temp_point);
            
        }
    }

    std::sort(filtered_point_cloud_ptr->points.begin(), filtered_point_cloud_ptr->points.end(), AMB_x_compare);

    for(int i = 0; i < (filtered_point_cloud_ptr->points.size() * 0.3) - 5; i++){
        int point_cnt = 0;
        double temp_distance = filtered_point_cloud_ptr->points[i].x;
        for(int j=i+1; j<i+6; j++){
            if(abs(temp_distance - filtered_point_cloud_ptr->points[j].x) >= x_range_threshold ) {
                break;
            }
            else{
                point_cnt++;
            }
        }
        if(point_cnt >= 5) {
            isDetected = true;
            distance = temp_distance;
            AMB_point = filtered_point_cloud_ptr->points[i];
        }
    }

    autoware_msgs::DetectedObjectArray AMB_object_ary;
    autoware_msgs::DetectedObject AMB_object;
    AMB_object_ary.header = in_sensor_cloud->header;
    AMB_object_ary.header.frame_id = base_id_;
    AMB_object.header = AMB_object_ary.header;
    AMB_object.label = "AMB";

    jsk_recognition_msgs::BoundingBoxArray AMB_box_ary;
    jsk_recognition_msgs::BoundingBox AMB_box;
    AMB_box_ary.header = AMB_object_ary.header;
    AMB_box.header = AMB_object_ary.header;

    if(isDetected && distance >= past_distance - 0.15){
        past_distance = distance;
        frame_cnt++;

        if(frame_cnt >= frame_threshold 
            && ( distance <= past_distance + 5 )) {
            //ROS_INFO("AMB Detected! : %lf", distance*-1);

            AMB_object.pose.position.x = distance;

            AMB_box.dimensions.x = car_depth_;
            AMB_box.dimensions.y = car_width_;
            AMB_box.dimensions.z = car_height_;
            AMB_box.pose.position.x = distance - car_depth_/2;
            AMB_box.pose.position.y = AMB_point.y;
            AMB_box.pose.position.z = AMB_point.z;

            AMB_object_ary.objects.push_back(AMB_object);
            AMB_box_ary.boxes.push_back(AMB_box);
        }
    }
    else{
        frame_cnt = 0;
        past_distance = -100;
    }
    
    object_manager_msgs::combined amb_msg;
    amb_msg.header = AMB_object.header;                
    

    amb_msg.obj_ary = AMB_object_ary;
    amb_msg.bb_ary = AMB_box_ary;
    
    //publisher_AMB_object_.publish(AMB_object_ary);


    sensor_msgs::PointCloud2 AMB_point_cloud;
    
    
    pcl::toROSMsg(*filtered_point_cloud_ptr, AMB_point_cloud);
    AMB_point_cloud.header = in_sensor_cloud->header;
    AMB_point_cloud.header.frame_id = base_id_;

    publisher_AMB_test_points_.publish(AMB_point_cloud); 


    return amb_msg;
}
// End

void RosRangeVisionFusionApp::LidarDetectionsCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud){
    if (!processing_ && !in_sensor_cloud->fields.empty())
    {
        processing_ = true;
        cloud_detections_ = in_sensor_cloud;
        SyncedDetectionsCallback(left_vision_detections_, right_vision_detections_, in_sensor_cloud);
        processing_ = false;
    }
}

void
RosRangeVisionFusionApp::SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &left_in_vision_detections,
                                                  const autoware_msgs::DetectedObjectArray::ConstPtr &right_in_vision_detections,
                                                  const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
    //ROS_WARN("[[[[[1");

    autoware_msgs::DetectedObjectArray fusion_objects;
    jsk_recognition_msgs::BoundingBoxArray fused_boxes;
    visualization_msgs::MarkerArray fused_objects_labels;

    fused_boxes.boxes.clear();
    fusion_objects.objects.clear();
    fused_objects_labels.markers.clear();

    if (empty_frames_ > 5)
    {
        ROS_INFO("[%s] Empty Detections. Make sure the vision detectors are running.", __APP_NAME__);
    }

    
    if (nullptr == left_in_vision_detections && nullptr == right_in_vision_detections)
    {
        //publisher_fused_boxes_.publish(fused_boxes);
        publisher_fused_objects_.publish(left_in_vision_detections);
        empty_frames_++;
        return;
    }

    //ROS_WARN("[[[[[2");

    if (!left_camera_lidar_tf_ok_)    
        left_camera_lidar_tf_ = FindTransform(left_image_frame_id_,in_sensor_cloud->header.frame_id);
    if (!right_camera_lidar_tf_ok_)
        right_camera_lidar_tf_ = FindTransform(right_image_frame_id_,in_sensor_cloud->header.frame_id);
    if (!map_lidar_tf_ok_)
        map_lidar_tf_ = FindTransform(map_name_, in_sensor_cloud->header.frame_id);
        
    
    if(
        !left_camera_lidar_tf_ok_ ||
        !left_camera_info_ok_ ||
        !right_camera_lidar_tf_ok_ ||
        !right_camera_info_ok_
        )
    {
        if(!left_camera_lidar_tf_ok_) ROS_ERROR("No left camera lidar tf!");
        if(!left_camera_info_ok_) ROS_ERROR("No left camera info!");
        if(!right_camera_lidar_tf_ok_) ROS_ERROR("No right camera lidar tf!");
        if(!right_camera_info_ok_) ROS_ERROR("No left camera info!");


        ROS_ERROR("[%s] Missing Camera-LiDAR TF or CameraInfo", __APP_NAME__);
        return;
    }

    //ROS_WARN("[[[[[3");

    lidar_base_tf_ = FindTransform(base_id_,
                                         in_sensor_cloud->header.frame_id);
    base_lidar_tf_ = FindTransform(in_sensor_cloud->header.frame_id, base_id_);

    fusion_objects = FuseRangeVisionDetections(left_in_vision_detections, right_in_vision_detections, in_sensor_cloud);
    fused_boxes = ObjectsToBoxes(fusion_objects);
    fused_objects_labels = ObjectsToMarkers(fusion_objects);

    //ROS_WARN("[[[[[4");

    object_manager_msgs::combined fused_object_msg;

    fused_object_msg.header = fusion_objects.header;
    fused_object_msg.obj_ary = fusion_objects;
    fused_object_msg.bb_ary = fused_boxes;

    publisher_fused_objects_.publish(fused_object_msg);
    debug_box_pub_.publish(fused_boxes);
    publisher_fused_text_.publish(fused_objects_labels);
    boxes_frame_ = fused_boxes.header.frame_id;
    empty_frames_ = 0;    
    

    object_manager_msgs::combined amb_msg;
    amb_msg = AMB_parse(in_sensor_cloud);
    publisher_AMB_object_.publish(amb_msg);

    left_vision_detections_ = nullptr;
    right_vision_detections_ = nullptr;

    //ROS_WARN("[[[[[5");
}

autoware_msgs::DetectedObjectArray
RosRangeVisionFusionApp::FuseRangeVisionDetections(const autoware_msgs::DetectedObjectArray::ConstPtr &left_in_vision_detections,
                                                   const autoware_msgs::DetectedObjectArray::ConstPtr &right_in_vision_detections,
                                                   const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{   

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_filtered_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_filtered_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // debug
    pcl::PointCloud<pcl::PointXYZ> image_point;
    pcl::PointCloud<pcl::PointXYZ> parsed_point;
    visualization_msgs::MarkerArray test_mark_ary;
    int marker_id = 0;
    double point_x_max = 25.0;

    // Filter points out of image area
    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
    for(unsigned int i = 0; i < current_sensor_cloud_ptr->points.size(); i++){
        geometry_msgs::Point left_temp_point, right_temp_point;
        if(current_sensor_cloud_ptr->points[i].x >= point_x_max) continue;
        
        left_temp_point.x = current_sensor_cloud_ptr->points[i].x;
        left_temp_point.y = current_sensor_cloud_ptr->points[i].y;
        left_temp_point.z = current_sensor_cloud_ptr->points[i].z;
        
        right_temp_point.x = current_sensor_cloud_ptr->points[i].x;
        right_temp_point.y = current_sensor_cloud_ptr->points[i].y;
        right_temp_point.z = current_sensor_cloud_ptr->points[i].z;
        

        cv::Point3f left_image_space_point = TransformPoint(left_temp_point, left_camera_lidar_tf_);
        cv::Point2i left_image_pixel = ProjectPoint(left_image_space_point, left_camera_name_);
        cv::Point3f right_image_space_point = TransformPoint(right_temp_point, right_camera_lidar_tf_);
        cv::Point2i right_image_pixel = ProjectPoint(right_image_space_point, right_camera_name_);

        // filtering points in image area
        if( (left_image_pixel.x >= 0) && (left_image_pixel.x < left_image_size_.width)
           && (left_image_pixel.y >= 0) && (left_image_pixel.y < left_image_size_.height) && (left_image_space_point.z > 0) ){
            left_filtered_sensor_cloud_ptr->points.push_back(current_sensor_cloud_ptr->points[i]);
            // test
            image_point.points.push_back(current_sensor_cloud_ptr->points[i]);
        }

        if( (right_image_pixel.x >= 0) && (right_image_pixel.x < right_image_size_.width)
           && (right_image_pixel.y >= 0) && (right_image_pixel.y < right_image_size_.height) && (right_image_space_point.z > 0) ){
            right_filtered_sensor_cloud_ptr->points.push_back(current_sensor_cloud_ptr->points[i]);
            
            // test
            image_point.points.push_back(current_sensor_cloud_ptr->points[i]);
        }
    }

    // Initialize Vectors for PCL
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > left_vision_matched_cloud_ptr_vec;
    for (size_t i = 0; i < left_in_vision_detections->objects.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        left_vision_matched_cloud_ptr_vec.push_back(temp_ptr);
    }
    
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > right_vision_matched_cloud_ptr_vec;
    for (size_t i = 0; i < right_in_vision_detections->objects.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        right_vision_matched_cloud_ptr_vec.push_back(temp_ptr);
    }

    // TEST For Publishing Points
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > left_filtered_vision_matched_cloud_ptr_vec, right_filtered_vision_matched_cloud_ptr_vec;
    
    int left_test_filtered_idx = left_in_vision_detections->objects.size();
    for (size_t i = 0; i <= left_test_filtered_idx; i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        left_filtered_vision_matched_cloud_ptr_vec.push_back(temp_ptr);
    }

    int right_test_filtered_idx = right_in_vision_detections->objects.size();
    for (size_t i = 0; i <= right_test_filtered_idx; i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        right_filtered_vision_matched_cloud_ptr_vec.push_back(temp_ptr);
    }

    autoware_msgs::DetectedObjectArray fused_objects;
    fused_objects.header = in_sensor_cloud->header;

    // For Left Camera Objects
    for (size_t i = 0; i < left_in_vision_detections->objects.size(); i++)
    {
        auto vision_object = left_in_vision_detections->objects[i];
        if( (vision_object.label != "car") && (vision_object.label != "person") ) {
            //ROS_INFO("Not Car or Person // current label : %s // i : %d", vision_object.label.c_str(), (int)i);
            continue;
        }

        cv::Rect vision_rect(vision_object.x, vision_object.y,
                             vision_object.width, vision_object.height);
        
        float x_min = std::numeric_limits<float>::max();

        // Points in bounding box
        for(unsigned int j=0; j < left_filtered_sensor_cloud_ptr->points.size(); j++){
            geometry_msgs::Point temp_point;
            temp_point.x = left_filtered_sensor_cloud_ptr->points[j].x;
            temp_point.y = left_filtered_sensor_cloud_ptr->points[j].y;
            temp_point.z = left_filtered_sensor_cloud_ptr->points[j].z;            
            cv::Point3f image_space_point = TransformPoint(temp_point, left_camera_lidar_tf_);
            cv::Point2i image_pixel = ProjectPoint(image_space_point, left_camera_name_);

            if( (image_pixel.x > vision_rect.x) && (image_pixel.x < vision_rect.x + vision_rect.width) &&
                    (image_pixel.y > vision_rect.y) && (image_pixel.y < vision_rect.y + vision_rect.height)){                
                
                left_vision_matched_cloud_ptr_vec[i]->points.push_back(left_filtered_sensor_cloud_ptr->points[j]);                
            }
        }

        // Set Dimension
        geometry_msgs::Vector3 dimension_;

        if(vision_object.label == "car"){
            dimension_.x = car_depth_;
            dimension_.y = car_width_;
            dimension_.z = car_height_;
        }
        else if(vision_object.label == "person"){
            dimension_.x = person_depth_;
            dimension_.y = person_width_;
            dimension_.z = person_height_;
        }
        else if(vision_object.label == "truck" || vision_object.label == "bus"){
            dimension_.x = truck_depth_;
            dimension_.y = truck_width_;
            dimension_.z = truck_height_;
        }

        // Get Object Distance
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::max();

        // Find distance to object
        bool distance_check = false;
        std::vector<pcl::PointXYZ> x_vec;
        for(unsigned int k=0; k<left_vision_matched_cloud_ptr_vec[i]->points.size(); k++){
            x_vec.push_back(left_vision_matched_cloud_ptr_vec[i]->points[k]);            
        }
        std::sort(x_vec.begin(), x_vec.end(), point_x_compare);        


        for(auto it = x_vec.begin(); it != x_vec.end(); ++it){            
            distance_check = x_distance_check((*it), 
                                              left_vision_matched_cloud_ptr_vec[i],
                                              dimension_);            
            if(distance_check) {
                min_x = (*it).x;
                break;
            }
        }

        if(!distance_check) continue; // If clouds looks not corret, don't make detected_objects
   
        float object_distance = min_x;
        if(object_distance > 150) object_distance = 0;

        // ROS_INFO("object_distance : %f", object_distance);
        
        // Create Filtered Point Cloud
        float filter_threshold = dimension_.x;
        float z_threshold = 0.1 - TF_Z;
        pcl::PointCloud<pcl::PointXYZ> temp_poly_point_cloud;

        geometry_msgs::Vector3 boundary;
        boundary.x = 0.5;
        boundary.y = 0.5;
        boundary.z = 0.5;
        
        for(unsigned int j=0; j < left_vision_matched_cloud_ptr_vec[i]->points.size(); j++){
            bool isInBoundary = count_points_in_boundary(left_vision_matched_cloud_ptr_vec[i]->points[j], 
                                                        left_vision_matched_cloud_ptr_vec[i],
                                                        boundary,
                                                        30
                                                        );
            if(!isInBoundary) continue;

            float point_x, point_y, point_z;
            point_x = left_vision_matched_cloud_ptr_vec[i]->points[j].x;
            point_y = left_vision_matched_cloud_ptr_vec[i]->points[j].y;
            point_z = left_vision_matched_cloud_ptr_vec[i]->points[j].z;
            float distance_to_point = sqrt(point_x*point_x - point_y*point_y);


            if( (std::abs(distance_to_point - object_distance) > filter_threshold) ) continue;            
            if(point_z < z_threshold) continue;

            left_filtered_vision_matched_cloud_ptr_vec[i]->points.push_back(left_vision_matched_cloud_ptr_vec[i]->points[j]);
            // test            
            parsed_point.points.push_back(left_vision_matched_cloud_ptr_vec[i]->points[j]);
            temp_poly_point_cloud.push_back(left_vision_matched_cloud_ptr_vec[i]->points[j]);
        }

        sensor_msgs::PointCloud2 cluster_cloud_;
        // ROS_INFO("filter size : %d", (int)left_filtered_vision_matched_cloud_ptr_vec[i]->points.size());

        pcl::toROSMsg(*left_filtered_vision_matched_cloud_ptr_vec[i], cluster_cloud_);
        cluster_cloud_.header = in_sensor_cloud->header;        

        // Set Pose
        for(unsigned int k=0; k<left_filtered_vision_matched_cloud_ptr_vec[i]->points.size(); k++){
            float point_y = left_filtered_vision_matched_cloud_ptr_vec[i]->points[k].y;
            float point_z = left_filtered_vision_matched_cloud_ptr_vec[i]->points[k].z;

            min_y = (min_y > point_y) ? point_y : min_y;
            min_z = (min_z > point_z) ? point_z : min_z;
            max_z = (max_z < point_z) ? point_z : max_z;
        }

        geometry_msgs::Pose pose_;
        pose_.position.x = object_distance + dimension_.x / 2;
        pose_.position.y = min_y + dimension_.y / 2;
        pose_.position.z = min_z + dimension_.z / 2;
        
        pose_.orientation.x = -1 * lidar_base_tf_.getRotation().x();
        pose_.orientation.y = -1 * lidar_base_tf_.getRotation().y();
        pose_.orientation.z = -1 * lidar_base_tf_.getRotation().z();
        pose_.orientation.w = lidar_base_tf_.getRotation().w();
       

        autoware_msgs::DetectedObject detected_object;
        detected_object.id = vision_object.id;
        detected_object.score = vision_object.score;
        detected_object.label = vision_object.label;
        detected_object.color = vision_object.color;
        detected_object.image_frame = vision_object.image_frame;
        detected_object.x = vision_object.x;
        detected_object.y = vision_object.y;
        detected_object.width = vision_object.width;
        detected_object.height = vision_object.height;
        detected_object.angle = vision_object.angle;
        //////////////////////////////////////////////////

        // detected_object.space_frame = in_sensor_cloud->header.frame_id; // OK
        detected_object.space_frame = "";
        detected_object.pose = pose_; // ok
        detected_object.dimensions = dimension_; // OK
        detected_object.pointcloud = cluster_cloud_; // OK
        //detected_object.convex_hull = polygon_; // OK
        detected_object.color.r = 20*i;
        detected_object.color.g = 20*i;
        detected_object.color.b = 20*i;
        detected_object.color.a = 1.;
        
        // Set Polygon
        std::vector<geometry_msgs::Point32> poly_point;
        get_poly_point_from_pcl(temp_poly_point_cloud, poly_point);
        geometry_msgs::Polygon polygon;
        for(auto it = poly_point.begin(); it != poly_point.end(); ++it){
            geometry_msgs::PointStamped lidar_poly_point;
            lidar_poly_point.header = in_sensor_cloud->header;
            lidar_poly_point.point.x = it->x;
            lidar_poly_point.point.y = it->y;
            lidar_poly_point.point.z = it->z;

            geometry_msgs::PointStamped map_poly_point;
            if(map_lidar_tf_ok_){                
                try
                {
                    transform_listener_->transformPoint(map_name_, lidar_poly_point, map_poly_point);
                    geometry_msgs::Point32 result_map_point;
                    result_map_point.x = map_poly_point.point.x;
                    result_map_point.y = map_poly_point.point.y;
                    result_map_point.z = 1.12;
                    polygon.points.push_back(result_map_point);

                }
                catch (tf::TransformException &ex)
                {
                    ROS_INFO("transform point fail!");
                }    
                
            }
            else polygon.points.push_back(*it);
        }
        detected_object.convex_hull.header = in_sensor_cloud->header;
        if(map_lidar_tf_ok_) detected_object.convex_hull.header.frame_id = map_name_;
        detected_object.convex_hull.polygon = polygon;

        // test
        visualization_msgs::Marker test_marker;
        polygon_to_marker(detected_object.convex_hull, test_marker, marker_id);
        test_mark_ary.markers.push_back(test_marker);
        marker_id++;

        fused_objects.objects.push_back(detected_object);
    }

    // For Right Camera Objects
    for (size_t i = 0; i < right_in_vision_detections->objects.size(); i++)
    {
        auto vision_object = right_in_vision_detections->objects[i];
        if( (vision_object.label != "car") && (vision_object.label != "person") ) {
            //ROS_INFO("Not Car or Person // current label : %s // i : %d", vision_object.label.c_str(), (int)i);
            continue;
        }

        cv::Rect vision_rect(vision_object.x, vision_object.y,
                             vision_object.width, vision_object.height);
        
        float x_min = std::numeric_limits<float>::max();

        // Points in bounding box
        for(unsigned int j=0; j < right_filtered_sensor_cloud_ptr->points.size(); j++){
            geometry_msgs::Point temp_point;
            temp_point.x = right_filtered_sensor_cloud_ptr->points[j].x;
            temp_point.y = right_filtered_sensor_cloud_ptr->points[j].y;
            temp_point.z = right_filtered_sensor_cloud_ptr->points[j].z;            
            cv::Point3f image_space_point = TransformPoint(temp_point, right_camera_lidar_tf_);
            cv::Point2i image_pixel = ProjectPoint(image_space_point, right_camera_name_);

            if( (image_pixel.x > vision_rect.x) && (image_pixel.x < vision_rect.x + vision_rect.width) &&
                    (image_pixel.y > vision_rect.y) && (image_pixel.y < vision_rect.y + vision_rect.height)){                
                
                right_vision_matched_cloud_ptr_vec[i]->points.push_back(right_filtered_sensor_cloud_ptr->points[j]);
            }
        }

        // Set Dimension
        geometry_msgs::Vector3 dimension_;

        if(vision_object.label == "car"){
            dimension_.x = car_depth_;
            dimension_.y = car_width_;
            dimension_.z = car_height_;
        }
        else if(vision_object.label == "person"){
            dimension_.x = person_depth_;
            dimension_.y = person_width_;
            dimension_.z = person_height_;
        }
        else if(vision_object.label == "truck" || vision_object.label == "bus"){
            dimension_.x = truck_depth_;
            dimension_.y = truck_width_;
            dimension_.z = truck_height_;
        }

        // Get Object Distance
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::max();

        bool distance_check = false;
        std::vector<pcl::PointXYZ> x_vec;
        for(unsigned int k=0; k<right_vision_matched_cloud_ptr_vec[i]->points.size(); k++){
            x_vec.push_back(right_vision_matched_cloud_ptr_vec[i]->points[k]);            
        }
        std::sort(x_vec.begin(), x_vec.end(), point_x_compare);

        for(auto it = x_vec.begin(); it != x_vec.end(); ++it){            
            distance_check = x_distance_check((*it), 
                                              right_vision_matched_cloud_ptr_vec[i],
                                              dimension_);            
            if(distance_check) {
                min_x = (*it).x;
                break;
            }
        }

        if(!distance_check) continue; // If clouds looks not corret, don't make detected_objects

        float object_distance = min_x;
        if(object_distance > 150) object_distance = 0;

        // ROS_INFO("object_distance : %f", object_distance);
        
        // Create Filtered Point Cloud
        float filter_threshold = dimension_.x;
        float z_threshold = 0.1 - TF_Z;
        pcl::PointCloud<pcl::PointXYZ> temp_poly_point_cloud;

        geometry_msgs::Vector3 boundary;
        boundary.x = 0.5;
        boundary.y = 0.5;
        boundary.z = 0.5;

        for(unsigned int j=0; j < right_vision_matched_cloud_ptr_vec[i]->points.size(); j++){
            bool isInBoundary = count_points_in_boundary(right_vision_matched_cloud_ptr_vec[i]->points[j], 
                                                        right_vision_matched_cloud_ptr_vec[i],
                                                        boundary,
                                                        30
                                                        );
            if(!isInBoundary) continue;

            float point_x, point_z;
            point_x = right_vision_matched_cloud_ptr_vec[i]->points[j].x;
            point_z = right_vision_matched_cloud_ptr_vec[i]->points[j].z;

            if( (std::abs(point_x - object_distance) > filter_threshold) ) continue;            
            if(point_z < z_threshold) continue;
            
            right_filtered_vision_matched_cloud_ptr_vec[i]->points.push_back(right_vision_matched_cloud_ptr_vec[i]->points[j]);
            // test            
            parsed_point.points.push_back(right_vision_matched_cloud_ptr_vec[i]->points[j]);            
            temp_poly_point_cloud.push_back(right_vision_matched_cloud_ptr_vec[i]->points[j]);
        }

        sensor_msgs::PointCloud2 cluster_cloud_;

        pcl::toROSMsg(*right_filtered_vision_matched_cloud_ptr_vec[i], cluster_cloud_);
        cluster_cloud_.header = in_sensor_cloud->header;        

        // Set Pose
        for(unsigned int k=0; k<right_filtered_vision_matched_cloud_ptr_vec[i]->points.size(); k++){
            float point_y = right_filtered_vision_matched_cloud_ptr_vec[i]->points[k].y;
            float point_z = right_filtered_vision_matched_cloud_ptr_vec[i]->points[k].z;

            min_y = (min_y > point_y) ? point_y : min_y;
            min_z = (min_z > point_z) ? point_z : min_z;
            max_z = (max_z < point_z) ? point_z : max_z;
        }

        geometry_msgs::Pose pose_;
        pose_.position.x = object_distance + dimension_.x / 2;
        pose_.position.y = min_y + dimension_.y / 2;
        pose_.position.z = min_z + dimension_.z / 2;
        
        pose_.orientation.x = -1 * lidar_base_tf_.getRotation().x();
        pose_.orientation.y = -1 * lidar_base_tf_.getRotation().y();
        pose_.orientation.z = -1 * lidar_base_tf_.getRotation().z();
        pose_.orientation.w = lidar_base_tf_.getRotation().w();

        autoware_msgs::DetectedObject detected_object;
        detected_object.id = vision_object.id;
        detected_object.score = vision_object.score;
        detected_object.label = vision_object.label;
        detected_object.color = vision_object.color;
        detected_object.image_frame = vision_object.image_frame;
        detected_object.x = vision_object.x;
        detected_object.y = vision_object.y;
        detected_object.width = vision_object.width;
        detected_object.height = vision_object.height;
        detected_object.angle = vision_object.angle;
        //////////////////////////////////////////////////

        detected_object.space_frame = in_sensor_cloud->header.frame_id; // OK
        detected_object.space_frame = "";
        detected_object.pose = pose_; // ok
        detected_object.dimensions = dimension_; // OK
        detected_object.pointcloud = cluster_cloud_; // OK
        //detected_object.convex_hull = polygon_; // OK
        detected_object.color.r = 20*i;
        detected_object.color.g = 20*i;
        detected_object.color.b = 20*i;
        detected_object.color.a = 1.;

        // Set Polygon
        std::vector<geometry_msgs::Point32> poly_point;
        get_poly_point_from_pcl(temp_poly_point_cloud, poly_point);
        geometry_msgs::Polygon polygon;
        for(auto it = poly_point.begin(); it != poly_point.end(); ++it){
            geometry_msgs::PointStamped lidar_poly_point;
            lidar_poly_point.header = in_sensor_cloud->header;
            lidar_poly_point.point.x = it->x;
            lidar_poly_point.point.y = it->y;
            lidar_poly_point.point.z = it->z;

            geometry_msgs::PointStamped map_poly_point;
            if(map_lidar_tf_ok_){                
                try
                {
                    transform_listener_->transformPoint(map_name_, lidar_poly_point, map_poly_point);
                    geometry_msgs::Point32 result_map_point;
                    result_map_point.x = map_poly_point.point.x;
                    result_map_point.y = map_poly_point.point.y;
                    result_map_point.z = 1.12;
                    polygon.points.push_back(result_map_point); 
                }
                catch (tf::TransformException &ex)
                {
                    ROS_INFO("transform point fail!");
                }   
                
            }
            else polygon.points.push_back(*it);
        }
        detected_object.convex_hull.header = in_sensor_cloud->header;
        if(map_lidar_tf_ok_) detected_object.convex_hull.header.frame_id = map_name_;
        detected_object.convex_hull.polygon = polygon;

        //testx_distance_check
        visualization_msgs::Marker test_marker;
        polygon_to_marker(detected_object.convex_hull, test_marker, marker_id);
        test_mark_ary.markers.push_back(test_marker);
        marker_id++;

        fused_objects.objects.push_back(detected_object);
    }

    bool personFlg = false;
    for(auto it = fused_objects.objects.begin(); it != fused_objects.objects.end(); ++it){
        if(it->label == "person") {
            personFlg = true;
            break;
        }
    }

    if(!personFlg) {
        autoware_msgs::DetectedObject fake_person;
        fake_person.label = "person";
        fake_person.space_frame = in_sensor_cloud->header.frame_id; // OK
        fake_person.space_frame = "";
        fake_person.pose.position.x = 1000;
        fake_person.pose.position.y = 1000;
        fake_person.pose.position.z = 1000;

        fused_objects.objects.push_back(fake_person);
    }

    bool carFlg = false;
    for(auto it = fused_objects.objects.begin(); it != fused_objects.objects.end(); ++it){
        if(it->label == "car") {
            carFlg = true;
            break;
        }
    }

    if(!carFlg) {
        autoware_msgs::DetectedObject fake_car;
        fake_car.label = "car";
        fake_car.space_frame = in_sensor_cloud->header.frame_id; // OK
        fake_car.space_frame = "";
        fake_car.pose.position.x = 1000;
        fake_car.pose.position.y = 1000;
        fake_car.pose.position.z = 1000;

        fused_objects.objects.push_back(fake_car);
    }



    sensor_msgs::PointCloud2 obj_test_msg, img_test_msg;
    toROSMsg(parsed_point, obj_test_msg);
    toROSMsg(image_point, img_test_msg);
    obj_test_msg.header = in_sensor_cloud->header;
    img_test_msg.header = in_sensor_cloud->header;

    publisher_img_test_points_.publish(img_test_msg);
    publisher_obj_test_points_.publish(obj_test_msg);
    publisher_obj_test_marker_.publish(test_mark_ary);

    return fused_objects;
}

bool RosRangeVisionFusionApp::x_distance_check(pcl::PointXYZ target_point, pcl::PointCloud<pcl::PointXYZ>::Ptr points, 
                                            geometry_msgs::Vector3 dimension)
{
    float x_min = target_point.x; 
    float x_max = target_point.x + (dimension.x)/2;
    float y_min = target_point.y - (dimension.y)/2;
    float y_max = target_point.y + (dimension.y)/2;
    
    
    int count = 0;
    for(unsigned int i=0; i<points->points.size(); i++){
        float x = points->points[i].x, y = points->points[i].y, z = points->points[i].z;
        if( ( x >= x_min ) && ( x <= x_max ) &&
            ( y >= y_min ) && ( y <= y_max ) ){
                count++;
            }                
    }

    if(count >= x_distance_filter_param_) {
        // ROS_INFO("[TRUE] count : %d // x_distance : %f", count, target_point.x);
        return true;
    }    
    else {
        // ROS_INFO("[FALSE] count : %d // x_distance : %f", count, target_point.x);
        return false;
    }
}

bool RosRangeVisionFusionApp::point_x_compare(pcl::PointXYZ& a, pcl::PointXYZ& b){
    return a.x < b.x;
}

bool RosRangeVisionFusionApp::AMB_x_compare(pcl::PointXYZ& a, pcl::PointXYZ& b){
    return a.x > b.x;
}

void RosRangeVisionFusionApp::get_poly_point_from_pcl(pcl::PointCloud<pcl::PointXYZ> point_cloud, std::vector<geometry_msgs::Point32>& poly_point){
    // pcl::PointCloud<pcl::PointXYZ> point_cloud;
    // pcl::fromROSMsg(pcl_msg, point_cloud);

    geometry_msgs::Point32 center;
    float x_sum = 0, y_sum = 0;

    for(auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it){
        x_sum += it->x;
        y_sum += it->y;
    }
    center.x = x_sum / point_cloud.points.size();
    center.y = y_sum / point_cloud.points.size();

    // Find the section of each point

    std::vector<std::vector<geometry_msgs::Point32>> section;
    section.resize(8);

    for(auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it){
       geometry_msgs::Point32 cur_point;
       geometry_msgs::Point32 grad_1_point, grad_minus_1_point;

       cur_point.x = it->x;
       cur_point.y = it->y;
       cur_point.z = 1.12;
       grad_1_point.x = cur_point.x;
       grad_1_point.y = (cur_point.x) - center.x + center.y;
       grad_minus_1_point.x = cur_point.x;
       grad_minus_1_point.y = (cur_point.x) + center.x + center.y;

       if(cur_point.y >= center.y && cur_point.x > center.x){
           if(cur_point.y < grad_1_point.y) section[0].push_back(cur_point);
           else if(cur_point.y >= grad_1_point.y) section[1].push_back(cur_point);
       }
       else if(cur_point.y > center.y && cur_point.x <= center.x){
           if(cur_point.y > grad_minus_1_point.y) section[2].push_back(cur_point);
           else if(cur_point.y <= grad_minus_1_point.y) section[3].push_back(cur_point);
       }
       else if(cur_point.y <= center.y && cur_point.x < center.x){
           if(cur_point.y > grad_1_point.y) section[4].push_back(cur_point);
           else if (cur_point.y <= grad_1_point.y) section[5].push_back(cur_point);
       }
       else if(cur_point.y < center.y && cur_point.x >= center.x){
           if(cur_point.y < grad_minus_1_point.y) section[6].push_back(cur_point);
           else if(cur_point.y >= grad_minus_1_point.y) section[7].push_back(cur_point);
       }
       else{
           std::cout<<"  Point value is uneaccepatable!"<<std::endl;
       }
    }

    // Find the most far point from center point

    for(int i = 0; i < 8; ++i){
        if(section[i].size() == 0) continue;
        std::vector<geometry_msgs::Point32>& points_in_section = (section[i]);
        float max_dist = -1;
        geometry_msgs::Point32 dist_max_point;
        bool isChanged = false;
        dist_max_point.x = -9999;
        dist_max_point.y = -9999;
        dist_max_point.z = -9999;

        //std::cout<<"Section size : "<<points_in_section.size()<<std::endl;
        if(points_in_section.empty()) continue;


        // ROS_ERROR("DEBUG1");
        for(auto it2 = points_in_section.begin(); it2 != points_in_section.end(); ++it2){
            
            geometry_msgs::Point32 point = (*it2);
            float dist = std::sqrt(std::pow(point.x - center.x, 2) + std::pow(point.y - center.y, 2));
            // if(dist > 4.0) ROS_ERROR("Dist : %lf", dist);

            if(dist - max_dist > 0 && fabs(dist) <= 1.5){
                max_dist = dist;
                dist_max_point = point;
                isChanged = true;
            }
        }        

        if(isChanged == false) continue;
        poly_point.push_back(dist_max_point);
    }

    // poly_point.push_back(center);
}

void RosRangeVisionFusionApp::polygon_to_marker(const geometry_msgs::PolygonStamped polygon, visualization_msgs::Marker& marker, int id){
    marker.header = polygon.header;
    marker.action=visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.2;
    marker.lifetime = ros::Duration(0.01);
    
    if(polygon.polygon.points.empty()) return;
    
    for(auto it = polygon.polygon.points.begin(); it != polygon.polygon.points.end(); ++it){
        geometry_msgs::Point temp_point;
        temp_point.x = it->x;
        temp_point.y = it->y;
        temp_point.z = it->z;
        marker.points.push_back(temp_point);
    }
    geometry_msgs::Point end_point;
    end_point.x = polygon.polygon.points[0].x;
    end_point.y = polygon.polygon.points[0].y;
    end_point.z = polygon.polygon.points[0].z;
    marker.points.push_back(end_point);
}


bool RosRangeVisionFusionApp::count_points_in_boundary(pcl::PointXYZ& target_point, pcl::PointCloud<pcl::PointXYZ>::Ptr points, geometry_msgs::Vector3 dimension, int threshold){
    float x_min = target_point.x; 
    float x_max = target_point.x + (dimension.x)/2;
    float y_min = target_point.y - (dimension.y)/2;
    float y_max = target_point.y + (dimension.y)/2;
    
    
    int count = 0;
    for(unsigned int i=0; i<points->points.size(); i++){
        float x = points->points[i].x, y = points->points[i].y, z = points->points[i].z;
        if( ( x >= x_min ) && ( x <= x_max ) &&
            ( y >= y_min ) && ( y <= y_max ) ){
                count++;
            }                
    }

    if(count < threshold) return false;
    else return true;
}