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

#include "range_vision_fusion/enhanced_range_vision_fusion.h"

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
RosRangeVisionFusionApp::ProjectPoint(const cv::Point3f &in_point)
{
    auto u = int(in_point.x * fx_ / in_point.z + cx_);
    auto v = int(in_point.y * fy_ / in_point.z + cy_);

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

bool
RosRangeVisionFusionApp::IsObjectInImage(const autoware_msgs::DetectedObject &in_detection)
{
    cv::Point3f image_space_point = TransformPoint(in_detection.pose.position, camera_lidar_tf_);

    cv::Point2i image_pixel = ProjectPoint(image_space_point);

    return (image_pixel.x >= 0)
           && (image_pixel.x < image_size_.width)
           && (image_pixel.y >= 0)
           && (image_pixel.y < image_size_.height)
           && (image_space_point.z > 0);
}

cv::Rect RosRangeVisionFusionApp::ProjectDetectionToRect(const autoware_msgs::DetectedObject &in_detection)
{
    cv::Rect projected_box;

    Eigen::Vector3f pos;
    pos << in_detection.pose.position.x,
            in_detection.pose.position.y,
            in_detection.pose.position.z;

    Eigen::Quaternionf rot(in_detection.pose.orientation.w,
                           in_detection.pose.orientation.x,
                           in_detection.pose.orientation.y,
                           in_detection.pose.orientation.z);

    std::vector<double> dims = {in_detection.dimensions.x,
                                  in_detection.dimensions.y,
                                  in_detection.dimensions.z};

    jsk_recognition_utils::Cube cube(pos, rot, dims);

    Eigen::Affine3f range_vision_tf;
    tf::transformTFToEigen(camera_lidar_tf_, range_vision_tf);
    jsk_recognition_utils::Vertices vertices = cube.transformVertices(range_vision_tf);

    std::vector<cv::Point> polygon;
    for (auto &vertex : vertices)
    {
        cv::Point p = ProjectPoint(cv::Point3f(vertex.x(), vertex.y(), vertex.z()));
        polygon.push_back(p);
    }

    projected_box = cv::boundingRect(polygon);

    return projected_box;
}

void
RosRangeVisionFusionApp::TransformRangeToVision(const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections,
                                                      autoware_msgs::DetectedObjectArray &out_in_cv_range_detections,
                                                      autoware_msgs::DetectedObjectArray &out_out_cv_range_detections)
{
    out_in_cv_range_detections.header = in_range_detections->header;
    out_in_cv_range_detections.objects.clear();
    out_out_cv_range_detections.header = in_range_detections->header;
    out_out_cv_range_detections.objects.clear();
    for (size_t i= 0; i < in_range_detections->objects.size(); i++)
    {
        if(IsObjectInImage(in_range_detections->objects[i]))
        {
            out_in_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
        }
        else
        {
            out_out_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
        }
    }
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

autoware_msgs::DetectedObjectArray
RosRangeVisionFusionApp::FuseRangeVisionDetections(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
                                                   const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{

    autoware_msgs::DetectedObjectArray range_in_cv;
    autoware_msgs::DetectedObjectArray range_out_cv;
    TransformRangeToVision(in_range_detections, range_in_cv, range_out_cv);

    autoware_msgs::DetectedObjectArray fused_objects;

    std::vector< std::vector<size_t> > vision_range_assignments (in_vision_detections->objects.size());
    std::vector<bool> used_vision_detections(in_vision_detections->objects.size(), false);
    std::vector< long > vision_range_closest (in_vision_detections->objects.size());

    for (size_t i = 0; i < in_vision_detections->objects.size(); i++)
    {
        auto vision_object = in_vision_detections->objects[i];

        cv::Rect vision_rect(vision_object.x, vision_object.y,
                             vision_object.width, vision_object.height);
        int vision_rect_area = vision_rect.area();
        long closest_index = -1;
        double closest_distance = std::numeric_limits<double>::max();

        for (size_t j = 0; j < range_in_cv.objects.size(); j++)
        {
            double current_distance = GetDistanceToObject(range_in_cv.objects[j]);

            cv::Rect range_rect = ProjectDetectionToRect(range_in_cv.objects[j]);
            int range_rect_area = range_rect.area();

            cv::Rect overlap = range_rect & vision_rect;
            if ( (overlap.area() > range_rect_area*overlap_threshold_)
                 || (overlap.area() > vision_rect_area*overlap_threshold_)
                    )
                    // Overlap이 되면 무조건 데이터를 갱신하는데... 최소 정도를 왜 파악하지 않을까?
                    // 어차피 비전에서 검출할 수 있는 정보는 항상 앞에 존재하기 때문?
                    // 내가 생각하기엔 오버랩 정도가 가장 크면서 거리가 가까워야 한다고 생각
            {
                vision_range_assignments[i].push_back(j);
                range_in_cv.objects[j].score = vision_object.score;
                range_in_cv.objects[j].label = vision_object.label;
                range_in_cv.objects[j].color = vision_object.color;
                range_in_cv.objects[j].image_frame = vision_object.image_frame;
                range_in_cv.objects[j].x = vision_object.x;
                range_in_cv.objects[j].y = vision_object.y;
                range_in_cv.objects[j].width = vision_object.width;
                range_in_cv.objects[j].height = vision_object.height;
                range_in_cv.objects[j].angle = vision_object.angle;
                range_in_cv.objects[j].id = vision_object.id;
                CheckMinimumDimensions(range_in_cv.objects[j]);
                if (vision_object.pose.orientation.x > 0
                    || vision_object.pose.orientation.y > 0
                    || vision_object.pose.orientation.z > 0)
                {
                    range_in_cv.objects[i].pose.orientation = vision_object.pose.orientation;
                }
                if(current_distance < closest_distance)
                {
                    closest_index = j;
                    closest_distance = current_distance;
                }
                used_vision_detections[i] = true;
            }//end if overlap
        }//end for range_in_cv
        vision_range_closest[i] = closest_index;
    }

    std::vector<bool> used_range_detections(range_in_cv.objects.size(), false);
    //only assign the closest
    for(size_t i = 0; i < vision_range_assignments.size(); i++)
    {
        if(!range_in_cv.objects.empty() && vision_range_closest[i] >= 0)
        {
            used_range_detections[i] = true;
            fused_objects.objects.push_back(range_in_cv.objects[vision_range_closest[i]]);
        }
    }
    for(size_t i = 0; i < used_vision_detections.size(); i++)
    {
        if (!used_vision_detections[i])
        {
            fused_objects.objects.push_back(in_vision_detections->objects[i]);
        }
    }
    
    return fused_objects;
}

void
RosRangeVisionFusionApp::SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
                                                       const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{
    autoware_msgs::DetectedObjectArray fusion_objects;
    jsk_recognition_msgs::BoundingBoxArray fused_boxes;
    visualization_msgs::MarkerArray fused_objects_labels;

    fused_boxes.boxes.clear();
    fusion_objects.objects.clear();
    fused_objects_labels.markers.clear();

    if (empty_frames_ > 5)
    {
        ROS_INFO("[%s] Empty Detections. Make sure the vision and range detectors are running.", __APP_NAME__);
    }

    if (nullptr == in_vision_detections
        && nullptr == in_range_detections)
    {
        empty_frames_++;
        return;
    }

    if (nullptr == in_vision_detections
        && nullptr != in_range_detections
        && !in_range_detections->objects.empty())
    {
        publisher_fused_boxes_.publish(fused_boxes);
        publisher_fused_objects_.publish(in_range_detections);
        empty_frames_++;
        return;
    }
    if (nullptr == in_range_detections
        && nullptr != in_vision_detections
        && !in_vision_detections->objects.empty())
    {
        publisher_fused_boxes_.publish(fused_boxes);
        publisher_fused_objects_.publish(in_vision_detections);
        empty_frames_++;
        return;
    }

    if (!camera_lidar_tf_ok_)
    {
        camera_lidar_tf_ = FindTransform(image_frame_id_,
                                         in_range_detections->header.frame_id);
    }
    if(
        !camera_lidar_tf_ok_ ||
        !camera_info_ok_)
    {
        ROS_INFO("[%s] Missing Camera-LiDAR TF or CameraInfo", __APP_NAME__);
        return;
    }

    fusion_objects = FuseRangeVisionDetections(in_vision_detections, in_range_detections);
    fused_boxes = ObjectsToBoxes(fusion_objects);
    fused_objects_labels = ObjectsToMarkers(fusion_objects);

    publisher_fused_objects_.publish(fusion_objects);
    publisher_fused_boxes_.publish(fused_boxes);
    publisher_fused_text_.publish(fused_objects_labels);
    boxes_frame_ = fused_boxes.header.frame_id;
    empty_frames_ = 0;

    vision_detections_ = nullptr;
    range_detections_ = nullptr;

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

        if (box.dimensions.x > 0 && box.dimensions.y > 0 && box.dimensions.z > 0)
        {
            final_boxes.boxes.push_back(box);
        }
    }
    return final_boxes;
}

void
RosRangeVisionFusionApp::VisionDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{

    if (!processing_ && !in_vision_detections->objects.empty())
    {
        processing_ = true;
        vision_detections_ = in_vision_detections;
        SyncedDetectionsCallback2(in_vision_detections, range_detections_, cloud_detections_);
        processing_ = false;
    }
}

void
RosRangeVisionFusionApp::RangeDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{
    if (!processing_ && !in_range_detections->objects.empty())
    {
        processing_ = true;
        range_detections_ = in_range_detections;
        SyncedDetectionsCallback2(vision_detections_, in_range_detections, cloud_detections_);
        processing_ = false;
    }
}


void RosRangeVisionFusionApp::ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
    if(!camera_info_ok_)
        return;
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
    cv::Mat in_image = cv_image->image;

    cv::Mat undistorted_image;
    cv::undistort(in_image, image_, camera_instrinsics_, distortion_coefficients_);
};

void
RosRangeVisionFusionApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
    image_size_.height = in_message.height;
    image_size_.width = in_message.width;

    camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
        }
    }

    distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
    for (int col = 0; col < 5; col++)
    {
        distortion_coefficients_.at<double>(col) = in_message.D[col];
    }

    fx_ = static_cast<float>(in_message.P[0]);
    fy_ = static_cast<float>(in_message.P[5]);
    cx_ = static_cast<float>(in_message.P[2]);
    cy_ = static_cast<float>(in_message.P[6]);

    intrinsics_subscriber_.shutdown();
    camera_info_ok_ = true;
    image_frame_id_ = in_message.header.frame_id;
    ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
    ROS_INFO("Projection Parameter : %f, %f, %f, %f", fx_, fy_, cx_, cy_);
}

tf::StampedTransform
RosRangeVisionFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
    tf::StampedTransform transform;

    ROS_INFO("%s - > %s", in_source_frame.c_str(), in_target_frame.c_str());

    if(in_target_frame == image_frame_id_){
        camera_lidar_tf_ok_ = false;
        try
        {
            transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
            camera_lidar_tf_ok_ = true;
            ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
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
            ROS_INFO("[%s] Lidar-Base TF obtained", __APP_NAME__);
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
    std::string camera_info_src, detected_objects_vision, min_car_dimensions, min_person_dimensions, min_truck_dimensions;
    std::string detected_objects_range, fused_topic_str = "/detection/combined_objects", fused_boxes_str = "/detection/combined_objects_boxes";
    std::string fused_text_str = "detection/combined_objects_labels";
    std::string name_space_str = ros::this_node::getNamespace();
    bool sync_topics = false;

    // Start
    std::string lidar_topic_str;
    // End

    ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Vision and Range Detections being published.", __APP_NAME__);
    in_private_handle.param<std::string>("detected_objects_range", detected_objects_range, "/detection/lidar_objects");
    ROS_INFO("[%s] detected_objects_range: %s", __APP_NAME__, detected_objects_range.c_str());

    in_private_handle.param<std::string>("detected_objects_vision", detected_objects_vision, "/detection/vision_objects");
    ROS_INFO("[%s] detected_objects_vision: %s", __APP_NAME__, detected_objects_vision.c_str());

    in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
    ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

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
        camera_info_src = name_space_str + camera_info_src;
    }

    //generate subscribers and sychronizers
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
    intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
                                                         1,
                                                         &RosRangeVisionFusionApp::IntrinsicsCallback, this);

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_vision.c_str());
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_range.c_str());
    if (!sync_topics)
    {
        detections_range_subscriber_ = in_private_handle.subscribe(detected_objects_vision,
                                                                   1,
                                                                   &RosRangeVisionFusionApp::VisionDetectionsCallback, this);

        detections_vision_subscriber_ = in_private_handle.subscribe(detected_objects_range,
                                                                    1,
                                                                    &RosRangeVisionFusionApp::RangeDetectionsCallback, this);

        // Start
        detections_lidar_subscriber_ = in_private_handle.subscribe(lidar_topic_str,
                                                                    1,
                                                                    &RosRangeVisionFusionApp::LidarDetectionsCallback, this);
        // End
    }
    else
    {
        vision_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                        detected_objects_vision, 1);
        range_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                        detected_objects_range, 1);
        // detections_synchronizer_ =
        //         new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
        //                                                        *vision_filter_subscriber_,
        //                                                        *range_filter_subscriber_,);
        // detections_synchronizer_->registerCallback(boost::bind(&RosRangeVisionFusionApp::SyncedDetectionsCallback, this, _1, _2));

        // Start
        lidar_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                                        lidar_topic_str, 1);
        detections_synchronizer_ =
                new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                               *vision_filter_subscriber_,
                                                               *range_filter_subscriber_, *lidar_filter_subscriber_);
        detections_synchronizer_->registerCallback(boost::bind(&RosRangeVisionFusionApp::SyncedDetectionsCallback2, this, _1, _2, _3));
        ROS_INFO("SYNCHED!");
        // End
    }

    publisher_fused_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(fused_topic_str, 1);
    publisher_fused_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>(fused_boxes_str, 1);
    publisher_fused_text_ = node_handle_.advertise<visualization_msgs::MarkerArray>(fused_text_str, 1);

    ROS_INFO("[%s] Publishing fused objects in %s", __APP_NAME__, fused_topic_str.c_str());
    ROS_INFO("[%s] Publishing fused boxes in %s", __APP_NAME__, fused_boxes_str.c_str());

    // Start
    points_raw_subscriber_ = in_private_handle.subscribe(lidar_topic_str, 1, &RosRangeVisionFusionApp::LidarCallback, this);
    publisher_test_points_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/test_points", 1);
    // End

}

void
RosRangeVisionFusionApp::Run()
{
    ros::NodeHandle private_node_handle("~");
    tf::TransformListener transform_listener;

    transform_listener_ = &transform_listener;

    InitializeRosIo(private_node_handle);

    ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

    ros::spin();

    ROS_INFO("[%s] END", __APP_NAME__);
}

RosRangeVisionFusionApp::RosRangeVisionFusionApp()
{
    camera_lidar_tf_ok_ = false;
    camera_info_ok_ = false;
    processing_ = false;
    image_frame_id_ = "";
    overlap_threshold_ = 0.5;
    empty_frames_ = 0;
}

// Start
void RosRangeVisionFusionApp::LidarCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud){
    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // //Make PointCloud2
    // pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

    // for(unsigned int i = 0; i < current_sensor_cloud_ptr->points.size(); i++){
    //     geometry_msgs::Point temp_point;
    //     temp_point.x = current_sensor_cloud_ptr->points[i].x;
    //     temp_point.y = current_sensor_cloud_ptr->points[i].y;
    //     temp_point.z = current_sensor_cloud_ptr->points[i].z;
    //     cv::Point3f image_space_point = TransformPoint(temp_point, camera_lidar_tf_);
    //     cv::Point2i image_pixel = ProjectPoint(image_space_point);

    //     if( (image_pixel.x >= 0) && (image_pixel.x < image_size_.width) // Projected point is not in the image area?
    //        && (image_pixel.y >= 0) && (image_pixel.y < image_size_.height) && (image_space_point.z > 0) ){
    //         filtered_sensor_cloud_ptr->points.push_back(current_sensor_cloud_ptr->points[i]);
    //     }
    // }


    // sensor_msgs::PointCloud2 filtered_point_cloud;
    
    
    // pcl::toROSMsg(*filtered_sensor_cloud_ptr, filtered_point_cloud);
    // filtered_point_cloud.header = in_sensor_cloud->header;


    // publisher_test_points_.publish(filtered_point_cloud);

}
// End

void RosRangeVisionFusionApp::LidarDetectionsCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud){
    if (!processing_ && !in_sensor_cloud->fields.empty())
    {
        processing_ = true;
        cloud_detections_ = in_sensor_cloud;
        SyncedDetectionsCallback2(vision_detections_, range_detections_, in_sensor_cloud);
        processing_ = false;
    }
}

void
RosRangeVisionFusionApp::SyncedDetectionsCallback2(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
                                                       const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections,
                                                       const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
    autoware_msgs::DetectedObjectArray fusion_objects;
    jsk_recognition_msgs::BoundingBoxArray fused_boxes;
    visualization_msgs::MarkerArray fused_objects_labels;

    fused_boxes.boxes.clear();
    fusion_objects.objects.clear();
    fused_objects_labels.markers.clear();

    if (empty_frames_ > 5)
    {
        ROS_INFO("[%s] Empty Detections. Make sure the vision and range detectors are running.", __APP_NAME__);
    }

    if (nullptr == in_vision_detections
        && nullptr == in_range_detections)
    {
        empty_frames_++;
        return;
    }

    if (nullptr == in_vision_detections
        && nullptr != in_range_detections
        && !in_range_detections->objects.empty())
    {
        publisher_fused_boxes_.publish(fused_boxes);
        publisher_fused_objects_.publish(in_range_detections);
        empty_frames_++;
        return;
    }

    if (nullptr == in_range_detections
        && nullptr != in_vision_detections
        && !in_vision_detections->objects.empty())
    {
        ROS_INFO("!!");
        // publisher_fused_boxes_.publish(fused_boxes);
        // publisher_fused_objects_.publish(in_vision_detections);
        empty_frames_++;
        return;
    }

    if (!camera_lidar_tf_ok_)
    {
        camera_lidar_tf_ = FindTransform(image_frame_id_,
                                         in_range_detections->header.frame_id);
    }
    if(
        !camera_lidar_tf_ok_ ||
        !camera_info_ok_)
    {
        ROS_INFO("[%s] Missing Camera-LiDAR TF or CameraInfo", __APP_NAME__);
        return;
    }

    // Start
    if (!lidar_base_tf_ok_)
    {
        lidar_base_tf_ = FindTransform(base_id_,
                                         in_range_detections->header.frame_id);
    }
    // End


    fusion_objects = FuseRangeVisionDetections2(in_vision_detections, in_range_detections, in_sensor_cloud);
    fused_boxes = ObjectsToBoxes(fusion_objects);
    fused_objects_labels = ObjectsToMarkers(fusion_objects);

    publisher_fused_objects_.publish(fusion_objects);
    publisher_fused_boxes_.publish(fused_boxes);
    publisher_fused_text_.publish(fused_objects_labels);
    boxes_frame_ = fused_boxes.header.frame_id;
    empty_frames_ = 0;

    vision_detections_ = nullptr;
    range_detections_ = nullptr;
}

///////////////////////////// Enhanced Rnage Vision Fusiopn! //////////////////////////////////////////////
autoware_msgs::DetectedObjectArray
RosRangeVisionFusionApp::FuseRangeVisionDetections2(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
                                                   const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections,
                                                   const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{   

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Filter points out of image area
    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
    for(unsigned int i = 0; i < current_sensor_cloud_ptr->points.size(); i++){
        geometry_msgs::Point temp_point;
        temp_point.x = current_sensor_cloud_ptr->points[i].x;
        temp_point.y = current_sensor_cloud_ptr->points[i].y;
        temp_point.z = current_sensor_cloud_ptr->points[i].z;
        cv::Point3f image_space_point = TransformPoint(temp_point, camera_lidar_tf_);
        cv::Point2i image_pixel = ProjectPoint(image_space_point);

        if( (image_pixel.x >= 0) && (image_pixel.x < image_size_.width)
           && (image_pixel.y >= 0) && (image_pixel.y < image_size_.height) && (image_space_point.z > 0) ){
            filtered_sensor_cloud_ptr->points.push_back(current_sensor_cloud_ptr->points[i]);
        }
    }

    // Initialize Vectors for PCL
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > vision_matched_cloud_ptr_vec;
    for (size_t i = 0; i < in_vision_detections->objects.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        vision_matched_cloud_ptr_vec.push_back(temp_ptr);
    }
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > filtered_vision_matched_cloud_ptr_vec;
    for (size_t i = 0; i < in_vision_detections->objects.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        filtered_vision_matched_cloud_ptr_vec.push_back(temp_ptr);
    }

    autoware_msgs::DetectedObjectArray fused_objects;
    fused_objects.header = in_range_detections->header;

    // For All Camera Objects
    for (size_t i = 0; i < in_vision_detections->objects.size(); i++)
    {
        auto vision_object = in_vision_detections->objects[i];
        cv::Rect vision_rect(vision_object.x, vision_object.y,
                             vision_object.width, vision_object.height);
        
        float x_min = std::numeric_limits<float>::max();

        // Points in bounding box
        for(unsigned int j=0; j < filtered_sensor_cloud_ptr->points.size(); j++){
            geometry_msgs::Point temp_point;
            temp_point.x = filtered_sensor_cloud_ptr->points[j].x;
            temp_point.y = filtered_sensor_cloud_ptr->points[j].y;
            temp_point.z = filtered_sensor_cloud_ptr->points[j].z;            
            cv::Point3f image_space_point = TransformPoint(temp_point, camera_lidar_tf_);
            cv::Point2i image_pixel = ProjectPoint(image_space_point);

            if( (image_pixel.x > vision_rect.x) && (image_pixel.x < vision_rect.x + vision_rect.width) &&
                    (image_pixel.y > vision_rect.y) && (image_pixel.y < vision_rect.y + vision_rect.height)){                
                
                vision_matched_cloud_ptr_vec[i]->points.push_back(filtered_sensor_cloud_ptr->points[j]);
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

        for(unsigned int k=0; k<vision_matched_cloud_ptr_vec[i]->points.size(); k++){
            float point_x = vision_matched_cloud_ptr_vec[i]->points[k].x;
            min_x = (min_x > point_x) ? point_x : min_x;
        }

        float object_distance = min_x;
        if(object_distance > 150) object_distance = 0;

        ROS_INFO("object_distance : %f", object_distance);
        
        // Create Filtered Point Cloud
        float filter_threshold = dimension_.x;

        for(unsigned int j=0; j < vision_matched_cloud_ptr_vec[i]->points.size(); j++){
            float point_x;
            point_x = vision_matched_cloud_ptr_vec[i]->points[j].x;
            
            if( (std::abs(point_x - object_distance) > filter_threshold) ) continue;            
            
            filtered_vision_matched_cloud_ptr_vec[i]->points.push_back(vision_matched_cloud_ptr_vec[i]->points[j]);
            // filtered_vision_matched_cloud_ptr_vec[0]->points.push_back(vision_matched_cloud_ptr_vec[i]->points[j]);
        }

        sensor_msgs::PointCloud2 cluster_cloud_;
        ROS_INFO("filter size : %d", (int)filtered_vision_matched_cloud_ptr_vec[i]->points.size());

        pcl::toROSMsg(*filtered_vision_matched_cloud_ptr_vec[i], cluster_cloud_);
        cluster_cloud_.header = in_sensor_cloud->header;
        // Publish Last Filtered Points
        publisher_test_points_.publish(cluster_cloud_);

        // Set Pose
        for(unsigned int k=0; k<filtered_vision_matched_cloud_ptr_vec[i]->points.size(); k++){
            float point_y = filtered_vision_matched_cloud_ptr_vec[i]->points[k].y;
            float point_z = filtered_vision_matched_cloud_ptr_vec[i]->points[k].z;

            min_y = (min_y > point_y) ? point_y : min_y;
            min_z = (min_z > point_z) ? point_z : min_z;
            max_z = (max_z < point_z) ? point_z : max_z;
        }

        geometry_msgs::Pose pose_;
        
        pose_.position.x = object_distance + dimension_.x / 2;
        pose_.position.y = min_y + dimension_.y / 2;
        pose_.position.z = min_z + dimension_.z / 2;
        
        //-0.000, -0.131, -0.000, 0.991
        // tf와의 관계가 왜 이런지 잘 모르겠음
        pose_.orientation.x = -1 * lidar_base_tf_.getRotation().x();
        pose_.orientation.y = -1 * lidar_base_tf_.getRotation().y();
        pose_.orientation.z = -1 * lidar_base_tf_.getRotation().z();
        pose_.orientation.w = lidar_base_tf_.getRotation().w();
        // pose_.orientation.x = camera_lidar_tf_.getRotation().x();
        // pose_.orientation.y = camera_lidar_tf_.getRotation().y();
        // pose_.orientation.z = camera_lidar_tf_.getRotation().z();
        // pose_.orientation.w = camera_lidar_tf_.getRotation().w();
        ROS_INFO("pose x : %f / pose y : %f / pose z : %f", pose_.position.x, pose_.position.x, pose_.position.z);
        ROS_INFO("Orientation x: %f/ y: %f / z: %f / w: %f", pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);

        // Convex Hull
        // std::vector<cv::Point2f> points;
        // for (unsigned int j = 0; j < filtered_vision_matched_cloud_ptr_vec[i]->points.size(); j++)
        // {
        //     cv::Point2f pt;
        //     pt.x = filtered_vision_matched_cloud_ptr_vec[i]->points[j].x;
        //     pt.y = filtered_vision_matched_cloud_ptr_vec[i]->points[j].y;
        //     points.push_back(pt);
        // }
        // std::vector<cv::Point2f> hull;
        // cv::convexHull(points, hull);
        // geometry_msgs::PolygonStamped polygon_;
        // polygon_.header = in_sensor_cloud->header;
        // for (size_t j = 0; j < hull.size() + 1; j++)
        // {
        //     geometry_msgs::Point32 point;
        //     point.x = hull[j % hull.size()].x;
        //     point.y = hull[j % hull.size()].y;
        //     point.z = min_z;
        //     polygon_.polygon.points.push_back(point);
        // }        
        // for (size_t i = 0; i < hull.size() + 1; i++)
        // {
        //     geometry_msgs::Point32 point;
        //     point.x = hull[i % hull.size()].x;
        //     point.y = hull[i % hull.size()].y;
        //     point.z = max_z;
        //     polygon_.polygon.points.push_back(point);
        // }
        // OK //////////////////////////////////////////////////

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

        detected_object.space_frame = in_range_detections->header.frame_id; // OK
        detected_object.pose = pose_; // ok
        detected_object.dimensions = dimension_; // OK
        detected_object.pointcloud = cluster_cloud_; // OK
        //detected_object.convex_hull = polygon_; // OK
        detected_object.color.r = 20*i;
        detected_object.color.g = 20*i;
        detected_object.color.b = 20*i;
        detected_object.color.a = 1.;
        
        fused_objects.objects.push_back(detected_object);
    }

    // Test
    // sensor_msgs::PointCloud2 vision_matched_point_cloud;
        
    // pcl::toROSMsg(*vision_matched_cloud_ptr_vec[0], vision_matched_point_cloud);
    // pcl::toROSMsg(*filtered_vision_matched_cloud_ptr_vec[0], vision_matched_point_cloud);
    // vision_matched_point_cloud.header = in_sensor_cloud->header;

    // publisher_test_points_.publish(vision_matched_point_cloud);
    
    
    return fused_objects;
}