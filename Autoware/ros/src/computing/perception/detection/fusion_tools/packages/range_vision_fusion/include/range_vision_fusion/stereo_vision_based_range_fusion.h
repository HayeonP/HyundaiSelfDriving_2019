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
 * range_vision_fusion_node.h
 *
 *  Created on: July, 05th, 2018
 */

#ifndef PROJECT_RANGE_VISION_FUSION_H
#define PROJECT_RANGE_VISION_FUSION_H

#define __APP_NAME__ "stereo_enhanced_range_vision_fusion"

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <jsk_recognition_utils/geo/cube.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_msgs/DetectedObjectArray.h"

// Start
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <object_manager_msgs/combined.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

// End



class RosRangeVisionFusionApp
{
    ros::NodeHandle                     node_handle_;
    ros::Publisher                      publisher_fused_objects_;
    
    //debug
    ros::Publisher                      debug_obj_pub_;
    ros::Publisher                      debug_box_pub_;

    ros::Publisher                      publisher_fused_text_;
    //Start
    ros::Publisher                      publisher_AMB_object_;
    //ros::Publisher                      publisher_AMB_box_;
    //End

    ros::Subscriber                     left_intrinsics_subscriber_, right_intrinsics_subscriber_;
    ros::Subscriber                     left_detections_vision_subscriber_, right_detections_vision_subscriber_;
    ros::Subscriber                     detections_lidar_subscriber_;
    ros::Subscriber                     points_raw_subscriber_;
    
    // debug
    ros::Publisher                      publisher_AMB_test_points_;
    ros::Publisher                      publisher_obj_test_points_;
    ros::Publisher                      publisher_img_test_points_;
    ros::Publisher                      publisher_obj_test_marker_;
    
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
                                        *left_vision_filter_subscriber_, *right_vision_filter_subscriber_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>
                                        *lidar_filter_subscriber_;

    tf::TransformListener*              transform_listener_;
    tf::StampedTransform                left_camera_lidar_tf_, right_camera_lidar_tf_;
    // Start
    tf::StampedTransform                lidar_base_tf_;
    tf::StampedTransform                base_lidar_tf_;
    tf::StampedTransform                map_lidar_tf_;
    // End

    cv::Size                            left_image_size_, right_image_size_;
    cv::Mat                             left_camera_instrinsics_, right_camera_instrinsics_;
    cv::Mat                             left_distortion_coefficients_, right_distortion_coefficients_;

    cv::Mat                             left_image_, right_image_;
    ros::Subscriber                     left_image_subscriber_, right_image_subscriber_;

    autoware_msgs::DetectedObjectArray::ConstPtr  left_vision_detections_, right_vision_detections_, range_detections_;
    // Start
    sensor_msgs::PointCloud2ConstPtr cloud_detections_;
    // End

    std::string                         left_image_frame_id_, right_image_frame_id_;
    std::string                         boxes_frame_;
    // Start
    std::string                         base_id_;
    std::string                         left_camera_name_, right_camera_name_, map_name_;
    // End

    bool                                processing_;
    bool                                left_camera_info_ok_, right_camera_info_ok_;
    bool                                left_camera_lidar_tf_ok_, right_camera_lidar_tf_ok_;
    bool                                map_lidar_tf_ok_;
    // Start
    bool                                lidar_base_tf_ok_;
    // End
    float                               left_fx_, left_fy_, left_cx_, left_cy_;
    float                               right_fx_, right_fy_, right_cx_, right_cy_;
    double                              overlap_threshold_;

    double                              car_width_, car_height_, car_depth_;
    double                              person_width_, person_height_, person_depth_;
    double                              truck_width_, truck_depth_, truck_height_;

    size_t                              empty_frames_;

    int                                 x_distance_filter_param_ = 5;

    typedef
    message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,
                                                    autoware_msgs::DetectedObjectArray,
                                                    sensor_msgs::PointCloud2>  SyncPolicyT;

    message_filters::Synchronizer<SyncPolicyT>
                                        *detections_synchronizer_;

    void CheckMinimumDimensions(autoware_msgs::DetectedObject &in_out_object);

    jsk_recognition_msgs::BoundingBoxArray ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects);

    void VisionDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_image_msg);

    // Start
    void LidarDetectionsCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);

    void SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &left_in_vision_detections,
                                  const autoware_msgs::DetectedObjectArray::ConstPtr &right_in_vision_detections,
                                  const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);

    autoware_msgs::DetectedObjectArray FuseRangeVisionDetections(const autoware_msgs::DetectedObjectArray::ConstPtr & left_in_vision_detections,
                                                                 const autoware_msgs::DetectedObjectArray::ConstPtr & right_in_vision_detections,
                                                                 const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);
    
    bool x_distance_check(pcl::PointXYZ target_point, pcl::PointCloud<pcl::PointXYZ>::Ptr points, geometry_msgs::Vector3 dimension);
    static bool point_x_compare(pcl::PointXYZ& a, pcl::PointXYZ& b);
    static bool AMB_x_compare(pcl::PointXYZ& a, pcl::PointXYZ& b);
    // End

    
    cv::Point3f TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform);

    cv::Point2i ProjectPoint(const cv::Point3f &in_point, std::string camera_name);

    autoware_msgs::DetectedObject TransformObject(const autoware_msgs::DetectedObject &in_detection,
                                                   const tf::StampedTransform& in_transform);

    autoware_msgs::DetectedObject MergeObjects(const autoware_msgs::DetectedObject &in_object_a,
                                               const autoware_msgs::DetectedObject & in_object_b);

    void CalculateObjectFeatures(autoware_msgs::DetectedObject &in_out_object,
                                 bool in_estimate_pose);

    visualization_msgs::MarkerArray ObjectsToMarkers(const autoware_msgs::DetectedObjectArray &in_objects);

    double GetDistanceToObject(const autoware_msgs::DetectedObject &in_object);

    /*!
     * Obtains Transformation between two transforms registered in the TF Tree
     * @param in_target_frame
     * @param in_source_frame
     * @return the found transformation in the tree
     */
    tf::StampedTransform
    FindTransform(const std::string &in_target_frame, const std::string &in_source_frame);

    void LeftIntrinsicsCallback(const sensor_msgs::CameraInfo& in_message);
    void RightIntrinsicsCallback(const sensor_msgs::CameraInfo& in_message);

    /*!
     * Reads the config params from the command line
     * @param in_private_handle
     */
    void InitializeRosIo(ros::NodeHandle &in_private_handle);

    // Start
    object_manager_msgs::combined AMB_parse(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);
    void get_poly_point_from_pcl(pcl::PointCloud<pcl::PointXYZ> point_cloud, std::vector<geometry_msgs::Point32>& poly_point);
    void polygon_to_marker(const geometry_msgs::PolygonStamped polygon, visualization_msgs::Marker& marker, int id);
    bool count_points_in_boundary(pcl::PointXYZ& target_point, pcl::PointCloud<pcl::PointXYZ>::Ptr points, geometry_msgs::Vector3 dimension, int threshold);
    // End

public:
    void Run();
    RosRangeVisionFusionApp();
};


#endif //PROJECT_RANGE_VISION_FUSION_H
