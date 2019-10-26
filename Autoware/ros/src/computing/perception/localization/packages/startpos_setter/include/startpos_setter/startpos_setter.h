#ifndef __STARTPOS_SETTER_H
#define __STARTPOS_SETTER_H

#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <random>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <startpos_setter/PointArray.h>
#include <startpos_setter/DTLaneArray.h>
#include <startpos_setter/NDTStat.h>

#define MAX_DISTANCE 1000.0

namespace SPosSetter{
class SPosSetter{
public:
    SPosSetter(ros::NodeHandle _nh, ros::NodeHandle _private_nh);
    ~SPosSetter(){}
private:
    ros::Publisher _spose_pub;
    ros::Subscriber _vpose_sub;
    ros::Subscriber _vlane_sub;
    ros::Subscriber _ndtstat_sub;
    ros::Subscriber _gnss_sub;
    
    static startpos_setter::PointArray _vector_point;
    static startpos_setter::DTLaneArray _vector_dtlane;
    static geometry_msgs::PoseStamped _current_gnss_pose;
    static geometry_msgs::PoseStamped _previous_gnss_pose;
    static std::vector<geometry_msgs::PoseWithCovarianceStamped> _initialPoseArray;
    
    static bool _vpose_initialized;
    static bool _vlane_initialized;
    static bool _gnss_initialized;
    static bool _make_randomsample;
    static bool _is_rsample_empty;

    static float _radius_inner;
    static float _radius_outer;
    static float _gnss_threshold;
    static float _score_threshold;
    static float _dtlane_threshold;

    static int _rsample_num;
    static int _rsample_index;
    static int _ignore_count;
    static int _ignore_index;

    void vmapPoseCallback(const startpos_setter::PointArray& msg);
    void vmapLaneCallback(const startpos_setter::DTLaneArray& msg);
    void gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void ndtStatCallback(const startpos_setter::NDTStat::ConstPtr& msg);
    std::vector<geometry_msgs::PoseWithCovarianceStamped> pubStartPose(geometry_msgs::PoseStamped gnss_pose);    
};
}
#endif
