#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <sys/timerfd.h>
#include <sys/mman.h>
#include <errno.h>
#include <iostream>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#define MAX_NSENSORS 8 

int quit(char *tag);
int init_timer(int msec);

namespace LidarNodeletBack{

  struct pose
  {
    double roll;
    double pitch;
    double yaw;
    double x;
    double y;
    double z;
    //below is not used
    bool isquaternion;
    double qx;
    double qy;
    double qz;
    double qw;
  };//struct pose

  class InputCloud
  {
  public:
    InputCloud(pose p,std::string topic,ros::NodeHandle nh);
    ~InputCloud() {}
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    void robot_sensors();
    Eigen::Matrix4f poseTotfmatrix();
    pcl::PointCloud<pcl::PointXYZI> tfdinCloud;
  private:
    pose ps;
    std::string topic_name;
    std::string frame_id;
    ros::Subscriber sub;
    Eigen::Matrix4f transform;
    pcl::PointCloud<pcl::PointXYZI> inCloud;
  };//class InputCloud

  class OutputCloud
  {
  public:
    OutputCloud(std::string topic,std::string frame,ros::NodeHandle nh);
    ~OutputCloud() {}
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    sensor_msgs::PointCloud2 outCloudMsg;
    ros::Publisher pub;
  private:
    std::string topic_name;
    std::string frame_id;
		
  };//class OutputCloud

  class LidarNodeletBack : public nodelet::Nodelet
  {
  public:
    LidarNodeletBack() {};
    void mergeNpub();
    ~LidarNodeletBack() {}
    virtual void onInit();
  private:
    double batchNum;
    int nsensors;
    InputCloud* inClAry[MAX_NSENSORS];
    OutputCloud* outCl;
  };//class CloudMerger


}// namespace of CloudMerger
PLUGINLIB_EXPORT_CLASS(LidarNodeletBack::LidarNodeletBack, nodelet::Nodelet)
