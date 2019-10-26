#ifndef __LIDAR_NODELET_H
#define __LIDAR_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <sys/timerfd.h>
#include <sys/mman.h>
#include <errno.h>

#include "lidar.h"

namespace LidarNodeletMid{
  const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
  const uint16_t ROTATION_MAX_UNITS    =    36000u;  // [deg/100]
  struct param{
    std::string calibration_path;
    std::string pub_topic;
    std::string sub_topic;
    std::string frame;
    int batchNum;
  };
  class LidarNodeletMid : public nodelet::Nodelet{
  private:
    ros::Publisher pub_cloud;
    ros::Subscriber sub_lidar;
    //nodelet_lidar_driver::VLP16 lidar_msg;
    sensor_msgs::PointCloud2 point_msg;
    pcl::PointCloud<PointXYZIR> cloud;
    struct param param;
    int count, queue_size;
    virtual void onInit();
    void LidarMsgCallback(const nodelet_lidar_driver::OSI64::ConstPtr& msg);
    void cloudPub(const nodelet_lidar_driver::OSI64::ConstPtr& msg);
    // const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
    // const uint16_t ROTATION_MAX_UNITS    =    36000u;  // [deg/100]
    int timerfd;
    unsigned long long missed;

    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

    double b_Start, b_End;
    double last_Start, last_End;
    double startTime, endTime, totTime = 0;
    std::vector<nodelet_lidar_driver::VLP16::ConstPtr> msg_vector;

  public:
  LidarNodeletMid() : count(0), queue_size(1000) {
      for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
	      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
	      cos_rot_table_[rot_index] = cosf(rotation);
	      sin_rot_table_[rot_index] = sinf(rotation);
      }
    };
    ~LidarNodeletMid() {}
  };
}
PLUGINLIB_EXPORT_CLASS(LidarNodeletMid::LidarNodeletMid, nodelet::Nodelet)
#endif
