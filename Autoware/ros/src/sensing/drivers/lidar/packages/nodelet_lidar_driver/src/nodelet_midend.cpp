#include <nodelet_lidar_driver/nodelet_midend.h>
Calibration calibration_;
namespace LidarNodeletMid{

  void LidarNodeletMid::LidarMsgCallback(const nodelet_lidar_driver::OSI64::ConstPtr& ptr){ // ???  vlp/osi 다름
    // msg_vector.push_back(ptr);
    // OSI64::add_OSI64_msg_to_cloud(*ptr, cloud);
    count++;
    OSI64::add_OSI64_msg_to_cloud(*ptr, cloud);
    OSI64::print_OSI64_packet(*ptr);
    if(count == param.batchNum){
      pcl::toROSMsg(cloud, point_msg);
      point_msg.header.frame_id = param.frame;
      point_msg.header.stamp = ptr->ts;
      pub_cloud.publish(point_msg);
      cloud.clear();
      count = 0;
    }
  }

  void LidarNodeletMid::onInit(){
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    private_nh.getParam("calibration_path", param.calibration_path);
    private_nh.getParam("pub_topic", param.pub_topic);
    private_nh.getParam("sub_topic", param.sub_topic);
    private_nh.getParam("frame", param.frame);
    private_nh.getParam("batchNum", param.batchNum);

    Calibration calibration(param.calibration_path, false);
    if (calibration.initialized == false) {
      ROS_ERROR("main() : cannot read VLP16 calibration file");
      exit(1);
    }
    calibration_ = calibration;
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(param.pub_topic, queue_size);
    sub_lidar = nh.subscribe(param.sub_topic, 100, &LidarNodeletMid::LidarMsgCallback, this);
  }
}
