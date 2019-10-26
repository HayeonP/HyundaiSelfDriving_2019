#ifndef __LIDARNODELETFRONT_H
#define __LIDARNODELETFRONT_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "lidar.h"

/****Lidar Mode****/
enum {VLP_ONLY, OSI_ONLY, VLP_OSI_FUSION};

namespace LidarNodeletFront{
  struct param{
    int mode;
    std::string pcap_file;
    std::string ip_addr[3];
  };
  class LidarNodeletFront : public nodelet::Nodelet{
  public:
  LidarNodeletFront() : socket_flag(false), queue_size(100) {}
    ~LidarNodeletFront() {}
  private:
    ros::Publisher pub_msg[3];
    // nodelet_lidar_driver::VLP16 lidar_msg;
    nodelet_lidar_driver::OSI64 lidar_msg;
    bool socket_flag, ret;
    int sockfd, queue_size;
    sockaddr_in sender_address;
    pcap_t *pcap_fd;
    struct param param;
    void pubMsg(std::string& dst_ipaddr, ip_pkt& ip_pkt);
    void extMsg();
    virtual void onInit();
  };
}
PLUGINLIB_EXPORT_CLASS(LidarNodeletFront::LidarNodeletFront, nodelet::Nodelet)
#endif
