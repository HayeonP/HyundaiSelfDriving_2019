#ifndef __ICHTHUS_CONFIG_H
#define __ICHTHUS_CONFIG_H

#include <iostream>
#include <ros/ros.h>
#include <autoware_config_msgs/ConfigNDT.h>

namespace IchthusCfg{
class IchthusCfg{
public:
    IchthusCfg(ros::NodeHandle _nh, ros::NodeHandle _private_nh);
    ~IchthusCfg(){};
private:
    autoware_config_msgs::ConfigNDT _config_ndt;
    ros::Publisher _ndt_pub;
    static int _config_num;
    void run();
    void pubConfig();
    void pubConfigNDT();
};
}

#endif