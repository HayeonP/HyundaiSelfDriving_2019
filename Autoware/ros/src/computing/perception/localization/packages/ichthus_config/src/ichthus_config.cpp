#include <ichthus_config/ichthus_config.h>

namespace IchthusCfg{
IchthusCfg::IchthusCfg(ros::NodeHandle _nh, ros::NodeHandle _private_nh){
    _ndt_pub = _nh.advertise<autoware_config_msgs::ConfigNDT>("/config/ndt", 1000);
    pubConfigNDT();
}

void IchthusCfg::run(){
    // for(i = 0; i < _config_num; i++){
    //     pubConfig();
    // }
}

void IchthusCfg::pubConfigNDT(){
    _config_ndt.header.seq = 0;
    _config_ndt.header.stamp.sec = 0;
    _config_ndt.header.stamp.nsec = 0;
    _config_ndt.header.frame_id = ' ';
    _config_ndt.init_pos_gnss = 0;
    _config_ndt.x = 0.0;
    _config_ndt.y = 0.0;
    _config_ndt.z = 0.0;
    _config_ndt.roll = 0.0;
    _config_ndt.pitch = 0.0;
    _config_ndt.yaw = 0.0;
    _config_ndt.use_predict_pose = true;
    _config_ndt.error_threshold = 1.0;
    _config_ndt.resolution = 1.0;
    _config_ndt.step_size = 0.1;
    _config_ndt.trans_epsilon = 0.01;
    _config_ndt.max_iterations = 30;
    _ndt_pub.publish(_config_ndt);
}
};

int main(int argc, char **argv){
    ros::init (argc, argv, "ichthus_config");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher ndt_pub = nh.advertise<autoware_config_msgs::ConfigNDT>("/config/ndt", 1000);
    ros::Rate loop_rate(10);
    autoware_config_msgs::ConfigNDT config_ndt;

    for(int i = 0; i < 4; i++){
    loop_rate.sleep();
    config_ndt.header.seq = 0;
    config_ndt.header.stamp.sec = 0;
    config_ndt.header.stamp.nsec = 0;
    config_ndt.header.frame_id = "";
    config_ndt.init_pos_gnss = 0;
    config_ndt.x = 0.0;
    config_ndt.y = 0.0;
    config_ndt.z = 0.0;
    config_ndt.roll = 0.0;
    config_ndt.pitch = 0.0;
    config_ndt.yaw = 0.0;
    config_ndt.use_predict_pose = true;
    config_ndt.error_threshold = 1.0;
    config_ndt.resolution = 1.0;
    config_ndt.step_size = 0.1;
    config_ndt.trans_epsilon = 0.01;
    config_ndt.max_iterations = 30;
    ndt_pub.publish(config_ndt);
    ros::spinOnce();
    }

    return 0;
}
