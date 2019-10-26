#include <iostream>
#include <ros/ros.h>
#include <ndtscore_publisher/NDTStat.h>

static float score;

void scoreCallback(const ndtscore_publisher::NDTStat::ConstPtr &msg){
    std::cout << "-------------------------------" << std::endl;        
    std::cout << "score : " << msg->score << std::endl << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ndtscore_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber score_sub;
    score_sub = nh.subscribe("ndt_stat", 100, scoreCallback);
    ros::spin();

    return 0;
}