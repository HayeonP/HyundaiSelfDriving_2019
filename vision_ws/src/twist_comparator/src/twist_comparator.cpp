#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom;
geometry_msgs::TwistStamped twist;

void cb1(const geometry_msgs::TwistStampedConstPtr& ptr){
    twist = *ptr;
    ROS_INFO("target - %lf %lf", twist.twist.linear.x, twist.twist.angular.z);
    ROS_INFO("cur - %lf %lf", odom.twist.twist.linear.x, odom.twist.twist.angular.z);
}
void cb2(const nav_msgs::OdometryConstPtr& ptr){
    odom = *ptr;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "twist_comparator");
    ros::NodeHandle nh;

    ros::Subscriber s1 = nh.subscribe("twist_cmd", 10, cb1);
    ros::Subscriber s2 = nh.subscribe("vehicle/odom", 10, cb2);
    ros::spin();
}