#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char *argv[]){
    ros::init(argc, argv, "pub_twist_main");
    ros::NodeHandle nh;

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 10);
    if (argc != 3) throw std::runtime_error("argc != 3");

    double v = std::stod(argv[1]);
    double w = std::stod(argv[2]);

    ros::Rate loop_rate(10);
    geometry_msgs::TwistStamped twist;
    while(ros::ok()){
        twist.header.stamp = ros::Time::now();
        twist.twist.linear.x = v;
        twist.twist.angular.z = w;
        twist_pub.publish(twist);
        loop_rate.sleep();
    }

    return 0;
}