#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <vector>
#include <math.h>

class Point{
public:
    double x;
    double y;
};

void get_poly_point_from_pcl(const sensor_msgs::PointCloud2 pcl_msg, std::vector<Point>& poly_point);

int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_convexhull");
    ros::NodeHandle nh;

    ros::Rate rate(10);
    ros::Publisher test_points_pub;
    ros::Publisher polygon_pub;
    ros::Publisher poly_points_pub;

    test_points_pub = nh.advertise<sensor_msgs::PointCloud2>("test_points", 1);
    polygon_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("test_polygon", 1);
    poly_points_pub = nh.advertise<sensor_msgs::PointCloud2>("poly_points", 1);

    std::vector<pcl::PointXYZ> point_vec;
    point_vec.push_back(pcl::PointXYZ(5,5,7));
    point_vec.push_back(pcl::PointXYZ(5,4.8,7));
    point_vec.push_back(pcl::PointXYZ(5,4.6,7));
    point_vec.push_back(pcl::PointXYZ(5,4.4,7));
    point_vec.push_back(pcl::PointXYZ(5,4.2,7));
    point_vec.push_back(pcl::PointXYZ(5,4.0,7));
    point_vec.push_back(pcl::PointXYZ(5,3.8,7));
    // point_vec.push_back(pcl::PointXYZ(5,3.6,7));
    // point_vec.push_back(pcl::PointXYZ(5,3.4,7));
    // point_vec.push_back(pcl::PointXYZ(5,3.2,7));
    point_vec.push_back(pcl::PointXYZ(5,5,7));
    point_vec.push_back(pcl::PointXYZ(5.2,3,7));
    point_vec.push_back(pcl::PointXYZ(5.4,3,7));
    point_vec.push_back(pcl::PointXYZ(5.6,3,7));
    // point_vec.push_back(pcl::PointXYZ(5.8,3,7));
    // point_vec.push_back(pcl::PointXYZ(6,3,7));
    // point_vec.push_back(pcl::PointXYZ(6.2,3,7));
    // point_vec.push_back(pcl::PointXYZ(6.4,3,7));
    point_vec.push_back(pcl::PointXYZ(6.6,3,7));

    point_vec.push_back(pcl::PointXYZ(7,5,7));
    point_vec.push_back(pcl::PointXYZ(7,4.8,7));
    point_vec.push_back(pcl::PointXYZ(7,4.6,7));
    // point_vec.push_back(pcl::PointXYZ(7,4.4,7));
    point_vec.push_back(pcl::PointXYZ(7,4.2,7));
    point_vec.push_back(pcl::PointXYZ(7,4.0,7));
    point_vec.push_back(pcl::PointXYZ(7,3.8,7));
    point_vec.push_back(pcl::PointXYZ(7,3.6,7));
    // point_vec.push_back(pcl::PointXYZ(7,3.4,7));
    // point_vec.push_back(pcl::PointXYZ(7,3.2,7));
    // point_vec.push_back(pcl::PointXYZ(5,5,7));
    // point_vec.push_back(pcl::PointXYZ(5.2,5,7));
    // point_vec.push_back(pcl::PointXYZ(5.4,5,7));
    point_vec.push_back(pcl::PointXYZ(5.6,5,7));
    point_vec.push_back(pcl::PointXYZ(5.8,5,7));
    point_vec.push_back(pcl::PointXYZ(6,5,7));
    point_vec.push_back(pcl::PointXYZ(6.2,5,7));
    point_vec.push_back(pcl::PointXYZ(6.4,5,7));
    point_vec.push_back(pcl::PointXYZ(6.6,5,7));




    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    
    for(auto it = point_vec.begin(); it != point_vec.end(); ++it){
        point_cloud.points.push_back((*it));
    }
    

    
    sensor_msgs::PointCloud2 points_msg;
    pcl::toROSMsg(point_cloud, points_msg);
    points_msg.header.frame_id = "/base_link";

    pcl::PointCloud<pcl::PointXYZ> poly_cloud;
    sensor_msgs::PointCloud2 polypoint_msg;   
    std::vector<Point> poly_point;
    get_poly_point_from_pcl(points_msg, poly_point);

    for(auto it = poly_point.begin(); it != poly_point.end(); ++it){
        pcl::PointXYZ temp_point;
        temp_point.x = it->x;
        temp_point.y = it->y;
        temp_point.z = 0;
        poly_cloud.points.push_back(temp_point);
    }


    pcl::toROSMsg(poly_cloud, polypoint_msg);
    polypoint_msg.header = points_msg.header;


    while(ros::ok()){
        points_msg.header.stamp = ros::Time::now();
        points_msg.header.seq++;

        test_points_pub.publish(points_msg);
        poly_points_pub.publish(polypoint_msg);
        ros::spinOnce();
        rate.sleep();
    }


    

    return 0;
}

void get_poly_point_from_pcl(const sensor_msgs::PointCloud2 pcl_msg, std::vector<Point>& poly_point){
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(pcl_msg, point_cloud);

    Point center;
    double x_sum = 0, y_sum = 0;

    for(auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it){
        x_sum += it->x;
        y_sum += it->y;
    }
    center.x = x_sum / point_cloud.points.size();
    center.y = y_sum / point_cloud.points.size();

    // Find the section of each point

    std::vector<std::vector<Point>> section;
    section.resize(8);

    for(auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it){
       Point cur_point;
       Point grad_1_point, grad_minus_1_point;

       cur_point.x = it->x;
       cur_point.y = it->y;
       grad_1_point.x = cur_point.x;
       grad_1_point.y = (cur_point.x) - center.x + center.y;
       grad_minus_1_point.x = cur_point.x;
       grad_minus_1_point.y = (cur_point.x) + center.x + center.y;

       if(cur_point.y >= center.y && cur_point.x > center.x){
           if(cur_point.y < grad_1_point.y) section[0].push_back(cur_point);
           else if(cur_point.y >= grad_1_point.y) section[1].push_back(cur_point);
       }
       else if(cur_point.y > center.y && cur_point.x <= center.x){
           if(cur_point.y > grad_minus_1_point.y) section[2].push_back(cur_point);
           else if(cur_point.y <= grad_minus_1_point.y) section[3].push_back(cur_point);
       }
       else if(cur_point.y <= center.y && cur_point.x < center.x){
           if(cur_point.y > grad_1_point.y) section[4].push_back(cur_point);
           else if (cur_point.y <= grad_1_point.y) section[5].push_back(cur_point);
       }
       else if(cur_point.y < center.y && cur_point.x >= center.x){
           if(cur_point.y < grad_minus_1_point.y) section[6].push_back(cur_point);
           else if(cur_point.y >= grad_minus_1_point.y) section[7].push_back(cur_point);
       }
       else{
           std::cout<<"  Point value is uneaccepatable!"<<std::endl;
       }
    }

    // Find the most far point from center point

    for(auto it1 = section.begin(); it1 != section.end(); ++it1){

        std::vector<Point>& points_in_section = (*it1);
        double max_dist = -1;
        Point dist_max_point;

        std::cout<<"Section size : "<<points_in_section.size()<<std::endl;
        if(points_in_section.empty()) continue;

        for(auto it2 = points_in_section.begin(); it2 != points_in_section.end(); ++it2){
            Point point = (*it2);
            double dist = std::sqrt(std::pow(point.x - center.x, 2) + std::pow(point.y - center.y, 2));
            if(dist > max_dist){
                max_dist = dist;
                dist_max_point = point;
            }
        }        

        poly_point.push_back(dist_max_point);
    }

    // poly_point.push_back(center);
}