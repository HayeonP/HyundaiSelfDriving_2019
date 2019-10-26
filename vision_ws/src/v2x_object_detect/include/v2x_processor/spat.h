#pragma once
#include <ros/ros.h>
#include <v2x_msgs/v2x_info.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <object_manager_msgs/combined.h>


constexpr static int N_SPAT_ITEMS = 16;
constexpr static int VISUALIZATION_WIDTH = 20; 

struct SPaTCoordinate{
    virtual int* getX() = 0;
    virtual int* getY() = 0;
};

struct SPaTBoundingBoxCoordinate1: public SPaTCoordinate {
    int x[N_SPAT_ITEMS] = {-3,-1,1,3,   6,6,6,6,    3,1,-1,-3,      -6,-6,-6,-6};
    int y[N_SPAT_ITEMS] = {6,6,6,6,     3,1,-1,-3,  -6,-6,-6,-6,    -3,-1,1,3};
    int* getX() {return x;}
    int* getY() {return y;}
};

struct SPaTBoundingBoxCoordinate2: public SPaTCoordinate {
    SPaTBoundingBoxCoordinate2(){
        SPaTBoundingBoxCoordinate1 s;
        for(int i = 0 ; i < N_SPAT_ITEMS; ++i){
            x[i] = s.getX()[i] + VISUALIZATION_WIDTH;
            y[i] = s.getY()[i];
        }
    }
    int x[N_SPAT_ITEMS];
    int y[N_SPAT_ITEMS];
    int* getX() {return x;}
    int* getY() {return y;}
};

struct SPaTBoundingBoxCoordinate3: public SPaTCoordinate {
    SPaTBoundingBoxCoordinate3(){
        SPaTBoundingBoxCoordinate1 s;
        for(int i = 0 ; i < N_SPAT_ITEMS; ++i){
            x[i] = s.getX()[i];
            y[i] = s.getY()[i] + VISUALIZATION_WIDTH;
        }
    }
    int x[N_SPAT_ITEMS];
    int y[N_SPAT_ITEMS];
    int* getX() {return x;}
    int* getY() {return y;}
};

struct SPaTBoundingBoxCoordinate9: public SPaTCoordinate {
    SPaTBoundingBoxCoordinate9(){
        SPaTBoundingBoxCoordinate1 s;
        for(int i = 0 ; i < N_SPAT_ITEMS; ++i){
            x[i] = s.getX()[i] + VISUALIZATION_WIDTH;
            y[i] = s.getY()[i] + VISUALIZATION_WIDTH;
        }
    }
    int x[N_SPAT_ITEMS];
    int y[N_SPAT_ITEMS];
    int* getX() {return x;}
    int* getY() {return y;}
};

class SPaTProcessor{
public:
    SPaTProcessor(ros::NodeHandle& nh, std::string combined_topic_name);
    void debugSPaT(const v2x_msgs::v2x_infoConstPtr& ptr);
    void processSPaT(const v2x_msgs::v2x_infoConstPtr& ptr);
    void publish();
private: //debug member
    SPaTBoundingBoxCoordinate1 S1;
    SPaTBoundingBoxCoordinate2 S2;
    SPaTBoundingBoxCoordinate3 S3;
    SPaTBoundingBoxCoordinate9 S9;
    visualization_msgs::MarkerArray markAry;
    bool debug;
private: //ros member
    ros::Publisher spat_pub_debug;
    ros::NodeHandle private_nh;
    jsk_recognition_msgs::BoundingBoxArray boundingBoxAry;
    autoware_msgs::DetectedObjectArray detectedObjAry;
    std::vector<double> x_spat, y_spat;
    int signal_1_idx, signal_2_idx, signal_3_idx;

    ros::Publisher combined_msg_pub;    
    object_manager_msgs::combined combined_msg;
};

enum {
    SPAT_RED=3, SPAT_GREEN=5, SPAT_YELLOW=7
};
static int get_signal_label(int v2x_signal_value){
    switch(v2x_signal_value){
    case SPAT_RED: return 0;
    case SPAT_GREEN: return 1;
    case SPAT_YELLOW: return 2;
    default : ROS_ERROR("undefined spat signal value : %d. It will be handled as green signal", v2x_signal_value);
    return 1;
    }
}