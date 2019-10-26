#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <vector>
#include "std_msgs/Int32MultiArray.h"

#define NumOfVectormap 9

class VectorMap {
public:
    VectorMap(int id, std::vector<int> path) {id = this->id; path = this->element;}
    ~VectorMap() {}

private:
    int id;
    std::vector<int> element;
};

class LoadVectorMap{
    public:
        LoadVectorMap():nh(){
            sub = nh.subscribe("workzone_blocked_paths", 1000, &LoadVectorMap::selectPath, this);
        }
        void selectPath(const std_msgs::Int32MultiArrayConstPtr& msgs);
        int setVectorMap(int idx){validVectormapIdx = idx; return 0;}
        int getVectorMap(){return validVectormapIdx;}

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        int validVectormapIdx = 0;
};

void LoadVectorMap::selectPath(const std_msgs::Int32MultiArrayConstPtr& msgs) {
    int validVectormap[NumOfVectormap] = {1, 1, 1, 1, 1, 1, 1, 1, 1}; 
    int numOfInvalid = 0;
    int workzoneID;
    
    std::vector<std::vector<int>> pathArray;
    
    std::vector<int> zone1 = {2, 5, 9};
    std::vector<int> zone2 = {1, 4, 6};
    std::vector<int> zone3 = {3, 7, 8};
    std::vector<int> zone4 = {4, 5, 8, 9};
    std::vector<int> zone5 = {6, 7, 8, 9};
    std::vector<int> zone6 = {2, 4, 8};
    std::vector<int> zone7 = {1, 5, 7};
    std::vector<int> zone8 = {3, 6, 9};
    pathArray.push_back(zone1); 
    pathArray.push_back(zone2);
    pathArray.push_back(zone3);
    pathArray.push_back(zone4);
    pathArray.push_back(zone5);
    pathArray.push_back(zone6);
    pathArray.push_back(zone7);
    pathArray.push_back(zone8);
    for(int i = 0; i < msgs->data.size(); i++){
        workzoneID = msgs->data[i] -1 ;
        // ROS_INFO("workzone[%d] : size: %d ", workzoneID, pathArray[workzoneID-1].size());
        for(int j = 0; j < pathArray[workzoneID].size(); j++){
            // ROS_INFO(" %d,", pathArray[workzoneID-1][j]);
            validVectormap[pathArray[workzoneID][j] - 1] = 0;
        }
    }
    
    for(int idx = 0; idx < NumOfVectormap; idx++){
        if(validVectormap[idx] == 1){ 
            validVectormapIdx = idx + 1;
		break;
	}
        else numOfInvalid++;
    }
    if(numOfInvalid == NumOfVectormap)
        ROS_WARN("selectPath() : There is no valid VECTOR MAP");
    else
        ROS_WARN("valid Map: %d", validVectormapIdx);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "avoid_workzone");

    int selectedVmap;
    LoadVectorMap loadVectormap;    
    std::string cmd = "roslaunch avoid_workzone_avc load_vectormap";

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        selectedVmap = loadVectormap.getVectorMap();
        if(selectedVmap != 0) break;
        loop_rate.sleep();
    }
    // std::cout << "selectedVmap" <<selectedVmap <<std::endl;
    cmd += std::to_string(selectedVmap) + ".launch";
    std::cout << cmd<< std::endl;
    system(cmd.c_str());
    sleep(10);
    return 0;
}
