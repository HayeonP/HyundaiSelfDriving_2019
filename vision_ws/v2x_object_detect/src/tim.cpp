#include <v2x_processor/tim.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_broadcaster.h>
#include <GeographicLib/Geocentric.hpp>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <cstdlib>
#include <thread>
#include <algorithm>


TIMProcessor::TIMProcessor(ros::NodeHandle& nh, std::string combined_topic_name)
    : private_nh("~"), earth(Constants::WGS84_a(), Constants::WGS84_f())
    {
    combined_msg_pub = nh.advertise<object_manager_msgs::combined>(
        combined_topic_name, 10
    );
    tim_workzone_debug = nh.advertise<visualization_msgs::MarkerArray>(
        "tim_debug", 10
    );
    tim_mission_manager_plugin_pub = nh.advertise<std_msgs::Int32MultiArray>(
        "workzone_blocked_paths", 10
    );

    std::cout.precision(15);
    std::cout << std::fixed;

    //load predefined workzone latlon
    std::vector<double> latlon_tmp;
    anchor_latlon_vec.resize(N_PATH);
    for(int i = 1; i <= N_PATH ; ++i){
        std::string name_param = "tim_anchor_latlon_point" + std::to_string(i);
        if (!private_nh.getParam(name_param, latlon_tmp))
            throw std::runtime_error(std::string() + "set " + name_param + "!");
        anchor_latlon_vec[i-1].resize(latlon_tmp.size() / 2);
        
        auto it = latlon_tmp.begin();
        int idx = 0;
        while(it != latlon_tmp.end()){
            LatLon latlon;
            latlon.lat = *it++;
            latlon.lon = *it++;
            anchor_latlon_vec[i-1][idx++] = latlon;
            if ((it - latlon_tmp.begin()) > 100) //detect parameter setting fault
                throw std::runtime_error("anchor point latlon is not dual");
        }
    }
    for(int i = 0 ; i < N_PATH; ++i){
        auto& vec = anchor_latlon_vec[i];
        std::cout << "anchor latlon vec " << i << " - ";
        for(auto& latlon : vec){
            std::cout << '(' << latlon.lat << ", " << latlon.lon << "), ";
        }
        std::cout << std::endl;
    }

    /* convert anchor_latlon to xyz, and translate each point to visualize */
    anchor_xyz_vec.resize(N_PATH);
    for(int anchor_idx = 0 ; anchor_idx < N_PATH; ++anchor_idx){
        auto& anchor_latlon = anchor_latlon_vec[anchor_idx];
        auto& anchor_xyz = anchor_xyz_vec[anchor_idx];
        anchor_xyz.resize(anchor_latlon.size());

        const size_t N_ANCHOR = anchor_latlon.size();
        for(size_t j = 0 ; j < N_ANCHOR; ++j){
            const double lat = anchor_latlon[j].lat;
            const double lon = anchor_latlon[j].lon;
            const double alt = KCITY_ALTITUDE;
            earth.Forward(
                lat, 
                lon, 
                alt, 
                anchor_xyz[j].x,
                anchor_xyz[j].y,
                anchor_xyz[j].z
            );
        }
    }
    ROS_ERROR("hmm");
    //normalize anchor xyz with first point
    xyz_standard = anchor_xyz_vec[0].front();
    for(auto& vec : anchor_xyz_vec){
        for(auto rit = vec.rbegin(); rit != vec.rend(); ++rit){
            rit->x -= xyz_standard.x;
            rit->y -= xyz_standard.y;
            rit->z -= xyz_standard.z;
        }
    }
    ROS_ERROR("hmm");
    
    //before workzone v2x goes stable, I'll use fake interface
    srand((unsigned)time(nullptr));    
    int selected_planning = rand() % 9;
    ROS_WARN("selected path : %d", selected_planning + 1);

    std::thread([&](int s){
        while(ros::ok()){
            publishFakeWorkzone(s);
            ros::Rate(10).sleep();
        }
    }, selected_planning).detach();
}

void TIMProcessor::setTfProcessor(const TransformationMatrixProcessor& tf){
    tf_processor = tf;
}


void TIMProcessor::processTIM(const v2x_msgs::v2x_infoConstPtr& ptr){
    ROS_INFO("processTim");
    
    boundingBoxAry.header.frame_id = "map";
    boundingBoxAry.header.stamp = ros::Time::now();
    boundingBoxAry.boxes.resize(ptr->tim_dataframe_cnt);
    detectedObjAry.header.frame_id = "map";
    detectedObjAry.header.stamp = ros::Time::now();
    detectedObjAry.objects.resize(ptr->tim_dataframe_cnt);
    int cur_latlon_idx = 0;
    int cur_cnt_idx = 0;
    if (!ptr->tim_dataframe_cnt) return;
    for(int i = 0 ; i < ptr->tim_dataframe_cnt; ++i){
        jsk_recognition_msgs::BoundingBox& box = boundingBoxAry.boxes[i];
        box.header.seq++;
        box.header.stamp = ros::Time::now();
        box.header.frame_id = "map";
        //find boundingbox with start, end latlon
        
        double lat_start = ptr->tim_nodelist_xy_latlon[cur_latlon_idx]/10000000.0;
        double lon_start = ptr->tim_nodelist_xy_latlon[cur_latlon_idx+1]/10000000.0;
        cur_latlon_idx += 2 * (ptr->tim_nodelist_xy_cnt[cur_cnt_idx] - 1);
        cur_cnt_idx++;
        
        double lat_end = ptr->tim_nodelist_xy_latlon[cur_latlon_idx]/10000000.0;
        double lon_end = ptr->tim_nodelist_xy_latlon[cur_latlon_idx+1]/10000000.0;
        cur_latlon_idx += 2;
        
        ROS_INFO("[tim]start : %lf %lf", lat_start, lon_start);
        ROS_INFO("[tim]end : %lf %lf", lat_end, lon_end);

        double start_map_x, start_map_y, start_map_z;
        double end_map_x, end_map_y, end_map_z;
        
        tf_processor.convertLatLonAltToMapXYZ(lat_start, lon_start, 0, 
            start_map_x, start_map_y, start_map_z);
        tf_processor.convertLatLonAltToMapXYZ(lat_end, lon_end, 0, 
            end_map_x, end_map_y, end_map_z);

        box.pose.position.x = (start_map_x + end_map_x)/2.0; //ptr->tim_anchor_lat;
        box.pose.position.y = (start_map_y + end_map_y)/2.0; //ptr->tim_anchor_lon;
        ROS_INFO("[tim] x, y : %lf, %lf", box.pose.position.x, box.pose.position.y);

        double dx = end_map_x - start_map_x;
        double dy = end_map_y - start_map_y;
        double yaw = std::atan2(dy, dx);
        box.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        double length = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        box.dimensions.x = length * 2;//계산 방법 고민 필요
        box.dimensions.y = ptr->tim_lanewidth[i]/1000.0;
        box.dimensions.z = 3;
        ////////////
        autoware_msgs::DetectedObject& obj = detectedObjAry.objects[i];
        obj.header.seq++;
        obj.header.stamp = ros::Time::now();
        obj.header.frame_id = "map";

        obj.label = "workzone";
        obj.pose = box.pose;
        obj.dimensions = box.dimensions;
    }
    publish();

    //findBlockedPathAndPublish(ptr);//rubicom's mission manager plugin
}

void TIMProcessor::publishFakeWorkzone(int selected_planning){
   //rubicom interface
    std::vector<std_msgs::Int32MultiArray> blocked_path_samples;
    blocked_path_samples.resize(9); // there are 9 feasible global planning
    blocked_path_samples[0].data = {1, 3, 5, 7, 8};
    blocked_path_samples[1].data = {2, 3, 5, 6, 8};
    blocked_path_samples[2].data = {2, 3, 4, 5, 7, 8};
    blocked_path_samples[3].data = {1, 3, 4, 6, 7};
    blocked_path_samples[4].data = {1, 2, 4, 6, 8};
    blocked_path_samples[5].data = {1, 2, 4, 5, 6, 7};
    blocked_path_samples[6].data = {1, 3, 5, 7, 8};
    blocked_path_samples[7].data = {1, 2, 7, 8};
    blocked_path_samples[8].data = {2, 3, 6, 7};
    
    auto msg = blocked_path_samples[selected_planning];
    tim_mission_manager_plugin_pub.publish(msg);
}

void TIMProcessor::findBlockedPathAndPublish(const v2x_msgs::v2x_infoConstPtr& ptr){
    /*
        mission 2 looks like :
           G      G : Goal
         / l \    S : Start
        []-[]-[]  l, - : road
         \ l /    [] : intersection
           S 
        This function find path, and publish appropriate vectormap's name.
        I make each street id as :
           G      
         1 2 3    
        []4[]5[]  
         6 7 8
           S 
    */
    
    static constexpr double WORKZONE_CLOSEST_ANCHOR_CUT_DISTANCE = 100;
    
    /* convert TIM workzone latlon to normalized xyz*/
    PoseXYZ& xyz_standard = anchor_xyz_vec[0].front();
    std::vector<PoseXYZ> target_candidate;
    if (!ptr->tim_nodelist_xy_latlon.size()) return;
    for(size_t i = 0 ; i < ptr->tim_nodelist_xy_latlon.size(); ++i){
        double lat = ptr->tim_nodelist_xy_latlon[i]/1000000.0;
        double lon = ptr->tim_nodelist_xy_latlon[i+1]/1000000.0;
        
        PoseXYZ pose;
        earth.Forward(lat, lon, KCITY_ALTITUDE, 
            pose.x, pose.y, pose.z);
        
        pose.x -= xyz_standard.x;
        pose.y -= xyz_standard.y;
        pose.z -= xyz_standard.z;

        target_candidate.push_back(pose);
    }

    /* find closed anchor for each point */
    std::vector<int> score(N_PATH);
    std::fill_n(score.begin(), score.size(), -1); //to avoid noise, default score is -1. I'll
    //pick blocked pathes if there are at least 2 points contained in anchor.
    
    for(size_t i = 0 ; i < target_candidate.size(); ++i){
        double d_max = 9999999;
        int closest_anchor_idx = -1;
        double d;

        for(int anchor_idx = 0 ; anchor_idx < N_PATH; ++anchor_idx){
            for(auto& xyz : anchor_xyz_vec[anchor_idx]){
                d = std::sqrt(
                    std::pow(target_candidate[i].x - xyz.x, 2) +
                    std::pow(target_candidate[i].y - xyz.y, 2) +
                    std::pow(target_candidate[i].z - xyz.z, 2) 
                );
                if (d < d_max){
                    d_max = d;
                    closest_anchor_idx = anchor_idx;
                }
            }
        }

        if (d_max < WORKZONE_CLOSEST_ANCHOR_CUT_DISTANCE){
            score[closest_anchor_idx]++;
            std::cout << "tim : d_max - " << d_max << '\n';
            std::cout << "tim : closest - " << closest_anchor_idx << '\n';
        }
    }

    std_msgs::Int32MultiArray msg;
    for(int anchor_idx = 0 ; anchor_idx < N_PATH; ++anchor_idx)
        if ( score[anchor_idx] > 0) msg.data.push_back(anchor_idx + 1);
    tim_mission_manager_plugin_pub.publish(msg);
    
    /* debug anchor point */
    visualization_msgs::MarkerArray markAry;
    int id = 0;
    for(int anchor_idx = 0 ; anchor_idx < N_PATH; ++anchor_idx){
        auto& xyz_vec = anchor_xyz_vec[anchor_idx];
        for(auto&& xyz : xyz_vec){
            markAry.markers.emplace_back();
            visualization_msgs::Marker& mark = markAry.markers.back();

            mark.header.frame_id = "v2x_workzone";
            mark.header.stamp = ros::Time::now();
            mark.ns = "v2x_tim";
            mark.id = id++;

            mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            mark.action = visualization_msgs::Marker::ADD;
            mark.pose.position.x = xyz.x;
            mark.pose.position.y = xyz.y;
            mark.pose.position.z = xyz.z;

            mark.pose.orientation.x = 0.0;
            mark.pose.orientation.y = 0.0;
            mark.pose.orientation.z = 0.0;
            mark.pose.orientation.w = 1.0;
            mark.scale.x = 0.5;
            mark.scale.y = 0.5; //no use
            mark.scale.z = 5;

            mark.color.a = 1; // Don't forget to set the alpha!
            mark.color.r = 0;
            mark.color.g = 1;
            mark.color.b = 0;

            mark.text = std::to_string(anchor_idx + 1);
        }
    }
    /* debug tim points */
    for(int i = 0 ; i < target_candidate.size(); ++i){
        markAry.markers.emplace_back();
        visualization_msgs::Marker& mark = markAry.markers.back();
        
        mark.header.frame_id = "v2x_workzone";
        mark.header.stamp = ros::Time::now();
        mark.ns = "v2x_tim";
        mark.id = i + 8;

        mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mark.action = visualization_msgs::Marker::ADD;
        mark.pose.position.x = target_candidate[i].x;
        mark.pose.position.y = target_candidate[i].y;
        mark.pose.position.z = target_candidate[i].z;

        mark.pose.orientation.x = 0.0;
        mark.pose.orientation.y = 0.0;
        mark.pose.orientation.z = 0.0;
        mark.pose.orientation.w = 1.0;
        mark.scale.x = 0.5;
        mark.scale.y = 0.5; //no use
        mark.scale.z = 5;

        mark.color.a = 1; // Don't forget to set the alpha!
        mark.color.r = 0.5;
        mark.color.g = 0.5;
        mark.color.b = 0;

        mark.text = "target";
    }
    tim_workzone_debug.publish(markAry);
}

void TIMProcessor::publish(){
    combined_msg.header = boundingBoxAry.header;
    combined_msg.bb_ary = boundingBoxAry;
    combined_msg.obj_ary = detectedObjAry;
    combined_msg_pub.publish(combined_msg);
}
