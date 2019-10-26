#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <mutex>
#include <dynamic_reconfigure/server.h>
#include <object_manager/ObjectManagerConfig.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PolygonStamped.h>
#include "object_manager/mission_handler.h"
#include "object_manager/ROIcheck.h"
#include "object_manager_msgs/combined.h"
#include <object_manager/AVCInterface.h>

#define AVC_CONTROL_RATE 100
/*
    This node subscribes to "DetectedObjectArray" and "BoundingBoxArray" topic from 
    each node. They must be dual, so publisher must publish 
    "DetectedObjectArray" and "BoundingBoxArray" together at the same time with same size
*/

/* All resources are released after publishing */

class ObjectManager{
public:
    struct ObjectInfoType{
        autoware_msgs::DetectedObjectArray detectedObjAry;
        jsk_recognition_msgs::BoundingBoxArray boundingBoxAry;
    };
    struct MissionInfo{
        int mission_identifier;
        geometry_msgs::PolygonStamped poly;
    };
public:
    ObjectManager():    
        private_nh("~"), estop(false){
            
        std::vector<std::string> combined_obj_ary_topic_names;
        //get params
        if(!private_nh.getParam("combined_obj_ary_topic_names", combined_obj_ary_topic_names))
            throw std::runtime_error("set combined_obj_ary_topic_names!");
        if(!private_nh.getParam("vision_idx", vision_idx))
            throw std::runtime_error("set vision_idx!");
        if(!private_nh.getParam("spat_idx", spat_idx))
            throw std::runtime_error("set spat_idx!");
        if(!private_nh.getParam("AMB_idx", AMB_idx))
            throw std::runtime_error("set AMB_idx!");

        objInfoVec.resize(combined_obj_ary_topic_names.size());
        use_flags.resize(combined_obj_ary_topic_names.size());

        //declare detected obj subscribers
        for(size_t i = 0 ; i < combined_obj_ary_topic_names.size(); ++i) {
            subVec.push_back(
                nh.subscribe<object_manager_msgs::combined>(
                    combined_obj_ary_topic_names[i],
                    10, 
                    std::bind(&ObjectManager::combinedSub, this, i , std::placeholders::_1)
                )
            );
        }

        printf("subscribing topics - \n");
        for(size_t i = 0 ; i < combined_obj_ary_topic_names.size(); ++i)
            printf("    %s\n", combined_obj_ary_topic_names[i].c_str());
        
        detected_object_ary_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("fused_detected_objects", 10);
        bounding_box_ary_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("fused_bounding_boxes", 10);
        cur_mission_poly_pub = 
            nh.advertise<geometry_msgs::PolygonStamped>("cur_mission_poly", 10);
        curposSub = nh.subscribe("ndt_pose", 10, &ObjectManager::curposCB, this);
        
        //dynamic reconfigure settings
        dyn_param_cb = boost::bind(&ObjectManager::param_callback, this, _1, _2);
        dyn_param_server.setCallback(dyn_param_cb);

        //load mission polygons
        std::vector<std::string> polygon_names;
        std::vector<geometry_msgs::PolygonStamped> polygons;
        if (!private_nh.getParam("polygon_names", polygon_names))
            throw std::runtime_error("set polygon_names!");
        for(auto& name : polygon_names){
            std::vector<double> poly_tmp;
            if (!private_nh.getParam(name.c_str(), poly_tmp))
                throw std::runtime_error("polygon - set " + name);
            else {
                polygons.emplace_back();
                for(auto it = poly_tmp.begin(); it != poly_tmp.end();){
                    polygons.back().polygon.points.emplace_back();
                    polygons.back().polygon.points.back().x = *it++;
                    polygons.back().polygon.points.back().y = *it++;
                }
            }
        }

        bool show_poly = false;
        private_nh.getParam("show_poly", show_poly);
        if (show_poly){
            for (auto& poly : polygons){
                ros::Rate(1).sleep();//need to show in rviz
                poly.header.frame_id = "map";
                poly.header.stamp = ros::Time::now();
                poly.header.seq++;
                cur_mission_poly_pub.publish(poly); //the header field will not be used in the future
            }
            ros::Rate(1).sleep();//need to show in rviz
        }

        //map polygon and identifier. This code dependent on config file.
        mission_info_vec.resize(N_MISSION);
        mission_info_vec[MISSION_NONE].mission_identifier = MISSION_NONE;
        mission_info_vec[MISSION_NONE].poly = polygons[MISSION_NONE];        
        mission_info_vec[MISSION_PEDESTRIAN].mission_identifier = MISSION_PEDESTRIAN;
        mission_info_vec[MISSION_PEDESTRIAN].poly = polygons[MISSION_PEDESTRIAN];
        mission_info_vec[MISSION_TRAFFIC_SIGN].mission_identifier = MISSION_TRAFFIC_SIGN;
        mission_info_vec[MISSION_TRAFFIC_SIGN].poly = polygons[MISSION_TRAFFIC_SIGN];
        mission_info_vec[MISSION_ACCIDENT_VEHICLE].mission_identifier = MISSION_ACCIDENT_VEHICLE;
        mission_info_vec[MISSION_ACCIDENT_VEHICLE].poly = polygons[MISSION_ACCIDENT_VEHICLE];
        mission_info_vec[MISSION_EMERGENCY_VEHICLE].mission_identifier = MISSION_EMERGENCY_VEHICLE;
        mission_info_vec[MISSION_EMERGENCY_VEHICLE].poly = polygons[MISSION_EMERGENCY_VEHICLE];
        
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_NONE]                 = 0;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_PEDESTRIAN]           = 1;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_TRAFFIC_SIGN]         = 3;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_ACCIDENT_VEHICLE]     = 4;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_EMERGENCY_VEHICLE]    = 5;

        cur_mission = mission_info_vec[MISSION_PEDESTRIAN];
        printf("current mission : %d", 
            MISSION_IDENTIFIER_TO_NUMBER[cur_mission.mission_identifier]);

        //start avc interface 
        AVCInterface::getInstancePtr();

        //ok code
        ndt_ok = false;
        sensing_ok = false;
        std::thread([&](){
            ros::Publisher ok_pub = nh.advertise<std_msgs::Bool>("object_manager_ok", 10);
            ros::Rate loop_rate(10);
            std_msgs::Bool ok;
            while(ros::ok()){
                ok.data = ndt_ok & sensing_ok;
                ok_pub.publish(ok);
		        if (ok.data)	break;
                loop_rate.sleep();
            }
        }).detach();
        printf("constructor end\n");
    }

    void combinedSub(int identifier, const object_manager_msgs::combinedConstPtr& ptr){
        if (identifier == 0) sensing_ok = true;
        AVCInterface::getInstancePtr()->lock();
        mission_handler.run(
            *AVCInterface::getInstancePtr(),
            cur_mission.mission_identifier, 
            *ptr
        );
        AVCInterface::getInstancePtr()->unlock();

        //objInfoVec[identifier] = *ptr;
        //out_bounding_boxes_.header = ptr->header;
        //publishDebuggingMsgs();
    }

    void object_publish(){
        out_bounding_boxes_.boxes.clear();
        out_bounding_boxes_.boxes.resize(0);

        for(size_t i = 0; i < objInfoVec.size(); ++i){
            if(!use_flags[i]) continue;
            object_manager_msgs::combined& objInfo = objInfoVec[i];
            for(size_t j = 0; j < objInfo.bb_ary.boxes.size(); ++j){
                out_bounding_boxes_.boxes.push_back(objInfo.bb_ary.boxes[j]);
            }
        }
        bounding_box_ary_pub.publish(out_bounding_boxes_);        
    }

    void param_callback(object_manager::ObjectManagerConfig &config, uint32_t level){
        use_flags[vision_idx] = config.vision_flg;
        use_flags[spat_idx] = config.spat_flg;
        use_flags[AMB_idx] = config.AMB_flg;
        mission_handler.set_param(config);        
    }

    void curposCB(const geometry_msgs::PoseStampedConstPtr& ptr){
        ndt_ok = true;
        static int seq = 0;
        curpos = *ptr;
        bool cur_mission_found = false;
        //find cur mission area. If mission changed, initialize estop and objects states.

        AVCInterface::getInstancePtr()->lock();
        AVCInterface::getInstancePtr()->setCurpos(curpos);
        AVCInterface::getInstancePtr()->publish();
        AVCInterface::getInstancePtr()->unlock();
        for(auto& mission_info : mission_info_vec){
            if (true == 
                isPointInPolygon(curpos.pose.position.x, curpos.pose.position.y, mission_info.poly.polygon))
            {
                cur_mission_found = true;
                if (cur_mission.mission_identifier != mission_info.mission_identifier){
                    cur_mission = mission_info;
                    AVCInterface::getInstancePtr()->lock();
                    AVCInterface::getInstancePtr()->initializeEstopAndObstacles();
                    AVCInterface::getInstancePtr()->unlock();
                    printf("[MISSION %d] Handler on!", 
                        MISSION_IDENTIFIER_TO_NUMBER[cur_mission.mission_identifier]);
                    break;
                }
            }
        }
        if (!cur_mission_found) cur_mission = mission_info_vec[MISSION_NONE];

        
        //publish debug polygon
        cur_mission.poly.header.stamp = ros::Time::now();
        cur_mission.poly.header.frame_id = "map";
        cur_mission.poly.header.seq = seq++;
        cur_mission_poly_pub.publish(cur_mission.poly);
    }

    void Run(){
        ros::spin();
    }

    void publishDebuggingMsgs(){
        object_publish();   
    }

private:
    //estop dependent
    ros::Publisher estop_pub;
    bool estop;

    //autoware obstacles dependent
    ros::Publisher autoware_obstacle_pub;
    autoware_msgs::DetectedObjectArray autoware_obstacles;    

    //fundamental members
    ros::NodeHandle nh, private_nh;
    std::vector<object_manager_msgs::combined> objInfoVec;
    std::vector<ros::Subscriber> subVec;
    std::vector<bool> use_flags;
    int vision_idx, spat_idx, AMB_idx;
    ros::Publisher detected_object_ary_pub;
    ros::Publisher bounding_box_ary_pub;
    ros::Subscriber curposSub;
    jsk_recognition_msgs::BoundingBoxArray out_bounding_boxes_;
    dynamic_reconfigure::Server<object_manager::ObjectManagerConfig> dyn_param_server;
    dynamic_reconfigure::Server<object_manager::ObjectManagerConfig>::CallbackType dyn_param_cb;

    //mission dependent
    MissionHandler mission_handler;
    geometry_msgs::PoseStamped curpos;
    std::vector<MissionInfo> mission_info_vec;
    MissionInfo cur_mission;
    ros::Publisher cur_mission_poly_pub;
    
    AVCInterface* avc_interface_ptr;
    std::map<int, int> MISSION_IDENTIFIER_TO_NUMBER;

    //ok members
    bool ndt_ok, sensing_ok;
    ros::Publisher ok_pub;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "object_manager");
    ObjectManager object_manager;
    object_manager.Run();

    return 0;
}
