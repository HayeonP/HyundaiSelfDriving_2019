#pragma once
#include <object_manager/mission_processor.h>
#include <object_manager/genAutowareObstacles.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

static constexpr double ELAPSED_TIME_TOLERANCE = 0.01;//second
static constexpr double CLOST_DISTANCE_TOLERANCE = 3;//meter
struct Obstacle{
    Obstacle(ros::Time t, std::string n, geometry_msgs::Polygon poly, geometry_msgs::Pose pose):
        detected_time(t), name(n), polygon(poly), pose(pose) {}
    
    double getElapsedTime(ros::Time t){
        return (t - detected_time).toSec();
    }
    double getDistance(double x, double y){
        double x_cur = pose.position.x;
        double y_cur = pose.position.y;
        return std::sqrt((x-x_cur)*(x-x_cur) + (y-y_cur)*(y-y_cur));
    }
    const std::string& getName() const{
        return name;
    }
    const geometry_msgs::Polygon getPolygon() const{
        return polygon;
    }

    ros::Time detected_time;
    std::string name;
    geometry_msgs::Polygon polygon;
    geometry_msgs::Pose pose;
};

class AccidentVehilceProcessor : public MissionProcessor{
public:
    AccidentVehilceProcessor(){
        car_id = 0;
    }
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){
        auto &detectedObj = msg.obj_ary;
        
        //process only camera msg
        bool isCarMsg = false;

        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if(it->label == "car") {
                isCarMsg = true;
                break;
            }
        }        

        if(!isCarMsg) 
            return;

        //collect car detectObject instance
        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if(it->label == "car" && it->convex_hull.polygon.points.size()) {
                autoware_msgs::DetectedObject d;
                genObstacleMsgFromPolygon(d, it->convex_hull.polygon, 0.5);
                d.label = it->label;
                // erase these lines after fixing camera bug
                if (!d.convex_hull.polygon.points.size()) {
                    continue;
                } 
                // erase end 
                std::string n = "car" + std::to_string(car_id);
                //find objects center position in map frame
                geometry_msgs::PointStamped p_velodyne, p_map;
                p_velodyne.header.frame_id = "velodyne";//hard coded
                //p_velodyne.header.stamp = ros::Time::now();
                p_velodyne.point.x = it->pose.position.x;
                p_velodyne.point.y = it->pose.position.y;
                try
                {
                    transform_listener.transformPoint("map", p_velodyne, p_map);
                }
                catch (tf::TransformException &ex){
                    ROS_ERROR("map-velodyne tf does not exist");
                    for(auto&& o : Obstacles_keep)
                        avc_interface.eraseObstacle(o.getName());
                    return;
                }
                
                geometry_msgs::Pose pose;
                pose.position.x = p_map.point.x;
                pose.position.y = p_map.point.y;
                
                //keep in memory to erase later 
                avc_interface.setObstacle(n, d);
               
                Obstacles_keep.emplace_back(ros::Time::now(), n, d.convex_hull.polygon, pose);
                car_id = (car_id + 1) % 10000000;//iteration
            }
        }

        //erase too far or old obstacle
        std::vector<std::string> eraseTargets;
        double x_vehicle = avc_interface.getCurpos().pose.position.x;
        double y_vehicle = avc_interface.getCurpos().pose.position.y;
        ros::Time t = ros::Time::now();
        
        for(auto it = Obstacles_keep.begin(); it != Obstacles_keep.end();){
            if ((it->getElapsedTime(t) > ELAPSED_TIME_TOLERANCE) &&
                (it->getDistance(x_vehicle, y_vehicle) > CLOST_DISTANCE_TOLERANCE)){
                eraseTargets.push_back(it->getName());
                it = Obstacles_keep.erase(it);
            }
            else it++;
        }

        for(auto&& n : eraseTargets)
            avc_interface.eraseObstacle(n);
        
        avc_interface.publish();
    }

    
    virtual void set_param(object_manager::ObjectManagerConfig &config)
    { 
    }
private:
    ros::NodeHandle nh;
    std::deque<Obstacle> Obstacles_keep;
    size_t car_id;
    tf::TransformListener transform_listener;
};
