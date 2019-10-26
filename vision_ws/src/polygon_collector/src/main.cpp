#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <fstream>
#include <signal.h>
#include <string>

class PointCollector{
public:
    PointCollector(std::string save_dir){
        pose_sub = nh.subscribe("/move_base_simple/goal", 10, &PointCollector::poseCB, this);
        polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("polygon_received", 10);
        this->save_dir = (save_dir.back() == '/') ? save_dir : save_dir + '/';
    }

    ~PointCollector(){
        std::string name;
        std::cout << "name of polygon : ";
        std::cin >> name;
        
        std::ofstream ofs(save_dir + name); 
        
        ofs << name << ": [";
        if (pose.size()){
            ofs << *pose.begin();
            for(auto it = pose.begin() + 1; it != pose.end(); ++it)
                ofs << ", " << *it;
        }
        ofs << "]\n";        
        ofs.close();
    }

    void poseCB(const geometry_msgs::PoseStampedConstPtr& ptr){
        pose.push_back(ptr->pose.position.x);
        pose.push_back(ptr->pose.position.y);

        polygon_received.header.seq++;
        polygon_received.header.stamp = ros::Time::now();
        polygon_received.header.frame_id = "map";
        
        polygon_received.polygon.points.emplace_back();
        polygon_received.polygon.points.back().x = ptr->pose.position.x;
        polygon_received.polygon.points.back().y = ptr->pose.position.y;
        polygon_received.polygon.points.back().z = ptr->pose.position.z;

        polygon_pub.publish(polygon_received);
    }
private:
    std::string save_dir;
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Publisher polygon_pub;
    std::vector<double> pose;//x, y
    geometry_msgs::PolygonStamped polygon_received;
}* ptr;

void mySigintHandler(int sig)
{
    delete ptr;
    ros::shutdown();
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "main");
    ptr = new PointCollector("/tmp");
    signal(SIGINT, mySigintHandler);
    ros::spin();

}
