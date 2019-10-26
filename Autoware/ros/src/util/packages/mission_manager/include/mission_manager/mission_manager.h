#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <sys/epoll.h>
#include <signal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include "mission_manager/vcs_msg.h"
#include "autoware_config_msgs/ConfigWaypointFollower.h"
#include <std_msgs/Int32MultiArray.h> //ICHTHUS:avoid_workzone_given_by_Hyundai AVC
#include <cstring> //ICHTHUS:avoid_workzone_given_by_Hyundai AVC

#define MAX_MISSION 1 
#define MAX_PF 23
#define MAX_EVENTS 10
#define BUF_SIZE 100
#define NODECNT 11
#define NumOfVectormap 9 //ICHTHUS:avoid_workzone_given_by_Hyundai AVC

using namespace std;
using namespace ros;

namespace mManagerNS
{

class mManager
{
  private:
	string message;
	int i;
	int event_cnt;
	int epfd;
	struct epoll_event ev, evs[MAX_EVENTS];

	//soongsil
/*	double init_pose[4]={16.2752189636,1.03413772583,0.0587489961962,0.998272786089};
	double estop_pose[4]={-8.67902088165,5.99792385101,-0.379988850894,0.92499106655};
	double goal_pose[MAX_MISSION][4]={
//			  {-1.41474628448, 93.0772247314, 1.0, 6.12323399574e-17},
                          {-3.62599086761, 1.58695220947, -0.197945671463, 0.980212992746},
                          };
	double pf_pose[MAX_PF][4]={
			{33.4811782837,55.8169555664,0.749566158151,0.661929433214},
			{13.4469776154,95.1792526245,-0.997255596479,0.0740356352794},
                        {-37.9331932068,68.6965942383,-0.62567204137,0.780086211035},
			{-29.7063846588,27.4634685516,-0.65085118694,0.759205329577},
                        };
	float pf_vel[MAX_PF]={3.0, 2.0, 3.0, 2.0};
	float ld[MAX_PF]= {2.0,2.0,2.0,2.0};
*/
	//kcity
	double init_pose[4]={281.854644775,-230.599258423,0.987886708224,0.155176840132};
	double estop_pose[4]={339.530700684,-219.044219971,-0.918728440249,0.394889925268};
//{350.86730957,-204.183380127,-0.841332137864,0.540518486082};
	double goal_pose[MAX_MISSION][4]={
				{216.733276367,-304.455932617,-0.936604773126,0.350387640988},
                                };

	double pf_pose[MAX_PF][4]={
			{245.829833984,-212.17149353,0.954355260276,0.298673797283},// mission 1
			{187.835327148,-177.967102051,0.942134489875,0.335235145814},//mission 2
			{132.388305664,-148.715591431,0.97367892708,0.227923993822},// mission 2-1
			{59.0589904785,-106.839149475,0.967099456437,0.254398587571},//mission 3
			{-277.451416016,-70.1518478394,0.999971308685,0.00757507799881},//s
			{-458.802856445,-171.085021973,0.999861217895,0.016659680346},//s-1
			{-469.825256348,-170.542938232,-0.99988173439,0.0153791167969},//s-2
			{-497.778411865,-170.023040771,-0.997652457906,0.0684804587696},//s-3
			{-502.631530762,-170.023040771,-0.99995195439,0.00980249514483},//circle
			{-536.760742188,-114.105499268,0.751636915755,0.659577096991},//ramp
			{-511.456512451,-22.6148033142,0.23892323251,0.971038459056},//ramp-1
			{-501.84854126,-18.2611846924,0.140808014054,0.990036920109},//ramp-2
			{-490.889434814,-14.5080633163,0.0415585913096,0.999136068555},//ramp-3
			{-482.032043457,-12.5564403534,0.0461484787429,0.998934591407},//ramp-4
			{-472.123840332,-11.3554420471,0.0589017266585,0.998263786079},//ramp-5
			{-455.48135376,-9.52487468719,-0.0782209835476,0.996936044956},//mission 4
			{-414.614929199,1.15954399109,-0.0122922736226,0.99992444715},//mission 5
                        {181.38494873,-0.38905531168,-0.0536880619841,0.998557755966},//mission 6_1
                        {202.71875,-0.38905531168,-0.163538661216,0.986536925963,},//mission 6_2
                        {214.725204468,-1.28209602833,-0.0855845960986,0.996330907335},//mission 6_3
                        {230.006149292,-3.26663541794,-0.0985374340388,0.995133344881},//mission 6_4
                        {335.959625244,-117.920593262,-0.413216411973,0.910632855149},//mission 6-1
//                        {339.717010498,-127.215171814,-0.482704833668,0.87578310303},//mission 6-2
                        {342.683380127,-133.938903809,-0.561860788089,0.827231802343},//mission 6-3
//                        {345.766235352,-140.36756897,-0.463027288391,0.886344024747},//mission 6-4
//                        {352.354156494,-147.100692749,-0.524068504308,0.851676113785},//mission 6-5
                          };
	float pf_vel[MAX_PF]= {
				6.0, //mission1
				7.0, //mission2
				7.0, //mission2-1
				9.0, //mission3
				11.0, //s
				10.0, //s-1
				9.0, //s-2
				8.0, //s-3
				7.0, //circle
				11.0, //ramp
				9.5, //ramp-1
				8.5, //ramp-2
				7.0, //ramp-3
				6.0, //ramp-4
				4.5, //ramp-5
				4.0, //mission4
				6.0, //mission5 default: 8.5
				10.0,//mission6_1
				11.0,//13.0,//mission6_2
				12.0,//14.0,//mission6_3
				13.0,//15.0,//mission6_4
				12.0,//12.0,//mission6-1
//				13.0,//mission6-2
				10.0,//mission6-3
//				11.0,//mission6-4
//				10.0,//mission6-5
				};//6: 20 9:30 11:40 14:50
	float ld[MAX_PF]= {
				2.0, //mission1
				2.3, //mission2
				1.9, //mission2-1
				1.6, //mission3
				1.6, //s
				1.6, //s-1
				1.6, //s-2
				1.6, //s-3
				1.8, //circle
				1.6, //ramp
				1.5, //ramp-1
				1.5, //ramp-2
				1.5, //ramp-3
				1.5, //ramp-4
				1.5, //ramp-5
				1.8, //mission4
				2.0, //mission5 
				1.3, //mission6_1
				1.3, //mission6_2
				1.3, //mission6_3
				1.3, //mission6_4
				1.3, //mission6-1
//				1.4, //mission6-2
				1.3, //mission6-3
//				1.4, //mission6-4
//				1.4, //mission6-5
				};

	//dasan
/*	double init_pose[4]={0,0,0,0};
        //Need to edit x,y,z,w & MAX_MISSION number
	double goal_pose[MAX_MISSION][4]={
				{0.0, 0.0, 0.0, 0.0},				
                                };
        //Need to edit x,y,z,w & MAX_PF number
	double pf_pose[MAX_PF][4]={
			{0.0, 0.0, 0.0, 0.0},
                          };
	float pf_vel[MAX_PF]={6.0};
*/
	bool bGplan;
	bool bLplan;
	bool bMatching;
	bool bVMap;
	bool bSensing;
	bool bkill;
	bool bpedestrian;
	bool temp_pedestrian;
	bool bloadfirst;
	double vertex[4];
	int missionNo;
	int profilingNo;
	int seqno;
	int validVectormapIdx = 0; //ICHTHUS:avoid_workzone_given_by_Hyundai AVC
	geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
	geometry_msgs::PoseStamped goalpose_msg;
	geometry_msgs::PolygonStamped goalpolygon_msg;
	geometry_msgs::PolygonStamped pfpolygon_msg;
	geometry_msgs::PolygonStamped estoppolygon_msg;
	std_msgs::Float32 pfvelocity_msg;
	std_msgs::Bool estop_msg;
	std_msgs::Bool obj_mgr_msg;
	autoware_config_msgs::ConfigWaypointFollower ld_msg;

	ros::Publisher pub_initial_pose;
	ros::Publisher pub_goal_pose;
	ros::Publisher pub_goal_polygon;
	ros::Publisher pub_pf_polygon;
	ros::Publisher pub_estop_polygon;
	ros::Publisher pub_pf_velocity;
	ros::Publisher pub_ld;
	ros::Publisher pub_estop;
	ros::Publisher pub_obj_mgr;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_vector_map_stat;
	ros::Subscriber sub_points_map_stat;
	ros::Subscriber sub_matching_stat;
	ros::Subscriber sub_gplanning_stat;
	ros::Subscriber sub_lplanning_stat;
	ros::Subscriber sub_sensing_stat;
	ros::Subscriber sub_detection_stat;
	ros::Subscriber sub_pedestrian;
	ros::Subscriber sub_v2x_workzone; //ICHTHUS:avoid_workzone_given_by_Hyundai AVC
	

	ros::NodeHandle nh;

	void cleanup();
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackManageVectorMapStat(const std_msgs::BoolConstPtr& msg);
	void callbackManagePointsMapStat(const std_msgs::BoolConstPtr& msg);
	void callbackManageMatchingStat(const std_msgs::BoolConstPtr& msg);
	void callbackManageGPlanningStat(const std_msgs::BoolConstPtr& msg);
	void callbackManageLPlanningStat(const std_msgs::BoolConstPtr& msg);
        void callbackManageSensingStat(const std_msgs::BoolConstPtr& msg);
        void callbackManageDetectionStat(const std_msgs::BoolConstPtr& msg);
	void callbackSelectVectormap(const std_msgs::Int32MultiArrayConstPtr& msg); //ICHTHUS:avoid_workzone_given_by_Hyundai AVC
        void callbackPedestrian(const std_msgs::BoolConstPtr& msg);
	void MatchMissionGoalArea(double* curr);
	void MatchProfilingArea(double* curr);
	void MatchEstopArea(double* curr);
	void settingPoseMsg(double* init, double* goal);
 public:
	mManager();
	~mManager();
	void MainLoop();
};

}
