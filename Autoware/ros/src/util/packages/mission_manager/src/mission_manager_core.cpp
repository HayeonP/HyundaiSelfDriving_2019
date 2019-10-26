#include "mission_manager/mission_manager.h"

namespace mManagerNS
{ 

 mManager::mManager()
 {
	bVMap = false;//enable when loading vector map.
	bMatching = false;//enable when starting mm, disable when localizing completely.
	bGplan = false;//enable when localizing completely, disable after global planning completely.
	bLplan = false;//enable when localizing completely, disable after global planning completely.
	bSensing = false;//enable when sensing completely.
	bkill = false;//enable when killing nodes.
	bpedestrian = false;//enable when appear pedestrian and disappear pedestrian.
	temp_pedestrian = false;//enable when appear pedestrian and disappear pedestrian.
	bloadfirst = true;
	missionNo = 1;
	profilingNo = 1;
	pub_initial_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
	pub_goal_polygon = nh.advertise<geometry_msgs::PolygonStamped>("/goal_polygon", 10);
	pub_goal_pose = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	pub_pf_polygon = nh.advertise<geometry_msgs::PolygonStamped>("/pf_polygon", 10);
	pub_pf_velocity = nh.advertise<std_msgs::Float32>("/mission_manager_maxvel", 10);
	pub_ld = nh.advertise<autoware_config_msgs::ConfigWaypointFollower>("/config/waypoint_follower", 10);
	pub_estop_polygon = nh.advertise<geometry_msgs::PolygonStamped>("/estop_polygon", 10);
	pub_estop = nh.advertise<std_msgs::Bool>("/mission_estop", 10);
	sub_current_pose = nh.subscribe("/current_pose", 10, &mManager::callbackGetCurrentPose, this);
	sub_vector_map_stat = nh.subscribe("/vector_map_loader_ok", 10, &mManager::callbackManageVectorMapStat, this);
	sub_points_map_stat = nh.subscribe("/points_map_loader_ok", 10, &mManager::callbackManagePointsMapStat, this);
	sub_matching_stat = nh.subscribe("/ndt_matching_ok", 10, &mManager::callbackManageMatchingStat, this);
	sub_gplanning_stat = nh.subscribe("/op_global_planner_ok", 10, &mManager::callbackManageGPlanningStat, this);
	sub_lplanning_stat = nh.subscribe("/pure_pursuit_ok", 10, &mManager::callbackManageLPlanningStat, this);
	sub_sensing_stat = nh.subscribe("/camera_ok", 10, &mManager::callbackManageSensingStat, this);
	sub_detection_stat = nh.subscribe("/object_manager_ok", 10, &mManager::callbackManageDetectionStat, this);
	sub_v2x_workzone = nh.subscribe("/workzone_blocked_paths", 10, &mManager::callbackSelectVectormap, this); //ICHTHUS:avoid_workzone_given    _by_Hyundai AVC
//	sub_pedestrian = nh.subscribe("/estop_mission1", 10, &mManager::callbackPedestrian, this);

	initialpose_msg.header.frame_id = "map";
	initialpose_msg.pose.pose.position.z = 0;
	initialpose_msg.pose.pose.orientation.x = 0;
	initialpose_msg.pose.pose.orientation.y = 0;
	goalpose_msg.header.frame_id = "map";
	goalpose_msg.pose.position.z = 0;
	goalpose_msg.pose.orientation.x = 0;
	goalpose_msg.pose.orientation.y = 0;
	
 	estoppolygon_msg.header.frame_id = "world";
 	estoppolygon_msg.polygon.points.resize(4);
 	pfpolygon_msg.header.frame_id = "world";
 	pfpolygon_msg.polygon.points.resize(4);
 	goalpolygon_msg.header.frame_id = "world";
 	goalpolygon_msg.polygon.points.resize(4);
 }

 mManager::~mManager()
 {
 }

 void mManager::cleanup()
 {
	//shutdown script
	int stat = system("/home/autoware/Autoware/ros/src/util/packages/mission_manager/script/shutdown");
	//find pid with script
	FILE *fp = NULL;
	char filename[NODECNT][1024] = {"pure_pursuit","vel_relay", "pose_relay", "twist_filter", "wf_simulator", "op_behavior_selector", "op_motion_predictor", "op_trajectory_evaluator", "op_trajectory_generator", "op_common_params", "op_global_planner"};
	char command[1024] = "/home/autoware/Autoware/ros/src/util/packages/mission_manager/script/find_pid ";
	char* cmd_ptr;
	char line[1024];
	int pid;

	for(int i = 0; i< NODECNT; i++)
	{
		cout << filename[i] <<endl;
		cmd_ptr = strcat(command, filename[i]);
		if((fp = popen(cmd_ptr, "r")) == NULL){
			ROS_ERROR("Can't open pipe {find_pid}");
			return ;
		}
		bkill = true;
		while(bkill)
		{
			bkill = false;
			while(fgets(line, 1024, fp) != NULL)
			{
				printf("%s\n", line);
			}
			if( strcmp(line, "None\n") != 0)
			{
				//ROS_WARN("%s : not killed yet, killing..",filename[i]);
				printf("%s : not killed yet, killing..\n",filename[i]);
				bkill = true;
				pid = atoi(line);
				kill(pid, 9);
			}
		}
		pclose(fp); 
	}
 }


 void mManager::MatchEstopArea(double* curr)
 {
	int margin = 5;
	double cur_goal_x, cur_goal_y;
	cur_goal_x = estop_pose[0];
	cur_goal_y = estop_pose[1];
	
	int lu_x = cur_goal_x - margin;
	int lu_y = cur_goal_y - margin;
	int rd_x = cur_goal_x + margin;
	int rd_y = cur_goal_y + margin;
	int z = 0.988161278591;
	
	bool bNeedEstop = (lu_x) <= curr[0] 
		    && (rd_x) >= curr[0] 
		    && (lu_y) <= curr[1]
	 	    && (rd_y) >= curr[1];
	
	estoppolygon_msg.polygon.points[0].x = lu_x;
	estoppolygon_msg.polygon.points[0].y = rd_y;
	estoppolygon_msg.polygon.points[0].z = z;
 	estoppolygon_msg.polygon.points[1].x = lu_x;
	estoppolygon_msg.polygon.points[1].y = lu_y;
	estoppolygon_msg.polygon.points[1].z = z;
	estoppolygon_msg.polygon.points[2].x = rd_x;
	estoppolygon_msg.polygon.points[2].y = lu_y;
	estoppolygon_msg.polygon.points[2].z = z;
	estoppolygon_msg.polygon.points[3].x = rd_x;
	estoppolygon_msg.polygon.points[3].y = rd_y;
	estoppolygon_msg.polygon.points[3].z = z;

	pub_estop_polygon.publish(estoppolygon_msg);
	if(bNeedEstop)
	{
		//ESTOP publish
		estop_msg.data = true;
		pub_estop.publish(estop_msg);
		int stat = system("/home/autoware/Autoware/ros/src/util/packages/mission_manager/script/kill_visionlab_detection.sh");
		cleanup();
		cout << "end!!"<<endl;
		//ESTOP publish
		estop_msg.data = false;
		pub_estop.publish(estop_msg);
		exit(0);
	}
 }

 void mManager::MatchProfilingArea(double* curr)
 {
	int margin_x = 10;
	int margin_y = 10;
	double cur_goal_x, cur_goal_y;
	cur_goal_x = pf_pose[profilingNo-1][0];
	cur_goal_y = pf_pose[profilingNo-1][1];

	int lu_x = cur_goal_x - margin_x;
	int lu_y = cur_goal_y - margin_y;
	int rd_x = cur_goal_x + margin_x;
	int rd_y = cur_goal_y + margin_y;

	int z = 0.988161278591;
	
	bool bNeedPf = (lu_x) <= curr[0] 
		    && (rd_x) >= curr[0] 
		    && (lu_y) <= curr[1]
	 	    && (rd_y) >= curr[1];

	//ax + b = y
	//a(lu_x - rd_x) + b < lu_y - rd_y
	//lu_x, rd_y 
	//rd_x, rd_y
	//------------------------
	//a(lu_x - rd_x) + lu_y - rd_y > 0
	//lu_x, luy
	//rd_x, lu_y

	if( profilingNo == 3) 
	{
		float l_x = 117.247840881;
		float l_y = -199.522705078;
		float r_x = 167.695526123;
		float r_y = -111.808387756;
		float a = (l_y - r_y) /(l_x - r_x) ;
		float b = l_y - (a * l_x);
	
		if((a*curr[0] + b- curr[1]) > 0) bNeedPf = false;
		else if((a*curr[0] + b- curr[1]) < 0) bNeedPf = true;

 		pfpolygon_msg.polygon.points.resize(2);
		pfpolygon_msg.polygon.points[0].x = l_x;
		pfpolygon_msg.polygon.points[0].y = l_y;
		pfpolygon_msg.polygon.points[0].z = z;
	 	pfpolygon_msg.polygon.points[1].x = r_x;
		pfpolygon_msg.polygon.points[1].y = r_y;
		pfpolygon_msg.polygon.points[1].z = z;
	}
	else
	{	
 	pfpolygon_msg.polygon.points.resize(4);
	pfpolygon_msg.polygon.points[0].x = lu_x;
	pfpolygon_msg.polygon.points[0].y = rd_y;
	pfpolygon_msg.polygon.points[0].z = z;
 	pfpolygon_msg.polygon.points[1].x = lu_x;
	pfpolygon_msg.polygon.points[1].y = lu_y;
	pfpolygon_msg.polygon.points[1].z = z;
	pfpolygon_msg.polygon.points[2].x = rd_x;
	pfpolygon_msg.polygon.points[2].y = lu_y;
	pfpolygon_msg.polygon.points[2].z = z;
	pfpolygon_msg.polygon.points[3].x = rd_x;
	pfpolygon_msg.polygon.points[3].y = rd_y;
	pfpolygon_msg.polygon.points[3].z = z;
	}
	pub_pf_polygon.publish(pfpolygon_msg);
	
	if(bNeedPf)
	{
		if(profilingNo <= MAX_PF)
		{
			ld_msg.param_flag = 0;
			ld_msg.lookahead_ratio = ld[profilingNo-1];
			ld_msg.minimum_lookahead_distance = 6.0;
			pub_ld.publish(ld_msg); //pub ld
			pfvelocity_msg.data = pf_vel[profilingNo-1];
			pub_pf_velocity.publish(pfvelocity_msg); //pub max vel
		}
		bNeedPf = false;
		profilingNo++;
	}
 }


 void mManager::MatchMissionGoalArea(double* curr)
 {
	int margin = 1;
	double cur_goal_x, cur_goal_y;
	double* goal;	
	if(missionNo > MAX_MISSION)
	{
		cout<<"end!!"<<endl;
		cleanup();
		exit(0);
	}
	goal = goal_pose[missionNo];
	cur_goal_x = goal_pose[missionNo - 1][0];
	cur_goal_y = goal_pose[missionNo - 1][1];
	
	int lu_x = cur_goal_x - margin;
	int lu_y = cur_goal_y - margin;
	int rd_x = cur_goal_x + margin;
	int rd_y = cur_goal_y + margin;
	int z = 0.988161278591;
	
	bool bIsGoal = (lu_x) <= curr[0] 
		    && (rd_x) >= curr[0] 
		    && (lu_y) <= curr[1]
	 	    && (rd_y) >= curr[1];
	
	goalpolygon_msg.polygon.points[0].x = lu_x;
	goalpolygon_msg.polygon.points[0].y = rd_y;
	goalpolygon_msg.polygon.points[0].z = z;
 	goalpolygon_msg.polygon.points[1].x = lu_x;
	goalpolygon_msg.polygon.points[1].y = lu_y;
	goalpolygon_msg.polygon.points[1].z = z;
	goalpolygon_msg.polygon.points[2].x = rd_x;
	goalpolygon_msg.polygon.points[2].y = lu_y;
	goalpolygon_msg.polygon.points[2].z = z;
	goalpolygon_msg.polygon.points[3].x = rd_x;
	goalpolygon_msg.polygon.points[3].y = rd_y;
	goalpolygon_msg.polygon.points[3].z = z;

	pub_goal_polygon.publish(goalpolygon_msg);
	if(bIsGoal)
	{
		if(missionNo != MAX_MISSION)
		{
		 settingPoseMsg(curr, goal);
		 bMatching = true;
		 cleanup();
		}
		bIsGoal = false;
		missionNo++;
	}
 }

 void mManager::settingPoseMsg(double* init, double* goal)
 {
	initialpose_msg.pose.pose.position.x = init[0];
	initialpose_msg.pose.pose.position.y = init[1];
	initialpose_msg.pose.pose.orientation.z = init[2];
	initialpose_msg.pose.pose.orientation.w = init[3];
 
	goalpose_msg.pose.position.x = goal[0];
	goalpose_msg.pose.position.y = goal[1];
	goalpose_msg.pose.orientation.z = goal[2];
	goalpose_msg.pose.orientation.w = goal[3];
 }


 void mManager::callbackPedestrian(const std_msgs::BoolConstPtr& msg)
 {
	bpedestrian = msg->data;
 }

 void mManager::callbackManageSensingStat(const std_msgs::BoolConstPtr& msg)
 {
	if(!bSensing)
	{
        	if(msg->data)
	        {
        	        //ROS_WARN("[Sensing] : OK");
        	        printf("[Sensing] : OK\n");
			bSensing = true;	
	        }
	        else
			printf("[Sensing] : NOT OK\n");
        	        //ROS_WARN("[Sensing] : NOT OK");
	}
 }
 void mManager::callbackManageDetectionStat(const std_msgs::BoolConstPtr& msg)
 {
        if(msg->data)
        {
                printf("[Detection] : OK\n");
                //ROS_WARN("[Detection] : OK");
        }
        else
                printf("[Detection] : NOT OK\n");
                //ROS_WARN("[Detection] : NOT OK");
 }

 void mManager::callbackManageVectorMapStat(const std_msgs::BoolConstPtr& msg)
 {
	if(msg->data)
	{
		printf("[VectorMap] : OK\n");
		//ROS_WARN("[VectorMap] : OK");
		bVMap = true;
	}
	else
		printf("[VectorMap] : NOT OK\n");
		//ROS_WARN("[VectorMap] : NOT OK");
		
 }
 void mManager::callbackManagePointsMapStat(const std_msgs::BoolConstPtr& msg)
 {
	if(msg->data)
	{
		printf("[PointsMap] : OK\n");
		//ROS_WARN("[PointsMap] : OK");
		bMatching = true;
	}
	else
		printf("[PointsMap] : NOT OK\n");
		//ROS_WARN("[PointsMap] : NOT OK");
 }
 void mManager::callbackManageMatchingStat(const std_msgs::BoolConstPtr& msg)
 {
	 if(bMatching)
	 {
		 if(msg->data)
		 {
			printf("[Matching] : OK\n");
			//ROS_WARN("[Matching] : OK");
			bMatching = false;
			bGplan = true;
			system("roslaunch mission_manager mm_mission_planning.launch &");//global planning
			if(bSensing)
			{
				system("roslaunch avc_launch visionlab_detection.launch &");//vision detection
			}
	 }
		 else
			printf("[Matching] : NOT OK\n");
			//ROS_WARN("[Matching] : NOT OK");
	 }
 }
 void mManager::callbackManageGPlanningStat(const std_msgs::BoolConstPtr& msg)
 {
	 if(bGplan && bVMap)
	 {
		 if(msg->data)
		 {
			printf("[GlobalPlanner] : OK\n");
			//ROS_WARN("[GlobalPlanner] : OK");
			system("roslaunch mission_manager mm_motion_planning.launch &");//local planning
			bGplan = false;
			bLplan = true;
		 }
		 else
		{
//			pub_initial_pose.publish(initialpose_msg); //when you drive in road, disable this line
		 	pub_goal_pose.publish(goalpose_msg);
			printf("[GlobalPlanning] : NOT OK\n");
			//ROS_WARN("[GlobalPlanning] : NOT OK");
		}
	 }

 }
 void mManager::callbackManageLPlanningStat(const std_msgs::BoolConstPtr& msg)
 {
	 if(bLplan)	
	 {
		 if(msg->data)
			{
				printf("[LocalPlanner] : OK\n");
				//ROS_WARN("[LocalPlanner] : OK");
				profilingNo--;
				if(profilingNo == 0) profilingNo = 1;	

				if(profilingNo <= MAX_PF)
				{
					ld_msg.param_flag = 0;
					ld_msg.lookahead_ratio = ld[profilingNo-1];
					ld_msg.minimum_lookahead_distance = 6.0;
					pub_ld.publish(ld_msg); //pub ld
					pfvelocity_msg.data = pf_vel[profilingNo-1];
					pub_pf_velocity.publish(pfvelocity_msg); //pub max vel
				}
				profilingNo++;
				bLplan = false;
				//ESTOP publish
				estop_msg.data = false;
				pub_estop.publish(estop_msg);
			}
		 else
			printf("[LocalPlanner] : NOT OK\n");
			//ROS_WARN("[LocalPlanner] : NOT OK");
	 }
 }
 void mManager::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
 {
	double cur_pose[4] = {msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z, msg->pose.orientation.w};

	MatchProfilingArea(cur_pose);
	MatchMissionGoalArea(cur_pose);
 	MatchEstopArea(cur_pose);
 }
 void mManager::callbackSelectVectormap(const std_msgs::Int32MultiArrayConstPtr& msg)
 {  
	//load vectormap ONLY ONCE!
	if(!bloadfirst) return;
	bloadfirst = false;

	//if(!bloadfirst && !bpedestrian) return;
	static std::vector<int> oldVectormap;
	 std::vector<int> newVectormap;
	 for(int i = 0; i < msg->data.size(); i++) {
		 newVectormap.push_back(msg->data[i]);
	 }
	 bool result = false;
	 if(oldVectormap.size() == newVectormap.size()) {
		 result = std::equal(newVectormap.begin(), newVectormap.end(), oldVectormap.begin());
	 }
	 if (result){
		 //std::cout << "Both vectors are equal" << std::endl;
		 return;
	 }

	//ESTOP publish
	estop_msg.data = true;
	pub_estop.publish(estop_msg);
	
	//CLEANNING UP//
	bGplan = false;
	bLplan = false;
	cleanup();
	//END CLEANNING//

	 oldVectormap = newVectormap;
	 printf("CHANGE Vectormap\n");
	 //ROS_WARN("CHANGE Vectormap");

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
	 for(int i = 0; i < msg->data.size(); i++){
		 workzoneID = msg->data[i] -1 ;
		 // ROS_INFO("workzone[%d] : size: %d ", workzoneID, pathArray[workzoneID-1].size());
		 for(int j = 0; j < pathArray[workzoneID].size(); j++){
			 //ROS_INFO("invalid map : %d",pathArray[workzoneID][j]);
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
		 printf("selectPath() : There is no valid VECTOR MAP\n");
		 //ROS_WARN("selectPath() : There is no valid VECTOR MAP");
	 else
		 printf("valid Map: %d\n", validVectormapIdx);
		 //ROS_WARN("valid Map: %d", validVectormapIdx);
	 std::string cmd = "roslaunch mission_manager load_vectormap";
	 cmd += std::to_string(validVectormapIdx) + ".launch &";
	 std::cout << cmd<< std::endl;
	 system(cmd.c_str());
	 printf("New Vector Map is loaded : %d\n", validVectormapIdx);
	 //ROS_WARN("New Vector Map is loaded : %d", validVectormapIdx);

	//REPLANNING//
	bGplan = true;
	system("roslaunch mission_manager mm_mission_planning.launch &");//global planning
	//END REPLANNING

 }

 void mManager::MainLoop()
 {
	 ros::Rate loop_rate(10);
	 settingPoseMsg(init_pose, goal_pose[0]);
	 cleanup();
	 ros::spin();
	 return;
 }
}
