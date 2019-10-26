#include<nodelet_lidar_driver/nodelet_backend.h>

int quit(char *tag){
  printf("%s error: %s (errno=%d)\n", tag, strerror(errno), errno);
  exit(1);
}

int init_timer(int msec) {
  int ret;
  int fd = -1;
  struct itimerspec timeout;
  unsigned long long missed;

  if (msec >= 1000) {
    printf("init_timer() accepts 1~999 ms.\n");
    exit(1);
  }

  if ((fd = timerfd_create(CLOCK_REALTIME, 0)) <= 0)
    quit("timerfd_create");

  //if ((ret = fcntl(fd, F_SETFL, O_NONBLOCK)) != 0)
  //  quit("fcntl");

  timeout.it_value.tv_sec = 0;
  timeout.it_value.tv_nsec = msec * 1000 * 1000;
  timeout.it_interval.tv_sec = 0; // recurring
  timeout.it_interval.tv_nsec = msec * 1000 * 1000;
  if ((ret = timerfd_settime(fd, 0, &timeout, NULL)) != 0)
    quit("timerfd_settime");

  return fd;
}

namespace LidarNodeletBack
{

  //InputCloud member function
  InputCloud::InputCloud(pose p,std::string topic,ros::NodeHandle nh)
  {
    //initialize InputCloud
    this->ps = p;
    this->topic_name = topic;
    this->transform = poseTotfmatrix();
    sub = nh.subscribe(topic_name,1,&InputCloud::cloudCallback,this);
  }

  void InputCloud::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    pcl::fromROSMsg(*input,this->inCloud);
    //std::cout << input->fields[3].name << std::endl;
    //std::cout<< inCloud.is_dense<<std::endl;
    //std::cout<< "incloud "<<inCloud.header.stamp << std::endl;
    pcl::transformPointCloud(this->inCloud,this->tfdinCloud,transform);
  }

  void InputCloud::robot_sensors()
  {
    this->inCloud;
  }
  Eigen::Matrix4f InputCloud::poseTotfmatrix()
  {
    Eigen::Matrix4f tfMatrix = Eigen::Matrix4f::Identity();
    /* method1 
     * tfMatrix(0,0) ~ tfMatrix(2,2) rotation
     * tfMatrix(3,0) ~ tfMatrix(3,2) translation
	 
     tfMatrix(0,0) = cos(pt[num].yaw)*cos(pt[num].pitch);
     tfMatrix(0,1) = cos(pt[num].yaw)*sin(pt[num].pitch)*sin(pt[num].roll) - sin(pt[num].yaw)*cos(pt[num].roll);
     tfMatrix(0,2) = cos(pt[num].yaw)*sin(pt[num].pitch)*cos(pt[num].roll) + sin(pt[num].yaw)*sin(pt[num].roll);
     tfMatrix(1,0) = sin(pt[num].yaw)*cos(pt[num].pitch);
     tfMatrix(1,1) = sin(pt[num].yaw)*sin(pt[num].pitch)*sin(pt[num].roll) + cos(pt[num].yaw)*cos(pt[num].roll);
     tfMatrix(1,2) = sin(pt[num].yaw)*sin(pt[num].pitch)*cos(pt[num].roll) - cos(pt[num].yaw)*sin(pt[num].roll);
     tfMatrix(2,0) = -sin(pt[num].pitch);
     tfMatrix(2,1) = cos(pt[num].pitch)*sin(pt[num].roll);
     tfMatrix(2,2) = cos(pt[num].pitch)*cos(pt[num].roll);

    */

    /*method2 
     */ 
    Eigen::AngleAxisd rollAngle(this->ps.roll,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(this->ps.pitch,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(this->ps.yaw,Eigen::Vector3d::UnitZ());
	
    Eigen::Quaternion<double> q =  yawAngle * pitchAngle * rollAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    for(int i = 0 ; i < 3 ; i++)
      for(int j = 0 ; j < 3 ; j++)
	tfMatrix(i,j) = rotationMatrix(i,j);
	
    //transform x,y,z (gap from virtual frame)
    tfMatrix(0,3) = this->ps.x;
    tfMatrix(1,3) = this->ps.y;
    tfMatrix(2,3) = this->ps.z;

    std::cout << "tfMatrix" << std::endl;
    std::cout << tfMatrix << std::endl;	
    return tfMatrix;

  }

  //InputCloud member function end

  //OutputCloud member function start
  OutputCloud::OutputCloud(std::string topic,std::string frame,ros::NodeHandle nh)
  {
    this->topic_name = topic;
    this->frame_id = frame;
    pub = nh.advertise<sensor_msgs::PointCloud2>(this->topic_name,1);
    this->outCloud.header.frame_id = this->frame_id;
    //std::cout << outCloud.is_dense << std::endl;
  }

  //OutputCloud member function end

  //CloudMerger Member function
  void LidarNodeletBack::mergeNpub()
  {
    /* synchronization problem
     * Time stamp of 4 sensors are apparently different.
     * velodyne drivers publish messages every 100ms(10hz)
     * Is it ok?
     */
    //  std::cout << "# of SENSORS " << this->nsensors << std::endl;
    outCl->outCloud.clear(); //clear before use. Header info is not cleared.
    // std::cout << outCl->outCloud.width <<std::endl;
    // std::cout << outCl->outCloud.header.frame_id <<std::endl;
    const pcl::uint64_t scan_dur = ceil(batchNum/753.5) * 1000; //pcl time unit : microseconds : 15ms
    pcl::uint64_t MAX_ts = 0;
    pcl::uint64_t latest_lidar_ts[this->nsensors] = {0,};
    bool lidar_valid_table[this->nsensors] = {false, };
  
    for(int i = 0 ; i < this->nsensors ; i++) {
      latest_lidar_ts[i] = inClAry[i]->tfdinCloud.header.stamp;
      if(latest_lidar_ts[i] > MAX_ts) MAX_ts = latest_lidar_ts[i];
    }
  
    for(int i = 0; i < this->nsensors ; i++){
      outCl->outCloud += inClAry[i]->tfdinCloud;
      if((MAX_ts - latest_lidar_ts[i] < scan_dur) && (MAX_ts > 0)) {
        lidar_valid_table[i] = true;
      }
      else {
	ROS_WARN("invalid value Sensor# : %d --> diff : %f", i, (MAX_ts - latest_lidar_ts[i])/1000.0);
      }
    }


    //initialize header info with first Cloud info
    outCl->outCloud.header.seq = inClAry[0]->tfdinCloud.header.seq;
    //outCl->outCloud.header.stamp = inClAry[0]->tfdinCloud.header.stamp;
    outCl->outCloud.header.stamp = ros::Time::now().toNSec()/1e3;
    //std::cout<< "outCloud time stamp : "<<outCl->outCloud.header.stamp << std::endl;
    pcl::toROSMsg(outCl->outCloud,outCl->outCloudMsg);
    outCl->pub.publish(outCl->outCloudMsg);
  }
  //CloudMerger member function end
  void LidarNodeletBack::onInit(){
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    std::string s_key("CloudIn0"); //searching key (input)
    std::string so_key("CloudOut"); //searching key (output)
    std::string key;
    this->nsensors = 0;
	
    //get all parameters
    for(int i = 1 ; i <= MAX_NSENSORS ; i++){
      //Searching key must be Cloud[1] ~ Cloud[MAX_NSENSORS]
      s_key[s_key.find((i + '0') - 1)] = i + '0';
      if(private_nh.searchParam(s_key,key)){
	std::string topic_name(s_key); //set to default
	pose cloud_pose = {0, 0, 0, 0, 0}; //set to default

	if(!private_nh.getParam(key+"/topic_name",topic_name)){
	  std::cout << "not found : "<< key +"/topic_name" << std::endl;
	}
	if(!private_nh.getParam(key+"/pose_yaw",cloud_pose.yaw)){
	  std::cout << "not found : "<< key +"/pose_yaw" << std::endl;
	}
	if(!private_nh.getParam(key+"/pose_pitch",cloud_pose.pitch)){
	  std::cout << "not found : "<< key +"/pose_pitch" << std::endl;
	}
	if(!private_nh.getParam(key+"/pose_roll",cloud_pose.roll)){
	  std::cout << "not found : "<< key +"/pose_roll" << std::endl;
	}
	if(!private_nh.getParam(key+"/pose_x",cloud_pose.x)){
	  std::cout << "not found : "<< key +"/pose_x" << std::endl;
	}
	if(!private_nh.getParam(key+"/pose_y",cloud_pose.y)){
	  std::cout << "not found : "<< key +"/pose_y" << std::endl;
	}
	if(!private_nh.getParam(key+"/pose_z",cloud_pose.z)){
	  std::cout << "not found : "<< key +"/pose_z" << std::endl;
	}
	cloud_pose.yaw = M_PI *(cloud_pose.yaw/180.0);//degree to radian
	cloud_pose.pitch = M_PI *(cloud_pose.pitch/180.0);//degree to radian
	cloud_pose.roll = M_PI *(cloud_pose.roll/180.0);//degree to radian
	std::cout << key << " info" << std::endl;
	std::cout << "(yaw,pitch,roll,x,y,z) = "
		  <<	cloud_pose.yaw << " " 
		  << cloud_pose.pitch << " " 
		  << cloud_pose.roll << " " 
		  << cloud_pose.x << " "
		  << cloud_pose.y << " "
		  << cloud_pose.z << std::endl;
	//create inputCloud object
	this->nsensors++;
	inClAry[i-1] = new InputCloud(cloud_pose,topic_name,nh);
      }
    }
    if(this->nsensors == 0) std::cout << "No input data found" << std::endl;
    //outputCloud object
    if(private_nh.searchParam(so_key,key)){
      std::string topic_name(so_key); //set to default
      std::string frame_id("default_frame_id"); //set to default
      if(!private_nh.getParam(key+"/topic_name",topic_name)){
	std::cout << "not found : "<< key +"/topic_name" << std::endl;
      }
      if(!private_nh.getParam(key+"/frame_id",frame_id)){
	std::cout << "not found : "<< key +"/frame_id" << std::endl;
      }		
      outCl = new OutputCloud(topic_name,frame_id,nh);
    }
    private_nh.getParam("batchNum", batchNum);

    int timerfd = init_timer(ceil(batchNum/753.5 * 1000));
    unsigned long long missed;
		
    // Spin
    while(ros::ok())
      {
	mergeNpub();
	ros::spinOnce();
	read(timerfd, &missed, sizeof(missed));
      }
  }

}//namespace of CloudMerger
