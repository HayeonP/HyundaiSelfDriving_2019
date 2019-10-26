#include <startpos_setter/startpos_setter.h>

namespace SPosSetter{
startpos_setter::PointArray SPosSetter::_vector_point;
startpos_setter::DTLaneArray SPosSetter::_vector_dtlane;
geometry_msgs::PoseStamped SPosSetter::_current_gnss_pose;
geometry_msgs::PoseStamped SPosSetter::_previous_gnss_pose;
std::vector<geometry_msgs::PoseWithCovarianceStamped> SPosSetter::_initialPoseArray;

bool SPosSetter::_vpose_initialized;
bool SPosSetter::_vlane_initialized;
bool SPosSetter::_gnss_initialized;
bool SPosSetter::_make_randomsample;
bool SPosSetter::_is_rsample_empty;

float SPosSetter::_radius_inner;
float SPosSetter::_radius_outer;
float SPosSetter::_gnss_threshold;
float SPosSetter::_score_threshold;
float SPosSetter::_dtlane_threshold;

int SPosSetter::_rsample_num;
int SPosSetter::_rsample_index;
int SPosSetter::_ignore_count;
int SPosSetter::_ignore_index;

SPosSetter::SPosSetter(ros::NodeHandle _nh, ros::NodeHandle _private_nh){
    _vpose_initialized = false;
    _vlane_initialized = false;
    _gnss_initialized = false;
    _is_rsample_empty = true;
    _ignore_index = 0;

    _private_nh.getParam("radius_inner", _radius_inner);
    _private_nh.getParam("radius_outer", _radius_outer);
    _private_nh.getParam("gnss_threshold", _gnss_threshold);
    _private_nh.getParam("score_threshold", _score_threshold);
    _private_nh.getParam("dtlane_threshold", _dtlane_threshold);
    _private_nh.getParam("rsample_num", _rsample_num);
    _private_nh.getParam("ignore_count", _ignore_count);

    _spose_pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
    _vpose_sub = _nh.subscribe("vector_map_info/point", 100, &SPosSetter::vmapPoseCallback, this);
    _vlane_sub = _nh.subscribe("vector_map_info/dtlane", 100, &SPosSetter::vmapLaneCallback, this);
    _ndtstat_sub = _nh.subscribe("ndt_stat", 100, &SPosSetter::ndtStatCallback, this);
    _gnss_sub = _nh.subscribe("gnss_pose", 100, &SPosSetter::gnssCallback, this);
}

std::vector<geometry_msgs::PoseWithCovarianceStamped> SPosSetter::pubStartPose(geometry_msgs::PoseStamped gnss_pose){
    float min_distance = MAX_DISTANCE, min_dir, distance;
    geometry_msgs::PoseStamped min_vector;
    geometry_msgs::Quaternion quat;
    geometry_msgs::PoseWithCovarianceStamped standard_pose;
    geometry_msgs::PoseWithCovarianceStamped random_pose;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> initialPoseArray;

    for(int i = 0; i < _vector_point.data.size(); i++){
        distance = sqrt((_vector_point.data[i].ly-gnss_pose.pose.position.x)*(_vector_point.data[i].ly-gnss_pose.pose.position.x)+(_vector_point.data[i].bx-gnss_pose.pose.position.y)*(_vector_point.data[i].bx-gnss_pose.pose.position.y)+(_vector_point.data[i].h-gnss_pose.pose.position.z)*(_vector_point.data[i].h-gnss_pose.pose.position.z));        
        if(distance < min_distance){
            min_distance = distance;
            min_dir = _vector_dtlane.data[i].dir;
            min_vector.pose.position.x = _vector_point.data[i].ly;
            min_vector.pose.position.y = _vector_point.data[i].bx;
            min_vector.pose.position.z = _vector_point.data[i].h;
        }
    }
    quat = tf::createQuaternionMsgFromYaw(min_dir);
    
    standard_pose.header = gnss_pose.header;
    if(min_distance < _dtlane_threshold){
        standard_pose.pose.pose.position = min_vector.pose.position;
        standard_pose.pose.pose.orientation = quat;
    }
    else{
        standard_pose.pose.pose.position = gnss_pose.pose.position;
        standard_pose.pose.pose.orientation = gnss_pose.pose.orientation;
    }
    standard_pose.pose.pose.orientation = quat;

    float degree, radius;
    std::random_device rn;
    std::mt19937_64 rnd(rn());
    std::uniform_real_distribution<float> nDegree(0.0f, 360.0f), fRadiusIn(0.0f, _radius_inner), fRadiusOut(_radius_inner, _radius_outer);

    initialPoseArray.push_back(standard_pose);
    random_pose = standard_pose;
    for(int i = 0; i < _rsample_num * 2 - 1; i++){
        degree = nDegree(rnd);
        if(i < _rsample_num - 1) radius = fRadiusIn(rnd);
        else radius = fRadiusOut(rnd);

        random_pose.pose.pose.position.x = standard_pose.pose.pose.position.x + radius * cos(degree * M_PI / 180.0);
        random_pose.pose.pose.position.y = standard_pose.pose.pose.position.y + radius * sin(degree * M_PI / 180.0);

        initialPoseArray.push_back(random_pose);
    }
    _previous_gnss_pose = gnss_pose;

    return initialPoseArray;
}

void SPosSetter::ndtStatCallback(const startpos_setter::NDTStat::ConstPtr& msg){
    if(!_vpose_initialized || !_vlane_initialized || msg->score < _score_threshold) return;
    if(_ignore_index == 0){
        if(sqrt((_current_gnss_pose.pose.position.x-_previous_gnss_pose.pose.position.x)*(_current_gnss_pose.pose.position.x-_previous_gnss_pose.pose.position.x)+(_current_gnss_pose.pose.position.y-_previous_gnss_pose.pose.position.y)*(_current_gnss_pose.pose.position.y-_previous_gnss_pose.pose.position.y)) > _gnss_threshold)
            _make_randomsample = true;
        if(_make_randomsample || _is_rsample_empty){
            _initialPoseArray = pubStartPose(_current_gnss_pose);
            _is_rsample_empty = false;
            _make_randomsample = false;
            _rsample_index = 0;
        }

        if(_rsample_index < _rsample_num)
            ROS_ERROR("[1] 0 < RADIUS < %f, index : %d", _radius_inner, _rsample_index);
        else
            ROS_ERROR("[2] %f < RADIUS < %f, index : %d", _radius_inner, _radius_outer, _rsample_index % _rsample_num);

        _spose_pub.publish(_initialPoseArray[_rsample_index]);
        _rsample_index++;

        if(_rsample_index == _rsample_num * 2)
            _is_rsample_empty = true;
    }
    _ignore_index++;
    if(_ignore_index == _ignore_count)
        _ignore_index = 0;
}

void SPosSetter::vmapPoseCallback(const startpos_setter::PointArray& msg){
    _vector_point = msg;
    _vpose_initialized = true;
}

void SPosSetter::vmapLaneCallback(const startpos_setter::DTLaneArray& msg){
    _vector_dtlane = msg;
    _vlane_initialized = true;
}

void SPosSetter::gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(!_gnss_initialized){
        _gnss_initialized = true;
        _previous_gnss_pose = *msg;
    }
    _current_gnss_pose = *msg;
}
};

int main(int argc, char** argv){
    ros::init (argc, argv, "startpos_setter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    SPosSetter::SPosSetter* pubStartPos = new SPosSetter::SPosSetter(nh, private_nh);
    ros::spin();

    return 0;
}
