#include <ros/ros.h>
#include <v2x_msgs/v2x_info.h>
#include <v2x_processor/spat.h>
#include <v2x_processor/bsm.h>
#include <v2x_processor/tim.h>
#include <vector>
#include <v2x_processor/tf_mat.h>

class V2xProcessor{
public:
    V2xProcessor() : 
        spat_processor(nh, "spat_info_combined"),
        bsm_processor(nh, "bsm_info_combined"),
        tim_processor(nh, "tim_info_combined"),
        private_nh("~")
    {
        std::vector<double> tf_mat;
        if (!private_nh.getParam("tf_mat", tf_mat))
            throw std::runtime_error("set tf_mat");
        TransformationMatrixProcessor tf(tf_mat);

        tim_processor.setTfProcessor(tf);
        bsm_processor.setTfProcessor(tf);

        //v2x msg processing
        v2x_sub = nh.subscribe("v2x_info", 10, &V2xProcessor::v2xSub, this);
    }

    void v2xSub(const v2x_msgs::v2x_infoConstPtr& ptr){
        switch(ptr->msg_type){
        case v2x_msgs::v2x_info::BSM_MSG_TYPE:
            bsm_processor.processBSM(ptr); break;
        case v2x_msgs::v2x_info::SPAT_MSG_TYPE:
            spat_processor.processSPaT(ptr); break;
        case v2x_msgs::v2x_info::TIM_MSG_TYPE:
            tim_processor.processTIM(ptr); break;
        default: break;
        }
    }
private:
    ros::Subscriber v2x_sub;
    ros::NodeHandle nh, private_nh;
    SPaTProcessor spat_processor;
    BSMProcessor bsm_processor;
    TIMProcessor tim_processor;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "v2x_processor_main");
    V2xProcessor v;
    ros::spin();
}