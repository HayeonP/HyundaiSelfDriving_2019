#include <ros/ros.h>
#include <v2x_processor/spat.h>
#include <v2x_processor/tf_mat.h>
#include <tf/transform_broadcaster.h>



SPaTProcessor::SPaTProcessor(ros::NodeHandle& nh, std::string combined_topic_name):
    private_nh("~"){
    combined_msg_pub = nh.advertise<object_manager_msgs::combined>(
        combined_topic_name, 10
    );
    
    x_spat.resize(3); y_spat.resize(3);
    if (!private_nh.getParam("x_spat", x_spat))
        throw std::runtime_error("set x_spat!");
    if (!private_nh.getParam("y_spat", y_spat))
        throw std::runtime_error("set y_spat!");
    if (!private_nh.getParam("signal_1_idx", signal_1_idx))
        throw std::runtime_error("set signal_1_idx!");
    if (!private_nh.getParam("signal_2_idx", signal_2_idx))
        throw std::runtime_error("set signal_2_idx!");
    if (!private_nh.getParam("signal_3_idx", signal_3_idx))
        throw std::runtime_error("set signal_3_idx!");
    ROS_INFO("signal 1, 2, 3 idx : %d %d %d", signal_1_idx,signal_2_idx,signal_3_idx);

    spat_pub_debug = nh.advertise<visualization_msgs::MarkerArray>(
        "spat_debug", 10
    );
    markAry.markers.resize(N_SPAT_ITEMS * 4);
    //We use only 3 signals

    boundingBoxAry.boxes.resize(3);
    detectedObjAry.objects.resize(3);
}

void SPaTProcessor::processSPaT(const v2x_msgs::v2x_infoConstPtr& ptr){
    boundingBoxAry.header.seq++;
    boundingBoxAry.header.stamp = ros::Time::now();
    boundingBoxAry.header.frame_id = "map";

    detectedObjAry.header.seq++;
    detectedObjAry.header.stamp = ros::Time::now();
    detectedObjAry.header.frame_id = "map";
    
    //3 signal positions in particular map. The map version for below coordinate is 
    //    final_0_2.pcd
    //double x[] = { -45, -155, -285};
    //double y[] = {-70, -70, -70};
    for(int i = 0; i < 3; ++i){
        auto& box = boundingBoxAry.boxes[i];
        box.header.seq++;
        box.header.stamp = ros::Time::now();
        box.header.frame_id = "map";

        box.pose.position.x = x_spat[i];
        box.pose.position.y = y_spat[i];
        box.pose.position.z = 0;
        box.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        box.dimensions.x = 4.0;
        box.dimensions.y = 40;
        box.dimensions.z = 3;

        autoware_msgs::DetectedObject& obj = detectedObjAry.objects[i];
        obj.header.seq++;
        obj.header.stamp = ros::Time::now();
        obj.header.frame_id = "map";

        obj.label = "signal_" + std::to_string(i);
        obj.pose = box.pose;
        obj.dimensions = box.dimensions;
    }

    switch(ptr->spat_id_region){
    case 400: //if red, the value is 3, then I'll send 1, else 0.
        boundingBoxAry.boxes[0].label = get_signal_label(
            ptr->spat_eventstate[signal_1_idx]
        ); //red, green, yellow : 0, 1, 2
        boundingBoxAry.boxes[0].value = ptr->spat_minendtime[signal_1_idx] / 10.0;//particular index of kcity
        detectedObjAry.objects[0].indicator_state = get_signal_label(
            ptr->spat_eventstate[signal_1_idx]
        ); //red, green, yellow : 0, 1, 2
        detectedObjAry.objects[0].score = ptr->spat_minendtime[signal_1_idx] / 10.0;
        break;
    case 500:
        boundingBoxAry.boxes[1].label = get_signal_label(
            ptr->spat_eventstate[signal_2_idx]
        ); //red, green, yellow : 0, 1, 2
        boundingBoxAry.boxes[1].value = ptr->spat_minendtime[signal_2_idx] / 10.0;//particular index of kcity
        detectedObjAry.objects[1].indicator_state = get_signal_label(
            ptr->spat_eventstate[signal_2_idx]
        ); //red, green, yellow : 0, 1, 2
        detectedObjAry.objects[1].score = ptr->spat_minendtime[signal_2_idx] / 10.0;
        break;
    case 700:
        boundingBoxAry.boxes[2].label = get_signal_label(
            ptr->spat_eventstate[signal_3_idx]
        ); //red, green, yellow : 0, 1, 2
        boundingBoxAry.boxes[2].value = ptr->spat_minendtime[signal_3_idx] / 10.0;//particular index of kcity
        detectedObjAry.objects[2].indicator_state = get_signal_label(
            ptr->spat_eventstate[signal_3_idx]
        ); //red, green, yellow : 0, 1, 2
        detectedObjAry.objects[2].score = ptr->spat_minendtime[signal_3_idx] / 10.0;
        break;
    }
    convertMap2Baselink_boundingbox(boundingBoxAry);
    convertMap2Baselink_detectedobj(detectedObjAry);
    publish();
}

void SPaTProcessor::debugSPaT(const v2x_msgs::v2x_infoConstPtr& ptr){
    static SPaTCoordinate *coor_ptr = nullptr;   
    ROS_INFO("process spat");

    //we use only 400, 500, 700
    int marker_start_idx = 0;
    switch(ptr->spat_id_region){
    case 0: break;
    case 400:
        marker_start_idx = N_SPAT_ITEMS * 3;
        coor_ptr = &S1; break;
    case 500:
        marker_start_idx = N_SPAT_ITEMS * 2;
        coor_ptr = &S2; break;
    case 700:
        marker_start_idx = N_SPAT_ITEMS * 1;
        coor_ptr = &S3; break;
    case 900:
        marker_start_idx = N_SPAT_ITEMS * 0;
        coor_ptr = &S9; break;
    default:
        //ROS_ERROR("id of not interest : %d", ptr->spat_id_region);
        break;
    }

    if (coor_ptr){
        for(int i = 0 ; i < N_SPAT_ITEMS; ++i){
            visualization_msgs::Marker& mark = markAry.markers[marker_start_idx + i];
            
            mark.header.frame_id = "v2x_spat";
            mark.header.stamp = ros::Time::now();
            mark.ns = "v2x_spat";
            mark.id = ptr->spat_id_region + i;

            mark.type = visualization_msgs::Marker::SPHERE;
            mark.action = visualization_msgs::Marker::ADD;
            mark.pose.position.x = coor_ptr->getX()[i];
            mark.pose.position.y = coor_ptr->getY()[i];
            mark.pose.position.z = 0;

            mark.pose.orientation.x = 0.0;
            mark.pose.orientation.y = 0.0;
            mark.pose.orientation.z = 0.0;
            mark.pose.orientation.w = 1.0;
            mark.scale.x = 0.5;
            mark.scale.y = 0.5;
            mark.scale.z = 0.5;

            double r=0, g=0, b=0, a=0;
            switch(ptr->spat_eventstate[i]){
            case 3: r=1; g=0; b=0; a=1; break;//red light
            case 5: r=0; g=1; b=0; a=1; break;
            default: a=0; break;
            }
                mark.color.a = a; // Don't forget to set the alpha!
            mark.color.r = r;
            mark.color.g = g;
            mark.color.b = b;

            mark.lifetime = ros::Duration(1);
        }
        ROS_INFO("region : %d", ptr->spat_id_region);
        spat_pub_debug.publish(markAry);
    }
}

void SPaTProcessor::publish(){
    combined_msg.header = boundingBoxAry.header;
    combined_msg.bb_ary = boundingBoxAry;
    combined_msg.obj_ary = detectedObjAry;
    combined_msg_pub.publish(combined_msg);
}