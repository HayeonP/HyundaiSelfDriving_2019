#pragma once
#include <ros/ros.h>
#include <GeographicLib/Geocentric.hpp>
#include <vector>
#include <v2x_processor/convertMap2Baselink.h>

using namespace GeographicLib;

class TransformationMatrixProcessor 
{
public:
    TransformationMatrixProcessor(){}
    TransformationMatrixProcessor(const std::vector<double>& vec) :
        earth(Constants::WGS84_a(), Constants::WGS84_f())
    {
        tf_mat = vec;
        for(size_t i = 0 ; i < vec.size(); ++i)
            ROS_INFO("tf_mat[%ld] : %lf", i , tf_mat[i]);
    }
    void convertLatLonAltToMapXYZ(
        double lat, double lon, double alt, 
        double& x_out, double& y_out, double& z_out
    )
    {
        double x_e, y_e, z_e;
        earth.Forward(lat, lon, alt, x_e, y_e, z_e);

        x_out = tf_mat[0] * x_e + tf_mat[1] * y_e + tf_mat[2] * z_e + tf_mat[3];
        y_out = tf_mat[4] * x_e + tf_mat[5] * y_e + tf_mat[6] * z_e + tf_mat[7];
        z_out = tf_mat[8] * x_e + tf_mat[9] * y_e + tf_mat[10] * z_e + tf_mat[11];
    }
private:
    std::vector<double> tf_mat;
    Geocentric earth;
};
