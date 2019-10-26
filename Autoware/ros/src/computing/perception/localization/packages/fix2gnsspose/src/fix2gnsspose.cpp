/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <GeographicLib/Geocentric.hpp>

using namespace GeographicLib;
//#include <gnss/geo_pos_conv.hpp>

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;
static double x, y, z;
static double _matrix[3][4];

static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
  Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
  earth.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
	
  double x_dash = _matrix[0][0] * x + _matrix[0][1] * y + _matrix[0][2] * z + _matrix[0][3];
  double y_dash = _matrix[1][0] * x + _matrix[1][1] * y + _matrix[1][2] * z + _matrix[1][3];
  double z_dash = _matrix[2][0] * x + _matrix[2][1] * y + _matrix[2][2] * z + _matrix[2][3];

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  // pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x_dash;
  pose.pose.position.y = y_dash;
  pose.pose.position.z = z_dash;

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) + pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));

  if (distance > 5.0)
  {
    yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _prev_pose = pose;
  }

  pose.pose.orientation = _quat;
  pose_publisher.publish(pose);
  stat_publisher.publish(gnss_stat_msg);

  //座標変換
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2gnsspose");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("a1", _matrix[0][0]);
  private_nh.getParam("b1", _matrix[0][1]);
  private_nh.getParam("c1", _matrix[0][2]);
  private_nh.getParam("d1", _matrix[0][3]);
  private_nh.getParam("a2", _matrix[1][0]);
  private_nh.getParam("b2", _matrix[1][1]);
  private_nh.getParam("c2", _matrix[1][2]);
  private_nh.getParam("d2", _matrix[1][3]);
  private_nh.getParam("a3", _matrix[2][0]);
  private_nh.getParam("b3", _matrix[2][1]);
  private_nh.getParam("c3", _matrix[2][2]);
  private_nh.getParam("d3", _matrix[2][3]);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe("fix", 100, GNSSCallback);

  ros::spin();
  return 0;
}
