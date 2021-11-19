#pragma once


#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose rest(const Eigen::Vector3d rest_pos, const Eigen::Vector4d rest_quat);