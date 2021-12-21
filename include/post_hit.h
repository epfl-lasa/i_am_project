#pragma once


#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose postHit(const Eigen::Vector3d object_pos_init, const Eigen::Vector4d rest_quat, const Eigen::Vector3d iiwa_base_pos, Eigen::Vector3d iiwa_flip, const int iiwa_no);