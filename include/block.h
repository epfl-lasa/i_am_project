#pragma once


#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose block(Eigen::Vector3d object_pos, Eigen::Vector3d predict_pos, Eigen::Vector4d block_quat, Eigen::Vector3d iiwa_base_pos, const int iiwa_no);