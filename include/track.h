#pragma once


#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose track(Eigen::Vector3d predict_pos, Eigen::Vector4d track_quat, Eigen::Vector3d ee_offset, Eigen::Vector3d iiwa_base_pos, const int iiwa_no);