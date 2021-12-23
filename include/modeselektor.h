#pragma once

#include "../include/hittable.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
//#include <iostream>


int modeSelektor(Eigen::Vector3d object_pos, Eigen::Vector3d object_vel, Eigen::Vector3d predict_pos, double ETA, Eigen::Vector3d ee_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector4d hittable_params, const int prev_mode);
int maniModeSelektor(Eigen::Vector3d object_pos, Eigen::Vector3d object_vel, Eigen::Vector3d ee_pos,  Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector4d hittable_params, const int prev_mode, const int key_ctrl, const int iiwa_no);