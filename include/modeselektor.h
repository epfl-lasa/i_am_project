#pragma once

#include "../include/hittable.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
//#include <iostream>


int modeSelektor(Eigen::Vector3d object_pos, Eigen::Vector3d object_vel, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_vel, Eigen::Vector3d object_pos_init, Eigen::Vector3d aim, Eigen::Vector2d ee_offset, const int prev_mode);
