#pragma once

#include "../include/quattools.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose block(Eigen::Vector3d object_pos, Eigen::Vector3d predict_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, double object_th_mod, Eigen::Vector3d iiwa_base_pos);