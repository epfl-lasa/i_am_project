#pragma once

#include "../include/calculate_alpha.h"
#include "../include/nominal_DS_aux.h"
#include "../include/nominal_DS_main.h"
#include "../include/modulated_DS.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose hitDS(double des_speed, double theta, Eigen::Vector3d object_pos, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_pos_init, Eigen::Vector3d iiwa_flip, const int iiwa_no);