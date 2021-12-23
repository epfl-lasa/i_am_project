#pragma once

#include "../include/calculate_alpha.h"
#include "../include/nominal_DS_aux.h"
#include "../include/nominal_DS_main.h"
#include "../include/modulated_DS.h"

#include "../include/quattools.h"

#include <numeric>
#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose hitDS(double des_speed, Eigen::Vector3d object_pos, Eigen::Vector3d center2, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_pos_init);