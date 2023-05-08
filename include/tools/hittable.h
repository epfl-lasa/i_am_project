#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <numeric>

std::tuple<bool, bool>
hittable(Eigen::Vector3d object_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector4d hittable_params);