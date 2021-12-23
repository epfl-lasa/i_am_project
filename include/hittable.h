#pragma once

#include <numeric>
#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

std::tuple<bool, bool> hittable(Eigen::Vector3d object_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector4d hittable_params);