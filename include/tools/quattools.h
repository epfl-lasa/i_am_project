#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <math.h>
#include <numeric>

// Quaternion multiplication a*b
Eigen::Vector4d quatProd(Eigen::Vector4d a, Eigen::Vector4d b);

// Quaternion to euler angle
Eigen::Vector3d quatToRPY(Eigen::Vector4d q);

// Euler angle to quaternion
Eigen::Vector4d rpyToQuat(double r, double p, double y);

// Computer quaternion from line that goes from point_1 to point_2
Eigen::Vector4d pointsToQuat(Eigen::Vector3d point_1, Eigen::Vector3d point_2);