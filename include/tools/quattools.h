#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <math.h>
#include <numeric>

Eigen::Vector4d quatProd(Eigen::Vector4d a, Eigen::Vector4d b);
Eigen::Vector3d quatToRPY(Eigen::Vector4d q);
Eigen::Vector4d rpyToQuat(double r, double p, double y);
Eigen::Vector4d pointsToQuat(Eigen::Vector3d point_1, Eigen::Vector3d point_2);