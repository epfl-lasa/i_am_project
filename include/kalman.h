#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> kalmanFilter(Eigen::Matrix<double,6,6> P_prior, Eigen::Matrix<double,6,1> s_prior, Eigen::Vector3d object_vel);
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Vector3d, double> predictPos(Eigen::Matrix<double,6,6> P, Eigen::Matrix<double,6,1> state, Eigen::Vector3d object_pos, Eigen::Vector3d object_vel, double mass);