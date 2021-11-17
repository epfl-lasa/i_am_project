#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>

Eigen::MatrixXd covarUpdate(Eigen::Matrix<double,6,6> P_prior);
Eigen::MatrixXd stateUpdate(Eigen::Vector3d object_pos, Eigen::Matrix<double,6,1> s_prior, Eigen::Matrix<double,6,6> P_pred);