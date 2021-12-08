#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>

//std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> kalmanFilter(Eigen::Matrix<double,6,6> P_prior, Eigen::Matrix<double,6,1> s_prior, Eigen::Vector3d object_vel);
//std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Vector3d, double> predictPos(Eigen::Matrix<double,6,6> P, Eigen::Matrix<double,6,1> state, Eigen::Vector3d object_pos, Eigen::Vector3d object_vel, double mass);

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> kalmanFilter(Eigen::MatrixXd P_prior, Eigen::VectorXd s_prior, Eigen::VectorXd u, Eigen::VectorXd y, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd W, Eigen::MatrixXd V);
std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::Vector3d, double> predictPos(Eigen::MatrixXd P_obj_lin, Eigen::VectorXd s_obj_lin, Eigen::Vector3d hit_force, Eigen::Vector3d object_pos, double dt, double mass);
//std::tuple<Eigen::MatrixXd, Eigen::VectorXd, double> predictTh(Eigen::MatrixXd P_obj_th, Eigen::VectorXd s_obj_th, double object_th, double dt, double inertia);