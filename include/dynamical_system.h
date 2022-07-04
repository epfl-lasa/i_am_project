#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>

#include <Eigen/Dense>

Eigen::Vector3f momentum_DS(Eigen::Matrix3d &rotation, Eigen::Matrix3d &gain, Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main);

class hitting_DS{
    private:
        Eigen::Vector3f current_position;
        Eigen::Vector3f linear_DS_attractor;
    
    public:
        hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main);
        Eigen::Vector3f momentum_DS(Eigen::Matrix3d &gain, Eigen::Vector3f &current_end_effector, Eigen::Vector3f &desired_position, Eigen::Vector3f &desired_velocity);
    
};