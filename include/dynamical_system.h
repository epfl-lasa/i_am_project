#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>

#include <Eigen/Dense>

class hitting_DS{
    public:
        Eigen::Vector3f current_position;
        Eigen::Vector3f DS_attractor;
        float sigma = 0.01;
        Eigen::Matrix3f gain = Eigen::Matrix3f::Identity(3,3);
    
        hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main);
        Eigen::Vector3f linear_DS();
        Eigen::Vector3f momentum_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &desired_position, Eigen::Vector3f &desired_velocity);
    
};