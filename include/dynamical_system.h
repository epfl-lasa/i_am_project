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
        Eigen::Vector3f DS_attractor = {0.3, 0.3, 0.5};
        Eigen::Vector3f des_direction = Eigen::Vector3f::Zero(3); // to be updated in the main code of hitting
        float des_speed = 0.5;
        
        float m_obj = 0.3;
        
        // for the flux DS
        float sigma = 0.15;

        Eigen::Matrix3f gain = -1.0 * Eigen::Matrix3f::Identity(3,3);

        hitting_DS(Eigen::Vector3f &current_end_effector);
        hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main);
        hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &desired_position, Eigen::Vector3f& hit_direction, float& hit_speed);
        Eigen::Vector3f linear_DS();
        Eigen::Vector3f linear_DS(Eigen::Vector3f &attractor);
        Eigen::Vector3f flux_DS(float dir_flux, Eigen::Matrix3f& current_inertia);
    
};