#include "dynamical_system.h"

hitting_DS::hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main){
    current_position = current_end_effector;
    linear_DS_attractor = attractor_main;
}

Eigen::Vector3f hitting_DS::momentum_DS(Eigen::Matrix3d &gain, Eigen::Vector3f &current_end_effector, Eigen::Vector3f &desired_position, Eigen::Vector3f &desired_velocity){
    
    float inner_product = 
    Eigen::Vector3f virtual_ee = desired_position + 
    alp

};