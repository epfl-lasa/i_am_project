#include "dynamical_system.h"

hitting_DS::hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main){
    current_position = current_end_effector;
    linear_DS_attractor = attractor_main;
}

Eigen::Vector3f hitting_DS::momentum_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &desired_position, Eigen::Vector3f &desired_velocity){

    Eigen::Vector3f relative_position = current_end_effector - desired_position; 
    Eigen::Vector3f virtual_ee = desired_position + desired_velocity * (relative_position.dot(desired_velocity) / (desired_velocity.squaredNorm()));
    float alpha = exp(-(current_end_effector - virtual_ee).norm()/(sigma * sigma));

    Eigen::Vector3f reference_velocity = alpha * desired_velocity + (1 - alpha) * gain * (current_end_effector - virtual_ee);
    return reference_velocity;

};