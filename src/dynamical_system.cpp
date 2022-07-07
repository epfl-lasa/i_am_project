#include "dynamical_system.h"

hitting_DS::hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main){
    current_position = current_end_effector;
    DS_attractor = attractor_main;
}

hitting_DS::hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &desired_position, Eigen::Vector3f &desired_velocity){
    current_position = current_end_effector;
    DS_attractor = desired_position;
    des_velocity = desired_velocity;
}

Eigen::Vector3f hitting_DS::linear_DS(){
    return -gain * (current_position - DS_attractor);
}

Eigen::Vector3f hitting_DS::momentum_DS(){

    Eigen::Vector3f relative_position = current_position - DS_attractor; 
    Eigen::Vector3f virtual_ee = DS_attractor + des_velocity * (relative_position.dot(des_velocity) / (des_velocity.squaredNorm()));
    float alpha = exp(-(current_position - virtual_ee).norm()/(sigma * sigma));

    Eigen::Vector3f reference_velocity = alpha * des_velocity + (1 - alpha) * gain * (current_position - virtual_ee);
    return reference_velocity;

};


