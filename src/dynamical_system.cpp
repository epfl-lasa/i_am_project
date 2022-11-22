#include "dynamical_system.h"

hitting_DS::hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &attractor_main){
    current_position = current_end_effector;
    DS_attractor = attractor_main;
}

hitting_DS::hitting_DS(Eigen::Vector3f &current_end_effector){
    current_position = current_end_effector;
}

hitting_DS::hitting_DS(Eigen::Vector3f &current_end_effector, Eigen::Vector3f &desired_position, Eigen::Vector3f& hit_direction, float& hit_speed){
    current_position = current_end_effector;
    DS_attractor = desired_position;
    des_direction = hit_direction;
    des_speed = hit_speed;
}

Eigen::Vector3f hitting_DS::linear_DS(){
    return gain * (current_position - DS_attractor);
}

Eigen::Vector3f hitting_DS::linear_DS(Eigen::Vector3f &attractor){
    return gain * (current_position - attractor);
}

Eigen::Vector3f hitting_DS::flux_DS(float dir_flux, Eigen::Matrix3f& current_inertia){

    /* ** Finding the virtual end effector position ** */

    Eigen::Vector3f reference_velocity = Eigen::Vector3f{0.0, 0.0, 0.0};
    Eigen::Vector3f relative_position = current_position - DS_attractor; 
    Eigen::Vector3f virtual_ee = DS_attractor + des_direction * (relative_position.dot(des_direction) / (des_direction.squaredNorm()));

    float dir_inertia = des_direction.transpose() * current_inertia * des_direction;

    std::cout  << dir_inertia << std::endl;
    float exp_term = (current_position - virtual_ee).norm();
    float alpha = exp(-exp_term/(sigma * sigma));
   
    reference_velocity = alpha * des_direction + (1 - alpha) * gain * (current_position - virtual_ee);

    reference_velocity = (dir_flux / dir_inertia) * (dir_inertia + m_obj) * reference_velocity / reference_velocity.norm();

    
    return reference_velocity;
}

Eigen::Vector3f hitting_DS::vel_max_DS(){
    Eigen::Vector3f reference_velocity = Eigen::Vector3f{0.0, 0.0, 0.0};
    Eigen::Vector3f relative_position = current_position - DS_attractor; 
    Eigen::Vector3f virtual_ee = DS_attractor + des_direction * (relative_position.dot(des_direction) / (des_direction.squaredNorm()));
    float exp_term = (current_position - virtual_ee).norm();
    float alpha = exp(-exp_term/(sigma * sigma));

    reference_velocity = alpha * des_direction + (1 - alpha) * gain * (current_position - virtual_ee);
    reference_velocity = des_speed * reference_velocity / reference_velocity.norm();

    // std::cout << "ref: " << reference_velocity.transpose() << std::endl;

    return reference_velocity;

}
