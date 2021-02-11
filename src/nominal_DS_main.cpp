#include "../include/nominal_DS_main.h"

Eigen::Vector3d nominal_main(Eigen::Matrix3d &rotation, Eigen::Matrix3d &gain, Eigen::Vector3d &current_end_effector, Eigen::Vector3d &attractor_main){
    Eigen::Vector3d velocity_main = rotation*gain*rotation.transpose()*(current_end_effector - attractor_main);

    return velocity_main;
};