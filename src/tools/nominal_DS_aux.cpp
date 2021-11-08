#include "../include/nominal_DS_aux.h"

Eigen::Vector3d nominal_aux(Eigen::Matrix3d &rotation, Eigen::Matrix3d &gain, Eigen::Vector3d &current_end_effector, Eigen::Vector3d &attractor_aux){
    Eigen::Vector3d velocity_aux = rotation*gain*rotation.transpose()*(current_end_effector - attractor_aux);

    return velocity_aux;
};