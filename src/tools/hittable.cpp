#include "../include/hittable.h"

std::tuple<bool, bool> hittable(Eigen::Vector3d object_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector4d hittable_params){
    bool hittable = false;
    bool too_far = false;

    double x_reach = hittable_params[0];
    double y_reach = hittable_params[1];
    double x_offset = hittable_params[2];
    double y_offset = hittable_params[3];
    
    Eigen::Vector3d d_center = center2 - center1;
    Eigen::Vector3d perp_center = {-d_center[1], d_center[0], d_center[2]};
    Eigen::Vector3d center = center1 + y_offset*d_center/d_center.norm() + x_offset*perp_center/perp_center.norm();

    if ( sqrt( pow((object_pos-center).dot(perp_center)/x_reach,2) + pow((object_pos-center).dot(d_center)/y_reach,2) ) <1){
        hittable = true;
    }

    if (object_pos.dot(d_center) < center.dot(d_center)){
        too_far = true;
    }

     
    return std::make_tuple(hittable,too_far);
}