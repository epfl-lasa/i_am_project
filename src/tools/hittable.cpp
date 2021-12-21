#include "../include/hittable.h"

bool hittable(Eigen::Vector3d object_pos, Eigen::Vector3d iiwa1_base_pos, Eigen::Vector3d iiwa_flip, int iiwa_no){
    int iiwa_sel = 3-2*iiwa_no;
    Eigen::Vector3d flip_vec = Eigen::Vector3d::Ones() + iiwa_flip*(iiwa_sel-1);
    double close_y = 0.1; //y-value closest to iiwa_base. don't confuse with value wrt world frame. This would actually represent the max y
    double far_y = 0.5;
    double close_x = 0.1;
    double far_x = 0.8;
    bool hittable = false;
    
    if (iiwa1_base_pos[1]+close_y < object_pos[1]*flip_vec[1] && object_pos[1]*flip_vec[1] < iiwa1_base_pos[1]+far_y && iiwa1_base_pos[0]+close_x < object_pos[0]*flip_vec[0] && object_pos[0]*flip_vec[0] < iiwa1_base_pos[0]+far_x){
        hittable = true;
    }

     
    return hittable;
}