#include "../include/block.h"

geometry_msgs::Pose block(Eigen::Vector3d predict_pos, Eigen::Vector4d rest_quat, Eigen::Vector3d iiwa_base_pos, const int iiwa_no){
  geometry_msgs::Pose pos_quat;
  int iiwa_sel = 3-2*iiwa_no;

  pos_quat.position.x = (predict_pos[0] - iiwa_base_pos[0])*iiwa_sel;
  pos_quat.position.y = -1*iiwa_base_pos[1]*iiwa_sel -1.0;
  pos_quat.position.z = predict_pos[2];
  pos_quat.orientation.x = rest_quat[0];
  pos_quat.orientation.y = rest_quat[1];
  pos_quat.orientation.z = rest_quat[2];
  pos_quat.orientation.w = rest_quat[3];
  return pos_quat;
}