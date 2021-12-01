#include "../include/block.h"

geometry_msgs::Pose block(Eigen::Vector3d predict_pos, Eigen::Vector4d block_quat, Eigen::Vector3d iiwa_base_pos, const int iiwa_no){
  geometry_msgs::Pose pose;
  int iiwa_sel = 3-2*iiwa_no;

  pose.position.x = (predict_pos[0] - iiwa_base_pos[0])*iiwa_sel;
  pose.position.y = -1*iiwa_base_pos[1]*iiwa_sel -1.0;
  pose.position.z = predict_pos[2];
  pose.orientation.w = block_quat[0];
  pose.orientation.x = block_quat[1];
  pose.orientation.y = block_quat[2];
  pose.orientation.z = block_quat[3];
  return pose;
}