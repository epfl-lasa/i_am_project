#include "../include/post_hit.h"

geometry_msgs::Pose postHit(const Eigen::Vector3d object_pos_init, const Eigen::Vector4d rest_quat, const Eigen::Vector3d iiwa_base_pos, Eigen::Vector3d iiwa_flip, const int iiwa_no){
  geometry_msgs::Pose pos_quat;
  int iiwa_sel = 3-2*iiwa_no;
  Eigen::Vector3d flip_vec = Eigen::Vector3d::Ones() + iiwa_flip*(iiwa_sel-1);

  pos_quat.position.x = (object_pos_init[0] - iiwa_base_pos[0]);
  pos_quat.position.y = (object_pos_init[1] - iiwa_base_pos[1]);
  pos_quat.position.z = (object_pos_init[2] - iiwa_base_pos[2]);
  pos_quat.orientation.w = rest_quat[0];
  pos_quat.orientation.x = rest_quat[1];
  pos_quat.orientation.y = rest_quat[2];
  pos_quat.orientation.z = rest_quat[3];
  return pos_quat;
}