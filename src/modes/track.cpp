#include "../include/track.h"

geometry_msgs::Pose track(Eigen::Vector3d predict_pos, Eigen::Vector4d track_quat, Eigen::Vector3d ee_offset, Eigen::Vector3d iiwa_base_pos, Eigen::Vector3d iiwa_flip, const int iiwa_no){
  geometry_msgs::Pose pos_quat;
  int iiwa_sel = 3-2*iiwa_no;
  Eigen::Vector3d flip_vec = Eigen::Vector3d::Ones() + iiwa_flip*(iiwa_sel-1);
  
  pos_quat.position.x = (predict_pos[0] - iiwa_base_pos[0]) + flip_vec[0]*ee_offset[0];
  pos_quat.position.y = (predict_pos[1] - iiwa_base_pos[1]) + flip_vec[1]*ee_offset[1];
  pos_quat.position.z = (predict_pos[2] - iiwa_base_pos[2]) + flip_vec[2]*ee_offset[2];
  pos_quat.orientation.w = track_quat[0];
  pos_quat.orientation.x = track_quat[1];
  pos_quat.orientation.y = track_quat[2];
  pos_quat.orientation.z = track_quat[3];

  return pos_quat;
}