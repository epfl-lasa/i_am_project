#include "../include/block.h"

geometry_msgs::Pose block(Eigen::Vector3d object_pos, Eigen::Vector3d predict_pos, Eigen::Vector4d block_quat, Eigen::Vector3d iiwa_base_pos, const int iiwa_no){
  geometry_msgs::Pose pose;
  int iiwa_sel = 3-2*iiwa_no;

  pose.position.x = (object_pos[0] + (iiwa_base_pos[1]+0.1*iiwa_sel-object_pos[1])*(predict_pos[0]-object_pos[0])/(predict_pos[1]-object_pos[1]) - iiwa_base_pos[0])*iiwa_sel; //calculate x on the predicted trajectory at point of interception with line y = just next to iiwa_base
  pose.position.y = 0.1;
  pose.position.z = predict_pos[2];
  pose.orientation.w = block_quat[0];
  pose.orientation.x = block_quat[1];
  pose.orientation.y = block_quat[2];
  pose.orientation.z = block_quat[3];
  return pose;
}