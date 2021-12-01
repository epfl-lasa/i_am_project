#include "../include/rest.h"

geometry_msgs::Pose rest(const Eigen::Vector3d rest_pos, const Eigen::Vector4d rest_quat){
  geometry_msgs::Pose pos_quat;
  pos_quat.position.x = rest_pos[0];
  pos_quat.position.y = rest_pos[1];
  pos_quat.position.z = rest_pos[2];
  pos_quat.orientation.w = rest_quat[0];
  pos_quat.orientation.x = rest_quat[1];
  pos_quat.orientation.y = rest_quat[2];
  pos_quat.orientation.z = rest_quat[3];
  return pos_quat;
}