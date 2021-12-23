#include "../include/rest.h"

geometry_msgs::Pose rest(Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector3d iiwa_base_pos){
  geometry_msgs::Pose pose;

  //Calculate pos and quat wrt to world frame
  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};
  Eigen::Vector3d pos_world = center1 - ee_offset[0]*d_center/d_center.norm() + v_offset;

  Eigen::Vector4d quat_world = pointsToQuat(center1, center2); //wrt world frame

  //Transform pos and quat from world to iiwa frame
  Eigen::Vector3d pos_iiwa = pos_world - iiwa_base_pos;
  Eigen::Vector4d quat_iiwa = quat_world;

  pose.position.x = pos_iiwa[0];
  pose.position.y = pos_iiwa[1];
  pose.position.z = pos_iiwa[2];
  pose.orientation.w = quat_iiwa[0];
  pose.orientation.x = quat_iiwa[1];
  pose.orientation.y = quat_iiwa[2];
  pose.orientation.z = quat_iiwa[3];
  return pose;
}