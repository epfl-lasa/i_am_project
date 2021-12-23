#include "../include/post_hit.h"

geometry_msgs::Pose postHit(Eigen::Vector3d object_pos_init, Eigen::Vector3d center2, Eigen::Vector3d iiwa_base_pos){
  geometry_msgs::Pose pose;

  //Calculate pos and quat wrt to world frame
  Eigen::Vector3d pos_world = object_pos_init;
  Eigen::Vector4d quat_world = pointsToQuat(object_pos_init, center2);

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