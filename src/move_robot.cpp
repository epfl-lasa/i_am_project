#include "move_robot.h"

double MINIMUM_EE_POSE = 0.20;

geometry_msgs::Pose postHit(Eigen::Vector3d object_pos_init,
                            Eigen::Vector3d center,
                            Eigen::Vector3d iiwa_base_pos,
                            Eigen::Vector2d ee_offset) {
                              
  Eigen::Vector3d offset = {0.0, 0.0, ee_offset[1]};
  object_pos_init += offset;
  center += offset;

  //Calculate pos and quat wrt to world frame
  Eigen::Vector3d pos_world = object_pos_init;
  Eigen::Vector4d quat_world = pointsToQuat(object_pos_init, center);

  //Transform pos and quat from world to iiwa frame
  Eigen::Vector3d pos_iiwa = pos_world - iiwa_base_pos;
  Eigen::Vector4d quat_iiwa = quat_world;

  if (pos_iiwa[2] < MINIMUM_EE_POSE) { pos_iiwa[2] = MINIMUM_EE_POSE; }

  geometry_msgs::Pose pose;
  pose.position.x = pos_iiwa[0];
  pose.position.y = pos_iiwa[1];
  pose.position.z = pos_iiwa[2];
  pose.orientation.w = quat_iiwa[0];
  pose.orientation.x = quat_iiwa[1];
  pose.orientation.y = quat_iiwa[2];
  pose.orientation.z = quat_iiwa[3];

  return pose;
}

geometry_msgs::Pose hitDSInertia(double dir_flux,
                                 Eigen::Vector3d object_pos,
                                 Eigen::Vector3d center,
                                 Eigen::Vector3d ee_pos,
                                 Eigen::Vector2d ee_offset,
                                 Eigen::Matrix3d& current_inertia) {

  Eigen::Vector3d reference_velocity = Eigen::Vector3d{0.0, 0.0, 0.0};
  double theta = -std::atan2(center[0] - object_pos[0], center[1] - object_pos[1]);//angle between object and target
  Eigen::Vector3d object_offset = {0.0, 0.0, ee_offset[1]};
  Eigen::Vector3d des_direction = {0.0, 1.0, 0.0};
  Eigen::Matrix3d gain, rot_mat;

  // gain << -0.9, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.9;
  gain = -2.0 * Eigen::Matrix3d::Identity(3, 3);
  float sigma = 0.2;
  float m_obj = 0.4;  

  object_pos += object_offset;
  rot_mat << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  des_direction = rot_mat * des_direction;
  Eigen::Vector3d ds_attractor = object_pos + des_direction;

  Eigen::Vector3d relative_position = ee_pos - ds_attractor;
  Eigen::Vector3d virtual_ee = ds_attractor + des_direction * (relative_position.dot(des_direction) / (des_direction.squaredNorm()));

  float dir_inertia = des_direction.transpose() * current_inertia * des_direction;
  ROS_INFO_STREAM("Dir_inertia: " << dir_inertia);

  float exp_term = (ee_pos - virtual_ee).norm();
  float alpha = exp(-exp_term / (sigma * sigma));

  reference_velocity = alpha * des_direction + (1 - alpha) * rot_mat * gain * rot_mat.transpose() * (ee_pos - virtual_ee);
  reference_velocity = (dir_flux / dir_inertia) * (dir_inertia + m_obj) * reference_velocity / reference_velocity.norm();

  Eigen::Vector4d quat_world = pointsToQuat(object_pos, ds_attractor);

  //Transform vel and quat from world to iiwa frames
  Eigen::Vector3d vel_iiwa = reference_velocity;
  Eigen::Vector4d quat_iiwa = quat_world;

  geometry_msgs::Pose vel_quat;
  vel_quat.position.x = vel_iiwa[0];
  vel_quat.position.y = vel_iiwa[1];
  vel_quat.position.z = vel_iiwa[2];
  vel_quat.orientation.w = quat_iiwa[0];
  vel_quat.orientation.x = quat_iiwa[1];
  vel_quat.orientation.y = quat_iiwa[2];
  vel_quat.orientation.z = quat_iiwa[3];

  return vel_quat;
}