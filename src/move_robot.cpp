#include "move_robot.h"

double MINIMUM_EE_POSE = 0.20;

geometry_msgs::Pose
track(Eigen::Vector3d predict_pos, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector3d iiwa_base_pos) {

  //Calculate pos and quat wrt to world frame
  Eigen::Vector3d d_points = center2 - predict_pos;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};
  Eigen::Vector3d pos_world = predict_pos - ee_offset[0] * d_points / d_points.norm() + v_offset;
  Eigen::Vector4d quat_world = pointsToQuat(predict_pos, center2);//wrt world frame

  //Transform pos and quat from world to iiwa frame
  Eigen::Vector3d pos_iiwa = pos_world - iiwa_base_pos;
  Eigen::Vector4d quat_iiwa = quat_world;

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

geometry_msgs::Pose
rest(Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector3d iiwa_base_pos) {

  //Calculate pos and quat wrt to world frame
  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};
  Eigen::Vector3d pos_world = center1 - ee_offset[0] * d_center / d_center.norm() + v_offset;
  Eigen::Vector4d quat_world = pointsToQuat(center1, center2);//wrt world frame

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

geometry_msgs::Pose postHit_DS(Eigen::Vector3d object_pos_init,
                            Eigen::Vector3d center2,
                            Eigen::Vector3d iiwa_base_pos,
                            Eigen::Vector2d ee_offset) {
  Eigen::Vector3d offset = {0.0, 0.0, ee_offset[1]};
  object_pos_init += offset;
  center2 += offset;

  //Calculate pos and quat wrt to world frame
  Eigen::Vector3d pos_world = object_pos_init;
  Eigen::Vector4d quat_world = pointsToQuat(object_pos_init, center2);

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

geometry_msgs::Pose postHit(Eigen::Vector3d object_pos_init,
                            Eigen::Vector3d center2,
                            Eigen::Vector3d iiwa_base_pos,
                            Eigen::Vector2d ee_offset) {
  Eigen::Vector3d offset = {0.0, 0.0, ee_offset[1]};
  object_pos_init += offset;
  center2 += offset;

  //Calculate pos and quat wrt to world frame
  Eigen::Vector3d pos_world = object_pos_init;
  Eigen::Vector4d quat_world = pointsToQuat(object_pos_init, center2);

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

geometry_msgs::Pose hitDSInertia(double des_speed,
                                 Eigen::Vector3d object_pos,
                                 Eigen::Vector3d center2,
                                 Eigen::Vector3d ee_pos,
                                 Eigen::Vector2d ee_offset,
                                 Eigen::Matrix3d& current_inertia) {

  Eigen::Vector3d reference_velocity = Eigen::Vector3d{0.0, 0.0, 0.0};
  double theta = -std::atan2(center2[0] - object_pos[0], center2[1] - object_pos[1]);//angle between object and target
  Eigen::Vector3d object_offset = {0.0, 0.0, ee_offset[1]};
  Eigen::Vector3d des_direction = {0.0, 1.0, 0.0};
  Eigen::Matrix3d gain, rot_mat;

  // gain << -0.9, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.9;
  gain = -2.0 * Eigen::Matrix3d::Identity(3, 3);
  float sigma = 0.2;
  float m_obj = 0.4;
  float dir_flux = 0.5;

  object_pos += object_offset;
  rot_mat << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  des_direction = rot_mat * des_direction;
  Eigen::Vector3d ds_attractor = object_pos + des_direction;

  Eigen::Vector3d relative_position = ee_pos - ds_attractor;
  Eigen::Vector3d virtual_ee =
      ds_attractor + des_direction * (relative_position.dot(des_direction) / (des_direction.squaredNorm()));

  float dir_inertia = des_direction.transpose() * current_inertia * des_direction;
  ROS_INFO_STREAM("Dir_inertia: " << dir_inertia);

  float exp_term = (ee_pos - virtual_ee).norm();
  float alpha = exp(-exp_term / (sigma * sigma));
  reference_velocity =
      alpha * des_direction + (1 - alpha) * rot_mat * gain * rot_mat.transpose() * (ee_pos - virtual_ee);

  reference_velocity =
      (dir_flux / dir_inertia) * (dir_inertia + m_obj) * reference_velocity / reference_velocity.norm();

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

geometry_msgs::Pose hitDS(double des_speed,
                          Eigen::Vector3d object_pos,
                          Eigen::Vector3d center2,
                          Eigen::Vector3d ee_pos,
                          Eigen::Vector3d ee_pos_init,
                          Eigen::Vector2d ee_offset) {

  //instead of flipping direction of attractor and such, make use of the fact that iiwa2 is exactly the same as iiwa1 but rotated around the worlds z-axis with pi
  //if then the only parameters that are defined w.r.t. the world frame are rotated also, the whole situation becomes identical
  //(passive_track gives cmds w.r.t. respective iiwa frame)

  double modulated_sigma = 3.0;
  double theta = -std::atan2(center2[0] - object_pos[0], center2[1] - object_pos[1]);//angle between object and target

  Eigen::Vector3d object_offset = {0.0, 0.0, ee_offset[1]};//{0.0, 0.0, 0.025};
  Eigen::Vector3d unit_x = {0.0, 1.0, 0.0};
  Eigen::Vector3d vel_world = {0.0, 0.0, 0.0};
  Eigen::Matrix3d gain_main, gain_aux, rot_mat;
  Eigen::Vector3d attractor_main, attractor_aux;

  object_pos += object_offset;
  gain_main << -0.9, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.9;
  gain_aux << -0.1, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.1;
  rot_mat << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  attractor_main = object_pos + 1 * rot_mat * unit_x;
  attractor_aux = (4.0 / 3.0) * object_pos - (1.0 / 3.0) * attractor_main;

  double alpha = calculate_alpha(ee_pos, ee_pos_init, object_pos, attractor_main);

  vel_world = alpha * nominal_aux(rot_mat, gain_aux, ee_pos, attractor_aux)
      + (1 - alpha) * nominal_main(rot_mat, gain_main, ee_pos, attractor_main)
      + modulated_DS(attractor_main, object_pos, ee_pos, modulated_sigma);

  vel_world = vel_world * des_speed / vel_world.norm();

  Eigen::Vector4d quat_world = pointsToQuat(object_pos, attractor_main);

  //Transform vel and quat from world to iiwa frames
  Eigen::Vector3d vel_iiwa = vel_world;
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

geometry_msgs::Pose block(Eigen::Vector3d object_pos,
                          Eigen::Vector3d predict_pos,
                          Eigen::Vector3d center1,
                          Eigen::Vector3d center2,
                          double object_th_mod,
                          Eigen::Vector3d iiwa_base_pos) {

  //The ratio between {the projections of {{the iiwa_base} and {the predicted position} wrt to the current position} on the vector through the centerpoints} is equal to
  //the ratio between {the intersection point of {the trajectory} and {the line crossing the iiwa_base perpendicular to the vector through centers} wrt the current object_pos} and {the predicted position wrt to the current position of the object}
  //This intersection point is precisely where the EE should stop the object and is therefore calculated as follows:
  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d traject = predict_pos - object_pos;
  double traject_on_d_center = traject.dot(d_center);
  double iiwa_offset_on_d_center = (iiwa_base_pos - object_pos).dot(d_center) + 0.1;
  Eigen::Vector3d pos_world = object_pos + iiwa_offset_on_d_center / traject_on_d_center * traject;
  Eigen::Vector4d rest_quat = pointsToQuat(center1, center2);
  Eigen::Vector4d quat_world = quatProd(rpyToQuat(0.0, 0.0, object_th_mod), rest_quat);

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

//   ------------------------------------------------ UTILS ------------------------------------------------
double calculate_alpha(Eigen::Vector3d& end_effector_position,
                       const Eigen::Vector3d& end_effector_init,
                       const Eigen::Vector3d& object_position,
                       Eigen::Vector3d& attractor_main) {

  Eigen::Vector3d normal_vector = attractor_main - object_position;
  Eigen::Vector3d initial_vector = end_effector_init - object_position;
  Eigen::Vector3d current_vector = end_effector_position - object_position;

  Eigen::Vector3d initial_vector_projection =
      initial_vector - initial_vector.dot(normal_vector) * normal_vector / (normal_vector.squaredNorm());
  Eigen::Vector3d current_vector_projection =
      current_vector - current_vector.dot(normal_vector) * normal_vector / (normal_vector.squaredNorm());

  double initial_vector_norm = initial_vector_projection.norm();
  double current_vector_norm = current_vector_projection.norm();
  double alpha;

  if (initial_vector_norm == 0) {
    alpha = 0;
  } else {
    alpha = current_vector_norm / initial_vector_norm;
  }

  return alpha;
};

Eigen::Vector3d modulated_DS(Eigen::Vector3d& attractor_main,
                             const Eigen::Vector3d& object_position_init,
                             Eigen::Vector3d& current_end_effector,
                             double sigma) {

  Eigen::Vector3d projection = object_position_init
      + (current_end_effector - object_position_init).dot(attractor_main - object_position_init)
          * (attractor_main - object_position_init) / (attractor_main - object_position_init).squaredNorm();

  double rbf_kernel = exp(-(current_end_effector - projection).squaredNorm() / (sigma * sigma));

  Eigen::Vector3d perp = (current_end_effector - object_position_init)
      - (current_end_effector - object_position_init).dot(attractor_main - object_position_init)
          * (attractor_main - object_position_init) / (attractor_main - object_position_init).squaredNorm();

  Eigen::Vector3d velocity_modulated;
  velocity_modulated = -rbf_kernel * perp;

  return velocity_modulated;
};

Eigen::Vector3d nominal_aux(Eigen::Matrix3d& rotation,
                            Eigen::Matrix3d& gain,
                            Eigen::Vector3d& current_end_effector,
                            Eigen::Vector3d& attractor_aux) {
  Eigen::Vector3d velocity_aux = rotation * gain * rotation.transpose() * (current_end_effector - attractor_aux);

  return velocity_aux;
};

Eigen::Vector3d nominal_main(Eigen::Matrix3d& rotation,
                             Eigen::Matrix3d& gain,
                             Eigen::Vector3d& current_end_effector,
                             Eigen::Vector3d& attractor_main) {
  Eigen::Vector3d velocity_main = rotation * gain * rotation.transpose() * (current_end_effector - attractor_main);

  return velocity_main;
};
