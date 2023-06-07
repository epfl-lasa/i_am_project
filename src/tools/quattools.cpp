#include "tools/quattools.h"

Eigen::Vector4d quatProd(Eigen::Vector4d a, Eigen::Vector4d b) {
  Eigen::Vector4d p;
  p[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  p[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  p[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  p[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

  return p;
}

Eigen::Vector3d quatToRPY(Eigen::Vector4d q) {
  Eigen::Vector3d angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  angles[0] = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (std::abs(sinp) >= 1) angles[1] = std::copysign(M_PI / 2, sinp);// use 90 degrees if out of range
  else
    angles[1] = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  angles[2] = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

Eigen::Vector4d rpyToQuat(double r, double p, double y) {
  //Eigen::Vector4d q_base = {std::cos(-M_PI/4), std::sin(-M_PI/4), 0.0, 0.0}; //rotate from base quaternion (ee pointing up) to 'ready to play' rotation

  Eigen::Vector4d q_r = {std::cos(r / 2), std::sin(r / 2), 0.0, 0.0};
  Eigen::Vector4d q_p = {std::cos(p / 2), 0.0, std::sin(p / 2), 0.0};
  Eigen::Vector4d q_y = {std::cos(y / 2), 0.0, 0.0, std::sin(y / 2)};

  //Eigen::Vector4d q = quatProd(q_r, quatProd(q_p, quatProd(q_y,q_base)));     //start with last rotation and then backwards
  Eigen::Vector4d q = quatProd(q_r, quatProd(q_p, q_y));

  return q;
}

Eigen::Vector4d pointsToQuat(Eigen::Vector3d point_1, Eigen::Vector3d point_2) {
  Eigen::Vector4d quat = Eigen::Vector4d::Zero();
  Eigen::Vector3d direction_x;
  Eigen::Vector3d direction_y;
  Eigen::Vector3d direction_z;
  Eigen::Matrix3d rot;
  Eigen::Quaterniond rot_quat;

  direction_z = point_2 - point_1;
  direction_x = {0.0, 0.0, 1.0};
  direction_y = direction_z.cross(direction_x);
  direction_z = direction_z / direction_z.norm();
  direction_y = direction_y / direction_y.norm();
  direction_x = direction_x / direction_x.norm();
  rot.col(0) = direction_x;
  rot.col(1) = direction_y;
  rot.col(2) = direction_z;
  rot_quat = rot;
  quat << rot_quat.w(), rot_quat.x(), rot_quat.y(), rot_quat.z();

  return quat;
}