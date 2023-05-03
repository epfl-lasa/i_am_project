#pragma once

#include "ros/ros.h"
#include <Eigen/Dense>
#include <ros/console.h>

// TODOO DELETE
// #include <iostream>
// #include <numeric>
// #include <string>
// #include <vector>
// #include <cstdio>
// #include "math.h"

class hitting_DS {
private:
  float des_speed_ = 2.0;
  float m_obj_ = 0.4;
  float sigma_ = 0.2;// for the flux DS

  Eigen::Vector3f current_position_;
  Eigen::Vector3f des_direction_ = Eigen::Vector3f::Zero(3);// to be updated in the main code of hitting
  Eigen::Vector3f DS_attractor_ = {0.3, 0.3, 0.5};
  Eigen::Matrix3f gain_ = -2.0 * Eigen::Matrix3f::Identity(3, 3);

public:
  hitting_DS(Eigen::Vector3f& current_end_effector);
  hitting_DS(Eigen::Vector3f& current_end_effector, Eigen::Vector3f& attractor_main);
  hitting_DS(Eigen::Vector3f& current_end_effector,
             Eigen::Vector3f& desired_position,
             Eigen::Vector3f& hit_direction,
             float& hit_speed);

  Eigen::Vector3f flux_DS(float dir_flux, Eigen::Matrix3f& current_inertia);
  Eigen::Vector3f linear_DS();
  Eigen::Vector3f linear_DS(Eigen::Vector3f& attractor);
  Eigen::Vector3f vel_max_DS();

  // Getter
  Eigen::Vector3f get_des_direction();
  Eigen::Vector3f get_current_position();
  Eigen::Vector3f get_DS_attractor();
  Eigen::Matrix3f get_gain();

  // Setter
  void set_des_direction(Eigen::Vector3f desired_direction);
  void set_current_position(Eigen::Vector3f current_position);
  void set_DS_attractor(Eigen::Vector3f DS_attractor);
  void set_gain(Eigen::Matrix3f gain);
};
