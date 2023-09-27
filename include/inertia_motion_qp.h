//|
//|    Copyright (C) 2021-2023 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors: Harshit Khurana (maintainer)
//|
//|    email:   harshit.khurana@epfl.ch
//|
//|    website: lasa.epfl.ch
//|
//|    This file is part of iam_dual_arm_control.
//|    This work was supported by the European Community's Horizon 2020 Research and Innovation
//|    programme (call: H2020-ICT-09-2019-2020, RIA), grant agreement 871899 Impact-Aware Manipulation.
//|
//|    iam_dual_arm_control is free software: you can redistribute it and/or modify  it under the terms
//|    of the GNU General Public License as published by  the Free Software Foundation,
//|    either version 3 of the License, or  (at your option) any later version.
//|
//|    iam_dual_arm_control is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#pragma once

#include "ros/ros.h"
#include <Eigen/Dense>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>

#include "dynamical_system.h"

class InertiaMotionQP {

private:
  std::string iiwa_position_topic_;
  std::string iiwa_inertia_topic_;
  std::string pub_vel_quat_topic_;

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pub_ref_position_;
  ros::Publisher pub_vel_quat_;
  ros::Subscriber iiwa_inertia_;
  ros::Subscriber iiwa_position_;

  Eigen::Vector3f ref_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Vector4f ref_quat_ = Eigen::Vector4f::Zero();
  Eigen::Vector3f iiwa_position_from_source_;
  Eigen::Vector4f iiwa_orientation_from_source_;
  Eigen::Matrix3f iiwa_task_inertia_pos_;

  std::unique_ptr<hitting_DS> generate_inertia_motion_ = std::make_unique<hitting_DS>(iiwa_position_from_source_);

public:
  InertiaMotionQP(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();
  void run();

  void iiwaInertiaCallback(const geometry_msgs::Inertia& inertia_msg);
  void iiwaPositionCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void publishJointPosition(const Eigen::VectorXf& joint_pos);
  void publishVelQuat(Eigen::Vector3f& DS_vel, Eigen::Vector4f& DS_quat);
  void setGains(Eigen::Matrix3f& gain);
  void updateCurrentEEPosition(Eigen::Vector3f& new_position);
};