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
#include <ros/console.h>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include "dynamical_system.h"

class PushRelease {

private:
  bool is_pushed_ = 0;

  std::string pub_dir_flux_topic_;
  std::string pub_vel_quat_topic_;
  std::string iiwa_base_position_topic_;
  std::string iiwa_inertia_topic_;
  std::string iiwa_position_topic_;
  std::string iiwa_vel_topic_;
  std::string object_position_topic_;

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pub_dir_flux_;
  ros::Publisher pub_vel_quat_;
  ros::Subscriber iiwa_base_position_;
  ros::Subscriber iiwa_inertia_;
  ros::Subscriber iiwa_position_;
  ros::Subscriber iiwa_vel_;
  ros::Subscriber object_position_;

  Eigen::Vector3f hit_direction_ = {0.0, 1.0, 0.0};
  Eigen::Vector3f ref_velocity_ = {0.0, 0.0, 0.0};
  Eigen::Vector4f ref_quat_ = Eigen::Vector4f::Zero();
  Eigen::Vector3f iiwa_base_position_from_source_;
  Eigen::Vector3f iiwa_position_from_source_;
  Eigen::Vector3f object_position_from_source_;
  Eigen::Vector3f object_position_world_;
  Eigen::Vector4f iiwa_base_orientation_from_source_;
  Eigen::Vector4f iiwa_orientation_from_source_;
  Eigen::Vector3f iiwa_vel_from_source_;
  Eigen::Vector4d object_orientation_from_source_;
  Eigen::Matrix3f iiwa_task_inertia_pos_;
  Eigen::Matrix3f rotation_;

  std::unique_ptr<hitting_DS> generate_hitting_ =
      std::make_unique<hitting_DS>(iiwa_position_from_source_, object_position_world_);

  // TOOD NOT NEEDED DELETE
  // float hit_momentum_;
  //   Eigen::Vector3f test_velocity_ = {0.0, 0.0, 0.0};
  //   geometry_msgs::Pose box_pose, iiwa_pose;
  //   geometry_msgs::Twist iiwa_vel;
  //   geometry_msgs::Inertia iiwa_inertia;
  // ros::Publisher pub_pos_quat_;

public:
  PushRelease(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();
  void run();

  void iiwaBasePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void iiwaInertiaCallback(const geometry_msgs::Inertia& inertia_msg);
  void iiwaPositionCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void iiwaVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void objectPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void objectPositionWorldFrame();
  void publishFlux(
      Eigen::Matrix3f& iiwa_task_inertia_pos,
      Eigen::Vector3f&
          iiwa_vel_from_source);
  void publishVelQuat(Eigen::Vector3f& DS_vel, Eigen::Vector4f& DS_quat);
  void updateCurrentEEPosition(Eigen::Vector3f& new_position);
  void updateCurrentObjectPosition(Eigen::Vector3f& new_position);
};