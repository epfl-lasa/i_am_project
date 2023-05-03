#pragma once

#include "ros/ros.h"
#include <Eigen/Dense>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>

#include "dynamical_system.h"

// TODO
// #include <experimental/filesystem> WHAT IS THIS??
// #include "math.h"
// #include <cstdio>
// #include <fstream>
// #include <iostream>
// #include <numeric>
// #include <string>
// #include <vector>

// #include <Eigen/Geometry>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/JointState.h>
// #include <std_msgs/Float64.h>

class InertiaMotionQP {

private:
  std::string pub_vel_quat_topic_;
  std::string iiwa_position_topic_;
  std::string iiwa_inertia_topic_;

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

  // TODO DELETE NOT NEEDED
  // Eigen::Vector3f iiwa_vel_from_source_;
  // std::string pub_ref_position_topic_;

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