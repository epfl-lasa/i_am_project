#pragma once

#include "ros/ros.h"
#include <ros/console.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <iostream>

#include "dynamical_system.h"

// TODO
// #include <experimental/filesystem> WHAT IS THIS??

// #include <cstdio>
// #include <numeric>
// #include "math.h"
// #include <vector>
// #include <string>
// #include <fstream>

// #include <sensor_msgs/JointState.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Float64.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <Eigen/Geometry>

class HitMotionSim {
private:
  bool is_hit_ = 0;
  float hit_momentum;

  std::string iiwa_inertia_topic_;
  std::string iiwa_position_topic_;
  std::string object_position_topic_;
  std::string pub_vel_quat_topic_;

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pub_vel_quat_;
  ros::Subscriber object_position_;
  ros::Subscriber iiwa_position_;
  ros::Subscriber iiwa_inertia_;

  Eigen::Vector3f hit_direction_ = {0.0, 1.0, 0.0};
  Eigen::Vector3f ref_velocity_ = {0.0, 0.0, 0.0};
  Eigen::Vector4f ref_quat_ = Eigen::Vector4f::Zero();
  Eigen::Vector3f iiwa_position_from_source_;
  Eigen::Vector3f object_position_from_source_;
  Eigen::Vector4d object_orientation_from_source_;
  Eigen::Matrix3f iiwa_task_inertia_pos_;

  std::unique_ptr<hitting_DS> generate_hitting_ =
      std::make_unique<hitting_DS>(iiwa_position_from_source_, object_position_from_source_);

  geometry_msgs::Pose box_pose_, iiwa_pose_;
  geometry_msgs::Twist iiwa_vel_;

  // TODO DELETE NEVER USED
  // ros::Publisher pub_pos_quat_;

public:
  explicit HitMotionSim(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();
  void run();

  int getIndex(std::vector<std::string> v, std::string value);
  void iiwaInertiaCallback_gazebo(const geometry_msgs::Inertia& inertia_msg);
  void iiwaPositionCallback_gazebo(const gazebo_msgs::LinkStates& link_states);
  void objectPositionCallback_gazebo(const gazebo_msgs::ModelStates& model_states);
  void publishVelQuat(Eigen::Vector3f& DS_vel, Eigen::Vector4f& DS_quat);
  void updateCurrentEEPosition(Eigen::Vector3f& new_position);
  void updateCurrentObjectPosition(Eigen::Vector3f& new_position);

  // TODO DELETE NOT USED
  // void setGains(Eigen::Matrix3f& gain);
  // void publishPosQuat(const Eigen::Vector3f& DS_pos, const Eigen::Vector4f& DS_quat);
};
