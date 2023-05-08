#pragma once

#include "ros/package.h"
#include "ros/ros.h"
#include <ros/console.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <math.h>
#include <numeric>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>

#include "../include/hittable.h"
#include "tools/quattools.h"

#include <experimental/filesystem>
#define _USE_MATH_DEFINES

struct modelProperties {
  double size_x;
  double size_y;
  double size_z;
  double com_x;
  double com_y;
  double com_z;
  double mass;
  double mu;
  double mu2;
  double izz;
};

class CollectData {
private:
  bool object_real_, iiwa_real_, manual_mode_, hollow_;

  int mode1_ = 5;
  int mode2_ = 5;

  struct modelProperties box_;

  double object_th_, object_th_dot_;

  //NEEDED ? DELETE
  //   double object_th_mod_, ETA_, stdev_;
  //   Eigen::Vector3d predict_pos_;

  Eigen::Vector3d object_pos_, object_vel_, object_rpy_, ee1_rpy_, ee2_rpy_;
  Eigen::Vector3d center1_;
  Eigen::Vector3d center2_;
  Eigen::Vector4d hittable_params_;

  Eigen::Matrix3d R_Opti_;

  std::vector<double> iiwa1_joint_angles_, iiwa2_joint_angles_;

  geometry_msgs::Pose object_pose_, ee1_pose_, ee2_pose_;
  geometry_msgs::Twist object_twist_, ee1_twist_, ee2_twist_;

  ros::NodeHandle nh_;
  ros::Rate rate_;

  ros::Subscriber object_subs_;
  ros::ServiceClient set_state_client_;
  ros::Subscriber iiwa1_ee_subs_;
  ros::Subscriber iiwa2_ee_subs_;
  ros::Subscriber iiwa_subs_;
  ros::Subscriber iiwa1_ee_twist_subs_;
  ros::Subscriber iiwa2_ee_twist_subs_;
  ros::Subscriber iiwa1_joint_subs_;
  ros::Subscriber iiwa2_joint_subs_;
  ros::Subscriber estimate_object_subs_;
  ros::Subscriber mode1_sub_, mode2_sub_;

public:
  explicit CollectData(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};
  void init();
  void run();
  int getIndex(std::vector<std::string> v, std::string value);

  //Gazebo callbacks
  void objectSimCallback(const gazebo_msgs::ModelStates model_states);
  void iiwaSimCallback(const gazebo_msgs::LinkStates link_states);

  //Optitrack callbacks
  void objectCallback(const geometry_msgs::Pose object_pose);
  void iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose);
  void iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose);

  //Passive_track (iiwa_toolkit) callbacks
  void iiwa1EETwistCallback(const geometry_msgs::Twist ee_twist);
  void iiwa2EETwistCallback(const geometry_msgs::Twist ee_twist);
  void iiwa1JointCallback(sensor_msgs::JointState joint_states);
  void iiwa2JointCallback(sensor_msgs::JointState joint_states);

  //i_am_predict callbacks
  void estimateObjectCallback(std_msgs::Float64MultiArray estimation);
  void modeIwaa1Callback(std_msgs::Int16 msg);
  void modeIwaa2Callback(std_msgs::Int16 msg);
};